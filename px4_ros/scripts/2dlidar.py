#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class Lidar:
    def __init__(self):

        self.lidar_data = LaserScan()
        self.filtered_data = np.zeros(360)
        #self.filtered_data = np.arange(360)
        self.candidates = []
        #self.scantest = []
        self.lidar_scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.isScan = False

        self.dotres = []
        self.dotmax = None
        self.forward_bound = 120
        self.max_forward = []
        self.foward_list = None

        # For simulation with burger
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.burger_pos = Pose()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.maxvel = 0.3
        self.desired_vel = Twist()
        self.desired_vel.linear.x = 0
        self.desired_vel.linear.y = 0
        self.desired_vel.angular.z = 0

        self.waypoint = [0, 0, 0, 0]
        self.desired_rpos = [0, 0, 0, 0]
        self.target_pos = [0, 0, 0, 0]
        self.current_pos = [0, 0, 0, 0]
        self.maxyaw = 0.15
        self.yaw_threshold = 0.01
        self.dist_threshold = 0.1
        self.increment = 1.0
        self.drone_width = 0.65


        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.thread_flag = False

        self.control_flag = False




    def send_vel(self):
        rate = rospy.Rate(30)  # Hz

        while not rospy.is_shutdown():

            yaw_error = self.desired_rpos[0] - euler_from_quaternion([self.burger_pos.orientation.x,
                                                                                  self.burger_pos.orientation.y,
                                                                                  self.burger_pos.orientation.z,
                                                                                  self.burger_pos.orientation.w])[2]

            self.current_pos[0] = euler_from_quaternion([self.burger_pos.orientation.x,
                                                                                  self.burger_pos.orientation.y,
                                                                                  self.burger_pos.orientation.z,
                                                                                  self.burger_pos.orientation.w])[2]
            self.current_pos[1] = self.burger_pos.position.x
            self.current_pos[2] = self.burger_pos.position.y
            self.current_pos[3] = self.burger_pos.position.z

            self.target_pos[0] = self.current_pos[0] + self.desired_rpos[0]
            self.target_pos[1] = self.current_pos[1] + self.increment*np.cos(self.desired_rpos[0])
            self.target_pos[2] = self.current_pos[2] + self.increment*np.sin(self.desired_rpos[0])


            ldist = self.rel_distance(self.target_pos)

            if self.control_flag:
                if abs(yaw_error) >= self.yaw_threshold:

                    if abs(yaw_error) >= self.maxyaw:
                        yaw_in = np.sign(yaw_error)*self.maxyaw
                    else:
                        yaw_in = self.maxyaw * yaw_error/self.maxyaw

                    self.desired_vel.angular.z = yaw_in
                    self.desired_vel.linear.x = 0

                elif ldist >= self.dist_threshold:
                    if ldist >= self.maxvel:
                        self.desired_vel.linear.x = self.maxvel
                    else:
                        self.desired_vel.linear.x = self.maxvel * ldist/self.maxvel

                    self.desired_vel.angular.z = 0

                else:
                    self.desired_vel.angular.z = 0
                    self.desired_vel.linear.x = 0

            else:
                self.desired_vel.angular.z = 0
                self.desired_vel.linear.x = 0



            self.cmd_pub.publish(self.desired_vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass



    def gazebo_callback(self, data):
        temp = data
        self.burger_pos = temp.pose[-1]
        if self.thread_flag is False:
            self.thread_flag = True
            self.vel_thread.start()



    def lidar_callback(self, data):
        self.lidar_data = data

        for i in range(len(self.lidar_data.ranges)):
            if self.lidar_data.ranges[i] == 0.0 or self.lidar_data.ranges[i] == np.inf:
                self.filtered_data[i] = 3.5
            else:
                self.filtered_data[i] = self.lidar_data.ranges[i]
        self.isScan = True
        #self.scan_possible()



    def scan_possible(self, dist=1.0, limit=0.65, ret=False):
        temp = np.arctan2(dist, limit)
        theta = np.pi/2 - temp
        theta_deg = np.rad2deg(theta)
        theta_int = int(math.ceil(theta_deg))


        possible_head = []
        for i in range(360):
            if i < theta_int:
                scan_pos = self.filtered_data[:theta_int+i+1]
                scan_neg = self.filtered_data[-(theta_int-i):]
                scan = np.concatenate((scan_neg, scan_pos), axis=None)


            elif i >= theta_int and i < 360-theta_int:
                scan = self.filtered_data[(i-theta_int):(i+theta_int+1)]

            else:
                scan_pos = self.filtered_data[(i-theta_int):]
                scan_neg = self.filtered_data[:theta_int-(360-i)+1]
                scan = np.concatenate((scan_pos, scan_neg), axis=None)

            #self.scantest.append(scan)
            min_dist = np.amin(scan)
            if min_dist >= dist:
                avg_dist = np.mean(scan)
                possible_head.append([i, avg_dist])

        self.candidates = possible_head

        if ret:
            return possible_head



    def selection(self, dpos):
        # dpos[0] = orientation (z)
        dposi_rel = np.array([dpos[1] - self.burger_pos.position.x,
                              dpos[2] - self.burger_pos.position.y,
                              dpos[3] - self.burger_pos.position.z])

        dori_rel = np.array(dpos[0] - self.burger_pos.orientation.z)

        alpha = np.arctan2(dposi_rel[1], dposi_rel[0])
        alpha_deg = np.rad2deg(alpha)

        forward = []
        dotprod = []

        for i in range(len(self.candidates)):

            beta_deg = alpha_deg - self.candidates[i][0]
            beta = np.deg2rad(beta_deg)
            dot = self.candidates[i][1] * np.cos(beta)
            dotprod.append([self.candidates[i][0], dot])

            if self.candidates[i][0] >= 181:
                temp_deg = self.candidates[i][0] - 360
            else:
                temp_deg = self.candidates[i][0]

            if temp_deg >= 0 and temp_deg <= (alpha_deg + self.forward_bound/2):
                forward.append([temp_deg, self.candidates[i][1]])
            elif temp_deg < 0 and temp_deg >= (alpha_deg - self.forward_bound/2):
                forward.append([temp_deg, self.candidates[i][1]])

        self.dotres = np.array(dotprod)

        forward = np.array(forward)
        self.foward_list = forward

        #max_idx = np.argmax(forward[:][1])
        #self.max_forward = forward[max_idx]

        max_idx = np.argmax(self.dotres[:][1])
        self.dotmax = self.dotres[max_idx]


    # dpos always [yaw, x, y, z]
    def rel_distance(self, dpos):
        dist = ((self.burger_pos.position.x - dpos[1])**2 + (self.burger_pos.position.x - dpos[2])**2)**0.5
        return dist



    def burger_mv(self):
        self.scan_possible(dist=self.increment)
        self.selection(self.waypoint)
        self.desired_rpos[0] = np.deg2rad(self.dotmax[0])
        self.desired_rpos[1] = self.increment
        self.control_flag = True

    def is_at_position(self, d_pos):
        """offset: meters"""
        desired = np.array(d_pos[1:3])
        pos = np.array([self.burger_pos.position.x,
                        self.burger_pos.position.y])

        return np.linalg.norm(desired - pos) < self.dist_threshold







def main():
    rospy.init_node('lidar_test', anonymous=True)

    test = Lidar()

    rate = rospy.Rate(5)

    # Wait 3 secs for sensor ready
    wait = 3 * 5
    for _ in range(wait):
        rate.sleep()

    target1 = [0, 2, 2, 0]

    test.waypoint = target1

    while(not test.is_at_position(target1)):
        test.burger_mv()
        rate.sleep()
    test.control_flag == False


if __name__ == '__main__':
    main()