
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class PosCtrl:
    def __init__(self):

        # For simulation with burger
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.burger_pos = Pose()
        self.burger_vel = Twist()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.maxvel = 0.5
        self.maxyaw = 0.2
        self.yaw_threshold = 0.001
        self.dist_threshold = 0.1
        self.dist_err = 0
        self.yaw_err = 0
        self.yaw_flag = False
        self.pos_flag = False


        self.desired_vel = Twist()
        self.desired_vel.linear.x = 0
        self.desired_vel.linear.y = 0
        self.desired_vel.angular.z = 0

        # x,y only
        # Yaw first toward to destination
        self.target_pos = [0, 0]
        self.current_pos = [0, 0]
        self.current_yaw = 0
        self.current_vel = [0, 0]
        self.current_yawrate = 0


        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()


    def send_vel(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            target_yaw = np.arctan2(self.target_pos[1] - self.current_pos[1],
                                    self.target_pos[0] - self.current_pos[0])
            self.yaw_err = target_yaw - self.current_yaw

            self.dist_err = np.linalg.norm(np.array(self.target_pos) - np.array(self.current_pos))

            linvel = np.linalg.norm(np.array(self.current_vel))



            if np.abs(self.yaw_err) >= self.yaw_threshold and self.yaw_flag:
                self.desired_vel.linear.x = 0
                self.pos_flag = False
                if np.abs(self.yaw_err) >= self.maxyaw:
                    self.desired_vel.angular.z = np.sign(self.yaw_err) * self.maxyaw
                else:
                    self.desired_vel.angular.z = self.yaw_err
            elif np.abs(self.yaw_err) < self.yaw_threshold:
                self.yaw_flag = False
                self.pos_flag = True


            if self.dist_err >= self.dist_threshold and self.yaw_flag==False and self.pos_flag:
                self.desired_vel.angular.z = 0
                self.yaw_flag = False
                if self.dist_err >= self.maxvel:
                    self.desired_vel.linear.x = self.maxvel
                else:
                    self.desired_vel.linear.x = self.dist_err
            elif self.dist_err < self.dist_threshold:
                self.pos_flag = False

            if not self.yaw_flag:
                self.desired_vel.angular.z = 0
            if not self.pos_flag:
                self.desired_vel.linear.x = 0



            self.cmd_pub.publish(self.desired_vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


    def gazebo_callback(self, data):
        temp = data
        self.burger_pos = temp.pose[-1]
        self.burger_vel = temp.twist[-1]
        self.current_pos[0] = self.burger_pos.position.x
        self.current_pos[1] = self.burger_pos.position.y
        self.current_yaw = euler_from_quaternion([self.burger_pos.orientation.x,
                                                  self.burger_pos.orientation.y,
                                                  self.burger_pos.orientation.z,
                                                  self.burger_pos.orientation.w])[2]

        self.current_vel[0] = self.burger_vel.linear.x
        self.current_vel[1] = self.burger_vel.linear.y
        self.current_yawrate = self.burger_vel.angular.z



    def is_at_position(self):
        dist_err = np.linalg.norm(np.array(self.target_pos) - np.array(self.current_pos))

        return dist_err < self.dist_threshold




def main():
    rospy.init_node('turtlebot_posctrl', anonymous=True)

    posctrl = PosCtrl()

    rate = 30
    loop = rospy.Rate(rate)

    # wait 3 secs
    for _ in range(3*rate):
        loop.sleep()

    posctrl.target_pos = [5,3]

    while(not rospy.is_shutdown()):
        print(posctrl.yaw_err, posctrl.dist_err)
        loop.sleep()
    #posctrl.target_pos = posctrl.current_pos



if __name__ == '__main__':
    main()



