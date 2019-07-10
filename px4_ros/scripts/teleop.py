#!/usr/bin/env python

# /mavros /setpoint_velocity/cmd_vel
# geomety_msgs/TwistStamped

import rospy
import numpy as np
import math
from pynput.keyboard import Key, Listener

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, State
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


class teleOpCtrl():
    def __init__(self):

        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.local_position = PoseStamped()
        self.state = State()
        self.mav_type = None

        self.local_velocity = TwistStamped()

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'local_pos', 'state']
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)


        # Subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_body',
                                              TwistStamped,
                                              self.local_vel_callback)

        # Publishers
        self.desired_vel = TwistStamped()
        self.desired_vel.twist.angular.z = 0
        self.desired_vel.twist.linear.x = 0
        self.desired_vel.twist.linear.y = 0
        self.desired_vel.twist.linear.z = 0

        #self.radius = 0.3
        #self.yaw_th = 0.05
        self.yaw_current = 0
        self.isLand = False
        self.yawrate = 0.5
        self.xvel = 0.5

        self.z_target = 0
        self.z_in = 0.1
        self.z_ub = 2.5  # Height upper bound
        self.z_lb = 0.5  # Height lower bound
        self.z_p_gain = 1.0
        self.cmd_vel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_vel, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # Keyboard input thread
        self.key_thread = Thread(target=self.keyInput, args=())
        self.key_thread.daemon = True
        self.key_thread.start()


    def send_vel(self):
        rate = rospy.Rate(30)  # Hz
        self.desired_vel.header = Header()
        self.desired_vel.header.frame_id = "base_link"

        #while not rospy.is_shutdown():
        while not self.isLand:
            # Z position control
            # P control
            z_error = self.z_target - self.local_position.pose.position.z
            self.desired_vel.twist.linear.z = self.z_p_gain * z_error
            self.desired_vel.header.stamp = rospy.Time.now()
            self.cmd_vel_pub.publish(self.desired_vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass




    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def global_position_callback(self, data):
        self.global_position = data
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data
        local_quat = self.local_position.pose.orientation
        eulers = euler_from_quaternion([local_quat.x, local_quat.y, local_quat.z, local_quat.w])
        self.yaw_current = eulers[2]
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def extended_state_callback(self, data):
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def local_vel_callback(self, data):
        self.local_velocity = data


    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)


    def landing(self):
        rate = rospy.Rate(30)
        landing_condition = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        self.set_mode_srv(custom_mode="AUTO.LAND")
        while(self.extended_state.landed_state != landing_condition):
            rate.sleep()



    def on_press(self, key):
        try:
            if key.char == 'r' :
                self.z_target += 0.05
                if self.z_target > self.z_ub:
                    self.z_target = self.z_ub

            elif key.char == 'f':
                self.z_target -= 0.05
                if self.z_target < self.z_lb:
                    self.z_target = self.z_lb

        except:
            if key == Key.up:
                self.desired_vel.twist.linear.x = self.xvel * np.cos(self.yaw_current)
                self.desired_vel.twist.linear.y = self.xvel * np.sin(self.yaw_current)
            elif key == Key.down:
                self.desired_vel.twist.linear.x = -self.xvel * np.cos(self.yaw_current)
                self.desired_vel.twist.linear.y = -self.xvel * np.sin(self.yaw_current)

            elif key == Key.left:
                self.desired_vel.twist.angular.z = self.yawrate
            elif key == Key.right:
                self.desired_vel.twist.angular.z = -self.yawrate


    def on_release(self, key):

        if key == Key.up:
            self.desired_vel.twist.linear.x = 0
            self.desired_vel.twist.linear.y = 0
        elif key == Key.down:
            self.desired_vel.twist.linear.x = 0
            self.desired_vel.twist.linear.y = 0

        elif key == Key.left:
            self.desired_vel.twist.angular.z = 0
        elif key == Key.right:
            self.desired_vel.twist.angular.z = 0

        elif key == Key.esc:
            # Stop listener
            return False


    def keyInput(self):
        with Listener(on_press=self.on_press,
                      on_release=self.on_release) as listener:
            listener.join()
        self.isLand = True




def main():
    rospy.init_node('cmdvel_node', anonymous=True)
    print("Node Started")
    agent = teleOpCtrl()
    print("Class Loaded")
    agent.wait_for_topics(20)
    print("Topic Ready")

    # Takeoff 1.5 m
    agent.z_target = 1.5

    rate = rospy.Rate(30)

    while not agent.isLand:
        rate.sleep()

    agent.landing()
    agent.set_arming_srv(False)


if __name__ == '__main__':
    main()

