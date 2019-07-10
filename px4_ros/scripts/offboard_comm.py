#!/usr/bin/env python
# ROS python API
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, State
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


class OffboardCtrl:
    def __init__(self):

        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.local_position = PoseStamped()
        self.state = State()
        self.mav_type = None

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

        # Publisher
        self.desired_pos = PoseStamped()
        self.desired_pos.pose.position.x = 0
        self.desired_pos.pose.position.y = 0
        self.desired_pos.pose.position.z = 0
        self.radius = 0.3
        self.yaw_th = 0.05
        self.z_in = 0.1
        self.z_ub = 2.7  # Height upper bound
        self.z_lb = 0.5  # Height lower bound
        self.yaw_test=0

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        #self.pos_thread = Thread(target=self.send_pos, args=())
        #self.pos_thread.daemon = True
        #self.pos_thread.start()
	# ----------------               -------------------#

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

        self.x_target = 0
        self.y_target = 0
        self.z_target = 0
        self.p_gain = 1.0
        self.yaw_p_gain = 0.5
        self.cmd_vel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

        self.vel_thread = Thread(target=self.send_vel_from_pos, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()



    def send_pos(self):
        rate = rospy.Rate(30)  # Hz
        self.desired_pos.header = Header()
        self.desired_pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.desired_pos.header.stamp = rospy.Time.now()

            self.pos_setpoint_pub.publish(self.desired_pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #--------------  pos to vel -------------------#
    def send_vel_from_pos(self):
        rate = rospy.Rate(30)  # Hz
        self.desired_vel.header = Header()
        self.desired_vel.header.frame_id = "base_link"

        #while not rospy.is_shutdown():
        while not self.isLand:

            x_error = self.desired_pos.pose.position.x - self.local_position.pose.position.x
            y_error = self.desired_pos.pose.position.y - self.local_position.pose.position.y
            z_error = self.desired_pos.pose.position.z - self.local_position.pose.position.z



            desired_quat = self.desired_pos.pose.orientation
            desired_eulers = euler_from_quaternion([desired_quat.x, desired_quat.y, desired_quat.z, desired_quat.w])
            desired_yaw = desired_eulers[2]

            local_quat = self.local_position.pose.orientation  
            local_eulers = euler_from_quaternion([local_quat.x, local_quat.y, local_quat.z, local_quat.w])   
            local_yaw = local_eulers[2]       

            yaw_error = desired_yaw - local_yaw

            self.desired_vel.twist.linear.x = self.p_gain * x_error
            self.desired_vel.twist.linear.y = self.p_gain * y_error	
            self.desired_vel.twist.linear.z = self.p_gain * z_error
            self.desired_vel.twist.angular.z = self.yaw_p_gain * yaw_error

            if self.desired_vel.twist.linear.x >= 0.5: self.desired_vel.twist.linear.x  = 0.5
            elif self.desired_vel.twist.linear.x <= -0.5: self.desired_vel.twist.linear.x  = -0.5

            if self.desired_vel.twist.linear.y >= 0.5: self.desired_vel.twist.linear.y  = 0.5
            elif self.desired_vel.twist.linear.y <= -0.5: self.desired_vel.twist.linear.y  = -0.5

            if self.desired_vel.twist.linear.z >= 0.5: self.desired_vel.twist.linear.z  = 0.5
            elif self.desired_vel.twist.linear.z <= -0.5: self.desired_vel.twist.linear.z  = -0.5
            
            self.desired_vel.header.stamp = rospy.Time.now()
            self.cmd_vel_pub.publish(self.desired_vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass



    def halt(self):
        self.desired_pos.pose.position.x = self.local_position.pose.position.x
        self.desired_pos.pose.position.y = self.local_position.pose.position.y
        self.desired_pos.pose.position.z = self.local_position.pose.position.z


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

    def is_at_position(self, d_pos, offset):
        """offset: meters"""
        desired = np.array(d_pos[1:4])
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))

        return np.linalg.norm(desired - pos) < offset

    def is_at_orientation(self, d_yaw, offset):
        """offset : radian"""
        local_quat = self.local_position.pose.orientation
        eulers = euler_from_quaternion([local_quat.x, local_quat.y, local_quat.z, local_quat.w])
        yaw_local = eulers[2] #radians

        print( "desired_yaw:",math.radians(d_yaw), "local_yaw:",yaw_local)
        if abs(yaw_local - math.radians(d_yaw)) <= offset:
            return False
        return True

    def move_to(self, d_pos):
        """timeout(int): seconds"""
        # set a position setpoint
        self.desired_pos.pose.position.x = d_pos[1]
        self.desired_pos.pose.position.y = d_pos[2]
        self.desired_pos.pose.position.z = d_pos[3]

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = d_pos[0]  # North
        yaw = math.radians(yaw_degrees)
        self.yaw_test = yaw_degrees
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.desired_pos.pose.orientation = Quaternion(*quaternion)



    def dronet_mv_yaw(self, rel_pos):
        # Yaw control first
        yaw = rel_pos[0]
        x = rel_pos[1]
        y = rel_pos[2]
        z = rel_pos[3]

        """
        current_angles = euler_from_quaternion(self.local_position.pose.orientation)
        yaw_current = current_angles[2]

        yaw_target = yaw_current + yaw
        """
        yaw_target1 = yaw
        yaw_target2 = np.arctan2(y, x)
        yaw_target = yaw_target1 * 0.5 + yaw_target2 * 0.5
        quaternion = quaternion_from_euler(0, 0, yaw_target)
        self.desired_pos.pose.orientation = Quaternion(*quaternion)

        loop_freq = 30  # Hz
        rate = rospy.Rate(loop_freq)
        yaw_reached = False
        while not yaw_reached:
            local_quat = self.local_position.pose.orientation
            eulers = euler_from_quaternion([local_quat.x, local_quat.y, local_quat.z, local_quat.w])
            yaw_local = eulers[2]
            # print("yaw loop")
            if abs(yaw_local - yaw_target) <= self.yaw_th:
                yaw_reached = True

            rate.sleep()
        # print("Yaw done")

        current_position = self.local_position.pose.position

        x_target = current_position.x + x
        y_target = current_position.y + y
        z_target = current_position.z + z

        if z_target > self.z_ub:
            z_target = self.z_ub
        elif z_target < self.z_lb:
            z_target = self.z_lb

        # Z control
        self.desired_pos.pose.position.z = z_target
        reached_z = False
        while not reached_z:
            desired = np.array((z_target))
            cur_pos = np.array((self.local_position.pose.position.z))

            zerror = np.linalg.norm(desired - cur_pos)
            # print("Z loop")

            if zerror < self.z_in:
                reached_z = True

            rate.sleep()
        # print("Z done")

        # XY control
        self.desired_pos.pose.position.x = x_target
        self.desired_pos.pose.position.y = y_target
        reached_xy = False
        while not reached_xy:
            desired = np.array((x_target, y_target))
            cur_pos = np.array((self.local_position.pose.position.x,
                                self.local_position.pose.position.y))

            xyerror = np.linalg.norm(desired - cur_pos)
            # print("XY loop")

            if xyerror < self.radius:
                reached_xy = True

            rate.sleep()
        # print("XY done")

    def dronet_mv(self, rel_pos):

        x = rel_pos[1]
        y = rel_pos[2]
        z = rel_pos[3]

        yaw_target = np.arctan2(y, x)
        quat = quaternion_from_euler(0, 0, yaw_target)
        self.desired_pos.pose.orientation = Quaternion(*quat)

        loop_freq = 30  # Hz
        rate = rospy.Rate(loop_freq)
        yaw_reached = False
        while not yaw_reached:
            local_quat = self.local_position.pose.orientation
            eulers = euler_from_quaternion([local_quat.x, local_quat.y, local_quat.z, local_quat.w])
            yaw_local = eulers[2]
            # print("yaw loop")
            if abs(yaw_local - yaw_target) <= self.yaw_th:
                yaw_reached = True

            rate.sleep()
        # print("Yaw done")

        current_position = self.local_position.pose.position

        x_target = current_position.x + x
        y_target = current_position.y + y
        z_target = current_position.z + z

        if z_target > self.z_ub:
            z_target = self.z_ub
        elif z_target < self.z_lb:
            z_target = self.z_lb

        # Z control
        self.desired_pos.pose.position.z = z_target
        reached_z = False
        while not reached_z:
            desired = np.array((z_target))
            cur_pos = np.array((self.local_position.pose.position.z))

            zerror = np.linalg.norm(desired - cur_pos)
            # print("Z loop")

            if zerror < self.z_in:
                reached_z = True

            rate.sleep()
        # print("Z done")

        # XY control
        self.desired_pos.pose.position.x = x_target
        self.desired_pos.pose.position.y = y_target
        reached_xy = False
        while not reached_xy:
            desired = np.array((x_target, y_target))
            cur_pos = np.array((self.local_position.pose.position.x,
                                self.local_position.pose.position.y))

            xyerror = np.linalg.norm(desired - cur_pos)
            # print("XY loop")

            if xyerror < self.radius:
                reached_xy = True

            rate.sleep()
        # print("XY done")

    def reach_position(self, d_pos, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.desired_pos.pose.position.x = d_pos[0]
        self.desired_pos.pose.position.y = d_pos[1]
        self.desired_pos.pose.position.z = d_pos[2]
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = d_pos[3]  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.desired_pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 30  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(d_pos, self.radius):
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
