#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
from pymavlink import mavutil
import rospkg
from offboard_comm import *
from dronet_ros import *


def main():
    rospy.init_node('setpoint_node', anonymous=True)

    print("node started")
    agent = OffboardCtrl()
    print("Offboard Class loaded")
    agent.wait_for_topics(20)
    print("Topic loaded")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('px4_ros')
    json_path = pkg_path + '/models/model_struct.json'
    weights_path = pkg_path + '/models/best_weights.h5'
    dronet = zedRosDronet(json_path=json_path, weights_path=weights_path)

    loop_rate = rospy.Rate(30)

    # agent.set_mode_srv(custom_mode='OFFBOARD')
    # agent.set_arming_srv(True)

    # pose
    # Yaw, X, Y, Z
    pos_takeoff = [0, 0, 0, 1.5]

    agent.halt()
    print("Set desired as current position - Init")

    # 3 sec hovering
    hov_time = 30  # 10 Hz update

    agent.move_to(pos_takeoff)
    while (agent.is_at_position(pos_takeoff, agent.radius) == False):
        loop_rate.sleep()
    print("Take off Done")


    for _ in range(hov_time):
        loop_rate.sleep()


    while not dronet.isLand:
        target = dronet.prediction()
        agent.dronet_mv(target)
        loop_rate.sleep()


    landing_condition = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    agent.set_mode_srv(custom_mode="AUTO.LAND")
    while(agent.extended_state.landed_state != landing_condition):
        loop_rate.sleep()

    print("Landing done")

    agent.set_arming_srv(False)



if __name__ == '__main__':
    main()
