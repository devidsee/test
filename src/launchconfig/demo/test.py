#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('robot_state_publisher', anonymous=True)
      
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        joint_goal = arm.get_current_joint_values()

        import re

        file = open('/home/wangyabin/catkin_ws/src/launchconfig/6AxisGCodeTranslator/a.txt','r')
        gcode = file.readlines()
        for line in gcode:
            # print(line)
            coord = re.split(r'[ ]', line)
            if coord:
                print("{} - {}- {}- {}- {}".format(coord[1].split('X')[1], coord[2].split('Y')[1], coord[3].split('Z')[1], coord[4].split('A')[1], coord[5].split('B')[1]))
                joint_goal[0] = coord[4].split('A')[1]
                joint_goal[1] = coord[1].split('X')[1]
                joint_goal[2] = coord[2].split('Y')[1]
                joint_goal[3] = coord[3].split('Z')[1]
                joint_goal[4] = coord[5].split('B')[1]
                joint_goal[5] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
                arm.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        arm.stop()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
