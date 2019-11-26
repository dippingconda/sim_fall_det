#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import sys
import copy
import os
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import pyassimp

import json
from collections import OrderedDict
from fdetSQLite import FallDetSQLite

now = time.strftime("%Y-%m-%d_%H:%M:%S")
time_dict = {'time':now}

json_list = []
json_dict = {}


def makeLogPath():
    home_dir = os.path.expanduser('~')
    if (os.path.exists(home_dir+'/imu_log')==False):
        os.mkdir(home_dir+'/'+'imu_log')
    return home_dir+'/imu_log'

#def make_json_file(to_json_list):
#    home_dir = os.path.expanduser('~')
#    now = getDateTime()
#    to_json_list.append(str(now))
#    if(os.path.exists(home_dir+'/imu_log/imu_data.json')):
#        with open(home_dir+'/imu_log/imu_data.json') as ori_data:
#            data = json.load(ori_data)
#        data.update(to_json_list)
#        with open(home_dir+'/imu_log/imu_data.json', 'w') as fwrite:
#            json.dump(to_json_list, fwrite, indent=4)
#    else:
#        with open(home_dir+'/imu_log/imu_data.json', 'w') as fwrite:
#            json.dump(to_json_list, fwrite, indent=4)

def make_json_file(to_json):
    home_dir = os.path.expanduser('~')

    if(os.path.exists(home_dir+'/imu_log/imu_data.json')):
        with open(home_dir+'/imu_log/imu_data.json') as ori_data:
            old_data = json.load(ori_data)
#         data.update(to_json)
        to_json = old_data + to_json
#        to_json.append(old_data)
#        print to_json
        with open(home_dir+'/imu_log/imu_data.json', 'w') as fwrite:
            json.dump(to_json, fwrite, indent=4)
    else:
        with open(home_dir+'/imu_log/imu_data.json', 'w') as fwrite:
            json.dump(to_json, fwrite, indent=4)


def getDateTime():
    return time.strftime("%Y-%m-%d_%H:%M:%S")

def loadEnv():
    rospy.init_node("humanoid", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander(robot_description="robot_description", ns="humanoid_shp")
    scene = moveit_commander.PlanningSceneInterface()
    group_list = robot.get_group_names()
    """
    group_names = ["body", "left_ankle", "left_arm", "left_leg", "left_wrist",
                    "right_ankle", "right_arm", "right_leg", "right_wrist"]
    """
    group_names = {"body": group_list[0], "left_ankle": group_list[1], "left_arm": group_list[2],"left_leg": group_list[3],
                    "left_wrist": group_list[4],
                    "right_ankle": group_list[5], "right_arm": group_list[6],"right_leg": group_list[7],
                    "right_wrist": group_list[8]}
    return robot, scene, group_names

def callMoveGroupCommander(group):
    groupCmd = moveit_commander.MoveGroupCommander(group)
    return groupCmd

def displayTrajectoryPublisher():
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    return display_trajectory_publisher

def displayTrajectory(robot, plan, display_trajectory_publisher):
    humanoid_shp = robot
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = humanoid_shp.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(1)

def getCurrentjointValues(move_group):
    print move_group.get_current_joint_values()
    return move_group.get_current_joint_values()

def getEndeffectorList(move_group):
    """
    body - neck
    left_arm - left_elbow, left_leg - left_knee, left_ankle - left_ankle
    right_arm - right_elbow, right_leg - right_knee, right_ankle - right_ankle
    """
    eef_list = []
    for move_gr in move_group:        
        eef_link = move_gr.get_end_effector_link()
        eef_link_pose = move_gr.get_current_pose()
        print move_gr.get_name()+" : "+eef_link+" "+str(eef_link_pose.pose)
        eef_list.append(eef_link)
    return eef_list

def getGroupsOfCommanders(joint_groups):
    group_names = joint_groups
    body_commander = callMoveGroupCommander(group_names["body"])
    left_arm_commander = callMoveGroupCommander(group_names["left_arm"])
    left_leg_commander = callMoveGroupCommander(group_names["left_leg"])
    left_ankle_commander = callMoveGroupCommander(group_names["left_ankle"])
    right_arm_commander = callMoveGroupCommander(group_names["right_arm"])
    right_leg_commander = callMoveGroupCommander(group_names["right_leg"])
    right_ankle_commander = callMoveGroupCommander(group_names["right_ankle"])
    commanderList = [body_commander, left_arm_commander, left_leg_commander, left_ankle_commander,
                    right_arm_commander, right_leg_commander, right_ankle_commander]

    return commanderList

def setMax_acc_vel_scalingFactors(commanderLists):
    for commander  in commanderLists:
        commander.set_max_acceleration_scaling_factor(1)
        commander.set_max_velocity_scaling_factor(1)


def getJointGroupInfo(commanderLists):
    joint_values_dict = {}
    for commander in commanderLists:
        joint_values_list = getCurrentjointValues(commander)
        joint_values_dict.update({commander.get_name(): joint_values_list})
        print commander.get_name()+" current joint values : "+str(joint_values_list)
        print commander.get_name()+" goal joint tolerance : "+str(commander.get_goal_joint_tolerance())
#        eefCurrentPoses.append(getCurrentPose(commander))

    return joint_values_dict

def setJointValue(move_group, joint_valueList):
    jointList = move_group.get_joints()
    jointName = move_group.get_name()
    print jointName+" is composed of "+str(jointList)
    print jointName+" = "+str(joint_valueList)
    move_group.set_goal_joint_tolerance(0.005)
    move_group.set_joint_value_target(joint_valueList)

def setRightLegJointValue(move_groups, joint_valueList):
    setJointValue(move_groups[5], joint_valueList[0:2])
    setJointValue(move_groups[6], [joint_valueList[2]])
    planning_groups = [move_groups[5], move_groups[6]]
    for move_gr in planning_groups:
#        move_gr.set_planning_time(5)
        move_gr.set_goal_joint_tolerance(0.003)
        plan = move_gr.go(wait=True)
#        move_gr.stop()
        move_gr.clear_pose_targets()

def setLeftLegJointValue(move_groups, joint_valueList):
    setJointValue(move_groups[3], [joint_valueList[2]])
    setJointValue(move_groups[2], joint_valueList[0:2])
    planning_groups = [move_groups[2], move_groups[3]]
    for move_gr in planning_groups:
#        move_gr.set_planning_time(5)
        move_gr.set_goal_joint_tolerance(0.003)
        plan = move_gr.go(wait=True)
#        move_gr.stop()
        move_gr.clear_pose_targets()

def setBodyJointValue(move_groups, joint_valueList):
    setJointValue(move_groups[0], joint_valueList[0:2])
    planning_groups = [move_groups[0]]
    for move_gr in planning_groups:
#        move_gr.set_planning_time(5)
        move_gr.set_goal_joint_tolerance(0.003)
        plan = move_gr.go(wait=True)
#        move_gr.stop()
        move_gr.clear_pose_targets()

def allJointControl(move_groups):
    #Right Leg
    setJointValue(move_groups[5], [0.03, -0.49])
    setJointValue(move_groups[6], [0])
    #Left Leg
    setJointValue(move_groups[3], [-0.25])
    setJointValue(move_groups[2], [-0.25, 0])
    #Body
    setJointValue(move_groups[0], [-0.10, 0])
    planning_groups = [move_groups[0], 
                        move_groups[2], move_groups[3],
                        move_groups[5], move_groups[6]]
    for move_gr in planning_groups:
        move_gr.set_planning_time(5)
        move_gr.set_goal_joint_tolerance(0.003)
        plan = move_gr.go(wait=True)
#       move_gr.stop()
        move_gr.clear_pose_targets()

def allJointControl_2(move_groups):
    #Right Leg
    setJointValue(move_groups[5], [0.40, -0.49])
    setJointValue(move_groups[6], [0.25])
    #Left Leg
    setJointValue(move_groups[3], [0.25])
    setJointValue(move_groups[2], [-0.25, 0])
    #Body
    setJointValue(move_groups[0], [-0.20, 0])
    planning_groups = [move_groups[0], 
                        move_groups[5], move_groups[2],
                        move_groups[3], move_groups[6]
                        ]
    for move_gr in planning_groups:
        move_gr.set_goal_tolerance(0.003)
        move_gr.set_num_planning_attempts(2)
        move_gr.set_planning_time(0.05)
        plan = move_gr.go(wait=True)
#       move_gr.stop()
#        move_gr.clear_pose_targets()

def allJointControl_3(move_groups):
    #Right Leg
    setJointValue(move_groups[5], [0.35, -0.40])
    setJointValue(move_groups[6], [0.25])
    #Left Leg
    setJointValue(move_groups[3], [0.25])
    setJointValue(move_groups[2], [-0.25, 0])
    #Body
    setJointValue(move_groups[0], [-0.20, 0])
    planning_groups = [move_groups[0], 
                        move_groups[2], move_groups[5],
                        move_groups[3], move_groups[6]
                        ]
    for move_gr in planning_groups:
#        move_gr.set_goal_tolerance(0.003)
#        move_gr.set_num_planning_attempts(2)
#        move_gr.set_planning_time(0.05)
        plan = move_gr.go(wait=True)
        move_gr.stop()
        move_gr.clear_pose_targets()

def allJointControl_4(move_groups):
    #Right Leg
    setJointValue(move_groups[5], [0.50, -0.49])
    setJointValue(move_groups[6], [0.25])
    #Left Leg
    setJointValue(move_groups[3], [-0.25])
    setJointValue(move_groups[2], [-0.25, 0])
    #Body
#    setJointValue(move_groups[0], [-0.20, 0])
    setJointValue(move_groups[0], [-0.00, 0])
    #Left Arm
    setJointValue(move_groups[1], [0.25, 0.55])
    planning_groups = [move_groups[1], 
                        move_groups[0],
                        move_groups[5], move_groups[2]#,
                        #move_groups[3]#, move_groups[6]
                        ]
    for move_gr in planning_groups:
#        move_gr.set_goal_tolerance(0.003)
        move_gr.set_num_planning_attempts(2)
#        move_gr.set_planning_time(0.05)
        plan = move_gr.go(wait=True)
#        move_gr.stop()
#        move_gr.clear_pose_targets()

#    rospy.sleep(2)
# 2nd pose
    #Right Leg
    setJointValue(move_groups[5], [0.17, 0.00])
    #Left Leg
    setJointValue(move_groups[3], [-0.25])
    setJointValue(move_groups[2], [0.25, 0.0])
    planning_groups = [#move_groups[3],
                       move_groups[5], move_groups[2]]
    for move_gr in planning_groups:
#        move_gr.set_goal_tolerance(0.003)
        move_gr.set_num_planning_attempts(2)
#        move_gr.set_planning_time(0.05)
        plan = move_gr.go(wait=True)
#        move_gr.stop()
#        move_gr.clear_pose_targets()

def imuCallback(msg):
    print "---------------------------------------------"
    rospy.loginfo(
            "\nangular velocity x = %s\tangular velocity y = %s\tangular velocity z = %s\n"+\
            "linear acceleration x = %s\tlinear acceleration y = %s\tlinear acceleration z = %s", 
            str(msg.angular_velocity.x), str(msg.angular_velocity.y), str(msg.angular_velocity.z),\
               str(msg.linear_acceleration.x) ,str(msg.linear_acceleration.y), str(msg.linear_acceleration.z)
    )
    json_list.append({'time' : now,\
        'label' : 'y',\
        'angular_velocity_x' : str(msg.angular_velocity.x),\
        'angular_velocity_y' : str(msg.angular_velocity.y),\
        'angular_velocity_z' : str(msg.angular_velocity.z),\
        'linear_acceleration_x' : str(msg.linear_acceleration.x),\
        'linear_acceleration_y' : str(msg.linear_acceleration.y),\
        'linear_acceleration_z' : str(msg.linear_acceleration.z)})
   
        

def main():
    try:
        imu_log_path = makeLogPath()
        cTime = getDateTime()
        rospy.sleep(5)
        print "main function"
        eefs_current_pose = []
        humanoid_shp, scene, group_names = loadEnv()
#        display_trajectory_publisher = displayTrajectoryPublisher()
        commanderLists = getGroupsOfCommanders(group_names)
        setMax_acc_vel_scalingFactors(commanderLists)
        getEndeffectorList(commanderLists)
        joint_values_dict = getJointGroupInfo(commanderLists)
        
        sub_imu = rospy.Subscriber('/imu', Imu, imuCallback)

#        setBodyJointValue(commanderLists, [-0.10, 0])
#        setLeftLegJointValue(commanderLists, [-0.25, 0, -0.25])
#        setRightLegJointValue(commanderLists, [0.40, -0.49, 0])

#        allJointControl(commanderLists)
        allJointControl_4(commanderLists)

#        right_leg_plan = commanderLists[5].plan()
#        right_ankle_plan = commanderLists[6].plan()
#        displayTrajectory(humanoid_shp, right_leg_plan, display_trajectory_publisher)
#        displayTrajectory(humanoid_shp, right_ankle_plan, display_trajectory_publisher)
        rospy.sleep(5)
#        setLeftLegJointValue(commanderLists, [-0.25, 0, -0.24])
#        left_leg_plan = commanderLists[2].plan()
#        left_ankle_plan = commanderLists[3].plan()
#        displayTrajectory(humanoid_shp, left_leg_plan, display_trajectory_publisher)
#        displayTrajectory(humanoid_shp, left_ankle_plan, display_trajectory_publisher)
#        setRightLegJointValue(commanderLists, [0.15, -0.06, 0.15])
#        right_leg_plan = commanderLists[5].plan()
#        right_ankle_plan = commanderLists[6].plan()
#        displayTrajectory(humanoid_shp, right_leg_plan, display_trajectory_publisher)
#        displayTrajectory(humanoid_shp, right_ankle_plan, display_trajectory_publisher)

        make_json_file(json_list)

        return 0
    except rospy.ROSInterruptException:
        print "ROS Interrupt"
        return 0
    except KeyboardInterrupt:
        print "Keyboard Interrupt"
        return 0
    finally:
        print("Program has been terminated")

if __name__ == "__main__":
    main()
