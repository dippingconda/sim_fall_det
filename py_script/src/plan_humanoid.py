
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import pyassimp


def loadEnv():
    rospy.init_node("humanoid")
    moveit_commander.roscpp_initialize(sys.argv)
#    rospy.init_node("humanoid", anonymous=True)
    robot = moveit_commander.RobotCommander(robot_description="robot_description", ns="humanoid_shp")
#    moveit_commander.move_group.MoveGroupCommander.place("robot_description", [0,0,0])
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