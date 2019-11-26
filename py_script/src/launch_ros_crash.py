#!/usr/bin/env python
import sys
import os
import roslaunch
import rospy


def get_moveit_path():
    home_dir = os.path.expanduser('~')
    catkin_dir = home_dir + '/catkin_ws/src'
    ros_sim_dir = catkin_dir + '/humanoid_shp_moveit'
    launch_file = '/launch/humanoid_shp_bringup_moveit.launch'

    return ros_sim_dir+launch_file


def launch_simulator(launch_file_path):
#    rospy.init_node('humanoid', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    launch.start()
#    rospy.loginfo("top module started")

def launch_node(pkg, py_script, node_name, args):
    node = roslaunch.core.Node(package=pkg, node_type=py_script, name=node_name, args=args, output='screen')
    launch_node = roslaunch.scriptapi.ROSLaunch()
    launch_node.start()
    process = launch_node.launch(node)


def main(cmd_pose):
    launch_file_path = get_moveit_path()
    launch_simulator(launch_file_path)
    rospy.sleep(1)
    launch_node(pkg='py_script', py_script='humanoid2_crash.py', node_name='humanoid', args=cmd_pose)
    rospy.sleep(25)
    launch.shutdown()

if __name__ == '__main__':
    main(sys.argv[1])
