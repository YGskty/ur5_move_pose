#!/usr/bin/env python

"""reveive the pose for ur5 to achieve a line.

node : node_receive_pose
publish topic : topic_pose

@author: zhw
@time: 2016/7/20

"""

import sys
import rospy
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander

def callback(data):
    rospy.loginfo('reveive the pose of x for ur5 is %s', data.position.x)

    print ">>>>> Printing robot last pose"
    print group.get_current_pose()

    ## Planning to a Pose goal
    print ">>>>> Generating plan"
    #pose_target.orientation.w = 1.0
    group.set_pose_target(data)
    plan = group.plan()
    print "============ Waiting while RVIZ displays plan..."
    rospy.sleep(0.5)

    print ">>>>> Go for plan"
    group.go(wait=True)

    print ">>>>> end current control <<<<<"


if __name__ == '__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('node_receive_pose', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    group = MoveGroupCommander("manipulator")

    # specify the planner
    group.set_planner_id("RRTkConfigDefault")
    rospy.sleep(0.5)

    rospy.Subscriber('topic_pose', Pose, callback)

    print ">>>>> Printing robot initial pose"
    print group.get_current_pose()

    rospy.spin()

    roscpp_shutdown()
