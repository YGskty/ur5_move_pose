#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('move_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    group = MoveGroupCommander("manipulator")

    # specify the planner
    group.set_planner_id("RRTkConfigDefault")

    rospy.sleep(3)

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.

    # display_trajectory_publisher = rospy.Publisher(
    #                                   '/move_group/display_planned_path',
    #                                   moveit_msgs.msg.DisplayTrajectory)

    # print ">>>>> remove scenes"

    # # clean the scene
    # scene.remove_world_object("pole")
    # scene.remove_world_object("ball")
    # scene.remove_world_object("table")
    # scene.remove_world_object("ground_plane")
    # rospy.sleep(5)

    print ">>>>> add scenes"
    # # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    # p.pose.position.x = 0
    # p.pose.position.y = -1
    # p.pose.position.z = 0.2
    # p.pose.orientation.w = 1.0
    # scene.add_box("pole", p, (0.4, 0.1, 0.4))

    # p.pose.position.x = 0
    # p.pose.position.y = 0.8
    # p.pose.position.z = 0.1
    # p.pose.orientation.w = 1.0
    # scene.add_sphere("ball", p, 0.1)

    # p.pose.position.x = 0
    # p.pose.position.y = 0
    # p.pose.position.z = -0.1
    # scene.add_box("table", p, (1.0, 2.6, 0.2))

    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.4
    scene.add_plane("ground_plane", p)

    rospy.sleep(20)

    ## We can get the name of the reference frame for this robot
    print ">>>>> Reference frame: %s" % group.get_planning_frame()
    print ">>>>> Printing robot state"
    print robot.get_current_state()

    print ">>>>> Printing robot pose"
    print group.get_current_pose()

    ## Planning to a Pose goal
    print ">>>>> Generating plan"
    pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0
    pose_target.position.x = 0.5
    pose_target.position.y = 0.2
    pose_target.position.z = 0.2
    group.set_pose_target(pose_target)

    plan = group.plan()

    #print "============ Waiting while RVIZ displays plan..."
    rospy.sleep(1)

    print ">>>>> Go for plan"
    group.go(wait=True)

    ## Adding/Removing Objects and Attaching/Detaching Objects

    collision_object = moveit_msgs.msg.CollisionObject()

    moveit_commander.roscpp_shutdown()

    rospy.spin()
    roscpp_shutdown()
