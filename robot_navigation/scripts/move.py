#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random

def random_goal():
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = random.uniform(-5, 5)  # 10m x 10m area around the robot
    goal.target_pose.pose.position.y = random.uniform(-5, 5)
    goal.target_pose.pose.orientation.w = 1.0  # Facing forward

    return goal

def main():
    rospy.init_node("random_goal_generator")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # Set the duration in seconds (100 seconds)
    duration = 100.0
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        goal = random_goal()
        rospy.loginfo("Sending goal: ({}, {})".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Goal reached!")

        # Check if the duration has passed
        if rospy.get_time() - start_time >= duration:
            break

    # Stop the node gracefully
    rospy.signal_shutdown("Script completed")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
