#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal generateRandomGoal() {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Generate random coordinates within a 10m x 10m area around the robot
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(-5.0, 5.0);
    goal.target_pose.pose.position.x = distribution(gen);
    goal.target_pose.pose.position.y = distribution(gen);
    
    // Set the orientation to face forward
    goal.target_pose.pose.orientation.w = 1.0;

    return goal;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_goal_generator");

    MoveBaseClient ac("move_base", true);

    // Wait for the action server to start
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

    double duration = 100.0;  // Set the duration in seconds (100 seconds)
    double start_time = ros::Time::now().toSec();

    while (ros::ok()) {
        move_base_msgs::MoveBaseGoal goal = generateRandomGoal();
        ROS_INFO("Sending goal: (%f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal);
        ac.waitForResult();
        ROS_INFO("Goal reached!");

        // Check if the duration has passed
        if (ros::Time::now().toSec() - start_time >= duration) {
            break;
        }
    }

    // Stop the node gracefully
    ROS_INFO("Script completed");
    ros::shutdown();

    return 0;
}
