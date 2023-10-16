#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <cmath>

class ObstacleDistanceCalculator {
public:
    ObstacleDistanceCalculator() : nh("~") {
        nh.param("obstacle_radius", obstacle_radius, 0.3);

        min_distance = std::numeric_limits<double>::infinity();

        lidar_sub = nh.subscribe("/scan", 1, &ObstacleDistanceCalculator::lidarCallback, this);
        obstacle_distance_pub = nh.advertise<geometry_msgs::Point>("/landmark_distance", 1);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        min_distance = std::numeric_limits<double>::infinity();
        size_t min_index = 0;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (scan->range_min <= scan->ranges[i] && scan->ranges[i] <= scan->range_max && scan->ranges[i] < min_distance) {
                min_distance = scan->ranges[i];
                min_index = i;
            }
        }

        if (std::isfinite(min_distance)) {
            double angle = scan->angle_min + min_index * scan->angle_increment;

            geometry_msgs::Point obstacle_distance;
            obstacle_distance.x = min_distance * cos(angle);
            obstacle_distance.y = min_distance * sin(angle);

            obstacle_distance_pub.publish(obstacle_distance);
        }
    }

    void run() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher obstacle_distance_pub;
    double obstacle_radius;
    double min_distance;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_distance_calculator");

    ObstacleDistanceCalculator calculator;
    calculator.run();

    return 0;
}
