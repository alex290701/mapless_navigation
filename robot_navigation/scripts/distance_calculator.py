#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from math import sin,cos

class ObstacleDistanceCalculator():
    def __init__(self):
        rospy.init_node("obstacle_distance_calculator")
        self.obstacle_distance =Point()
        self.obstacle_radius = 0.3  # Define the obstacle radius in meters
        self.min_distance = float("inf")  # Initialize to positive infinity

        # Subscribe to LiDAR data
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publish the obstacle distance
        self.obstacle_distance_pub = rospy.Publisher("/landmark_distance", Point, queue_size=10)

    def lidar_callback(self, data):
        # Find the minimum distance in the LiDAR scan within the obstacle radius
        for i, distance in enumerate(data.ranges):
            if data.range_min <= distance <= data.range_max and distance < self.min_distance:
                self.min_distance = distance

        # Calculate x and y distance from the robot
        angle = data.angle_min + (i * data.angle_increment)
        self.obstacle_distance.x = self.min_distance * cos(angle)
        self.obstacle_distance.y = self.min_distance * sin(angle)
        # Publish the obstacle distance
        self.obstacle_distance_pub.publish(self.obstacle_distance)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        obstacle_calculator = ObstacleDistanceCalculator()
        obstacle_calculator.run()
    except rospy.ROSInterruptException:
        pass
