#!/usr/bin/env python

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point

class CubeSpawner():

    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('model_description') + "/urdf/"
        self.red_cube = self.path + "red_cube.urdf"
        self.spawned_cubes = []
        self.min_spacing = 1.0
        self.cube_positions = []

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def random_position(self):
        while True:
            x = random.uniform(-5, 5)
            y = random.uniform(-5, 5)  
            if all(self.min_distance(x, y, cube) for cube in self.spawned_cubes) and not (-0.2 <= x <= 0.2 and -0.2 <= y <= 0.2):
                return x, y


    def min_distance(self, x, y, cube):
        for c in self.spawned_cubes:
            pose = self.get_pose(c)
            dx = x - pose.position.x
            dy = y - pose.position.y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance < self.min_spacing:
                return False
        return True

    def get_pose(self, model_name):
        pose = Pose()
        try:
            pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)(model_name, "world").pose
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
        return pose

    def spawn_model(self):
        x, y = self.random_position()
        with open(self.red_cube, "r") as f:
            cube_urdf = f.read()
        quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=x, y=y, z=0.75), orient)
        model_name = "red_cube_" + str(len(self.spawned_cubes))
        self.sm(model_name, cube_urdf, "", pose, "world")
        self.spawned_cubes.append(model_name)
        self.cube_positions.append(pose)

    def delete_all_models(self):
        for model in self.spawned_cubes:
            self.dm(model)

    def shutdown_hook(self):
        self.delete_all_models()
        print("Shutting down")

if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("landmark_spawner")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    cs = CubeSpawner()
    rospy.on_shutdown(cs.shutdown_hook)
    rate = rospy.Rate(15)

    # Spawn multiple instances of the red cube
    for _ in range(10):
        cs.spawn_model()
        rate.sleep()

    rospy.spin()
