#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <random>
#include <fstream>

class CubeSpawner
{
public:
    CubeSpawner(ros::NodeHandle &nh) : nh_(nh)
    {
        rospack_ = ros::package::getPath("model_description");
        path_ = rospack_ + "/urdf/";
        red_cube_ = path_ + "red_cube.urdf";
        min_spacing_ = 1.0;
    }

    void randomPosition(double &x, double &y)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> y_dist(-5.0, 5.0);

        while (true)
        {
            x = x_dist(gen);
            y = y_dist(gen);
            if (minDistance(x, y))
            {
                if (!(x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0))
                {
                    return;
                }
            }
        }
    }

    bool minDistance(double x, double y)
    {
        for (const std::string &cube : spawned_cubes_)
        {
            geometry_msgs::Pose pose = getPose(cube);
            double dx = x - pose.position.x;
            double dy = y - pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < min_spacing_)
            {
                return false;
            }
        }
        return true;
    }

    geometry_msgs::Pose getPose(const std::string &model_name)
    {
        geometry_msgs::Pose pose;
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = model_name;
        srv.request.relative_entity_name = "world";

        if (get_model_state_client_.call(srv))
        {
            pose = srv.response.pose;
        }
        else
        {
            ROS_WARN("Service call failed");
        }
        return pose;
    }

    void spawnModel()
    {
        double x, y;
        randomPosition(x, y);
        std::string model_name = "red_cube_" + std::to_string(spawned_cubes_.size());
        std::ifstream file(red_cube_);
        std::string cube_urdf((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        tf::Quaternion quat;
        quat.setEuler(0, 0, 1.57);
        geometry_msgs::Quaternion orient;
        orient.x = quat.getX();
        orient.y = quat.getY();
        orient.z = quat.getZ();
        orient.w = quat.getW();

        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.75;
        pose.orientation = orient;

        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = model_name;
        srv.request.model_xml = cube_urdf;
        srv.request.initial_pose = pose;
        srv.request.reference_frame = "world";

        if (spawn_model_client_.call(srv))
        {
            spawned_cubes_.push_back(model_name);
            cube_positions_.push_back(pose);
        }
    }

    void deleteAllModels()
    {
        for (const std::string &model : spawned_cubes_)
        {
            gazebo_msgs::DeleteModel srv;
            srv.request.model_name = model;
            delete_model_client_.call(srv);
        }
    }

    void shutdownHook()
    {
        deleteAllModels();
        ROS_INFO("Shutting down");
    }

    void run()
    {
        ROS_INFO("Waiting for gazebo services...");
        ros::service::waitForService("/gazebo/delete_model");
        ros::service::waitForService("/gazebo/spawn_urdf_model");

        ros::Rate rate(15);

        for (int i = 0; i < 10; ++i)
        {
            spawnModel();
            rate.sleep();
        }

        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    std::string rospack_;
    std::string path_;
    std::string red_cube_;
    double min_spacing_;
    std::vector<std::string> spawned_cubes_;
    std::vector<geometry_msgs::Pose> cube_positions_;
    ros::ServiceClient spawn_model_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient delete_model_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    ros::ServiceClient get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmark_spawner");
    ros::NodeHandle nh;
    CubeSpawner cs(nh);
    // ros::shutdown();

    cs.run();
    return 0;
}
