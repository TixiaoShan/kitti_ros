#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>

using namespace std;


class PoseReader
{
private:

    ros::NodeHandle nh;

    ros::Publisher pubPath;
    ros::Publisher pubOdom;

    vector<vector<double>> kitti_poses;
    vector<vector<double>> xyzrpy_poses;

public:
    PoseReader()
    {
        pubPath = nh.advertise<nav_msgs::Path>     ("kitti_ros/path", 1);
        pubOdom = nh.advertise<nav_msgs::Odometry> ("kitti_ros/odometry", 1);

        std::string sequence;
        nh.getParam("kitti_sequence", sequence);

        // read txt pose file
        std::string pkg_path = ros::package::getPath("kitti_ros");
        std::string file_path = pkg_path + "/dataset/poses/" + sequence + ".txt";
        ROS_INFO("Pose file: %s", file_path.c_str());

        ifstream inFile;
        inFile.open(file_path.c_str());
        if (!inFile)
        {
            ROS_ERROR("File could not be opened.");
            exit(1);
        }
        else
        {
            while (!inFile.eof())
            {
                vector<double> cur_pose(12);
                for (int i = 0; i < 12; i++)
                {
                    inFile >> cur_pose[i];
                }
                kitti_poses.push_back(cur_pose);
            }
            inFile.close();
        }
        if (kitti_poses.size() > 1) // last pose is 0,0,0
            kitti_poses.resize(kitti_poses.size()-1);


        Eigen::Matrix4f lidar_to_camera;
        lidar_to_camera(0,0) = 7.533745e-03; lidar_to_camera(0,1) = -9.999714e-01; lidar_to_camera(0,2) = -6.166020e-04; lidar_to_camera(0,3) = -4.069766e-03;
        lidar_to_camera(1,0) = 1.480249e-02; lidar_to_camera(1,1) = 7.280733e-04; lidar_to_camera(1,2) = -9.998902e-01; lidar_to_camera(1,3) = -7.631618e-02;
        lidar_to_camera(2,0) = 9.998621e-01; lidar_to_camera(2,1) = 7.523790e-03; lidar_to_camera(2,2) = 1.480755e-02; lidar_to_camera(2,3) = -2.717806e-01;
        lidar_to_camera(3,0) = 0; lidar_to_camera(3,1) = 0; lidar_to_camera(3,2) = 0; lidar_to_camera(3,3) = 1;
        Eigen::Affine3f affine_lidar_to_camera;
        affine_lidar_to_camera = lidar_to_camera;

        // convert to ROS format
        for (int i = 0; i < (int)kitti_poses.size(); ++i)
        {
            vector<double> cur_pose = kitti_poses[i];
            Eigen::Matrix4f M;
            M(0,0) = cur_pose[0]; M(0,1) = cur_pose[1]; M(0,2) = cur_pose[2]; M(0,3) = cur_pose[3];
            M(1,0) = cur_pose[4]; M(1,1) = cur_pose[5]; M(1,2) = cur_pose[6]; M(1,3) = cur_pose[7];
            M(2,0) = cur_pose[8]; M(2,1) = cur_pose[9]; M(2,2) = cur_pose[10]; M(2,3) = cur_pose[11];
            M(3,0) = 0; M(3,1) = 0; M(3,2) = 0; M(3,3) = 1;

            Eigen::Affine3f m;
            m = M;
            m = affine_lidar_to_camera.inverse() * m * affine_lidar_to_camera;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (m, x, y, z, roll, pitch, yaw);

            vector<double> pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(z);
            pose.push_back(roll);
            pose.push_back(pitch);
            pose.push_back(yaw);
            xyzrpy_poses.push_back(pose);
        }
    }

    ~PoseReader(){}

    void visualization()
    {
        usleep(1e6);
        ROS_INFO("Visualization begins in 1 seconds.");

        for (int i = 0; i < (int)xyzrpy_poses.size(); ++i)
        {
            float x, y, z, roll, pitch, yaw;
            x = xyzrpy_poses[i][0];
            y = xyzrpy_poses[i][1];
            z = xyzrpy_poses[i][2];
            roll = xyzrpy_poses[i][3];
            pitch = xyzrpy_poses[i][4];
            yaw = xyzrpy_poses[i][5];

            // tf
            static tf::TransformBroadcaster tfMap2Base;
            tf::Transform map_to_base_link = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
            tfMap2Base.sendTransform(tf::StampedTransform(map_to_base_link, ros::Time::now(), "map", "base_link"));

            // odometry
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "map";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = z;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            pubOdom.publish(odom);

            // path
            static nav_msgs::Path pathMsg;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose = odom.pose.pose;
            pathMsg.poses.push_back(pose_stamped);
            pathMsg.header.stamp = ros::Time::now();
            pathMsg.header.frame_id = "map";
            pubPath.publish(pathMsg);

            usleep(1e4);

            if (!ros::ok())
                break;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_ros");

    PoseReader PR;
    
    ROS_INFO("\033[1;32m----> KITTI Pose Reader.\033[0m");

    PR.visualization();

    ros::spin();
    
    return 0;
}