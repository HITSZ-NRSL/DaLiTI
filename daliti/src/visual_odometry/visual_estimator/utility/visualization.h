#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>
#include <rosbag/bag.h>


#include "line_geometry.h"

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern int IMAGE_ROW, IMAGE_COL;

extern std::string bag_file_name;
extern int if_write_res_to_bag;

void init_rosbag_for_recording();

void registerPub(ros::NodeHandle &n);

tf::Transform transformConversion(const tf::StampedTransform& t);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
    const Eigen::Vector3d &V, const std_msgs::Header &header, const int &failureId, 
    const Eigen::Vector3d &t_ic, const Eigen::Quaterniond &q_ic, bool recv_lio_fail_flag);

void printStatistics(const Estimator &estimator, double t);

void pub_VIO_Odometry(const Estimator &estimator, const std_msgs::Header &header);

void pub_LIO_Odometry(const StatesGroup & state, const std_msgs::Header &header, int fail_flag);

void pub_fused_Odometry(const Estimator &estimator, const StatesGroup & state, const std_msgs::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubLinesCloud(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
				   Eigen::Matrix3d loop_correct_r);

void pubPoseGraph(CameraPoseVisualization* posegraph, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);