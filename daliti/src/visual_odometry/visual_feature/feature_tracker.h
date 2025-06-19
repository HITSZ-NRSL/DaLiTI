#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <fstream>


#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud.h>
#include "std_msgs/Float32MultiArray.h"

#include "camera_models/CameraFactory.h"
#include "camera_models/CataCamera.h"
#include "camera_models/PinholeCamera.h"

#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "parameters.h"
#include "tic_toc.h"

#include "daliti/images_info.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();

    void readImage(const cv::Mat &_img, double _cur_time, Matrix3d relative_R, const sensor_msgs::PointCloudConstPtr &edgedes_msg, int init_flag,int edge_num);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;
    int cur_num, forw_num;

    static int n_id;
};

class DepthRegister
{
public:
    ros::NodeHandle n;
    // publisher for visualization
    ros::Publisher pub_depth_feature;
    ros::Publisher pub_depth_image;
    ros::Publisher pub_origin_pcdepth_proj_image;
    ros::Publisher pub_depth_image_depth;
    ros::Publisher pub_depth_cloud;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    const int num_bins = 1024;
    vector<vector<PointType>> pointsArray;

    DepthRegister(ros::NodeHandle n_in) : n(n_in)
    {
        // messages for RVIZ visualization
        pub_depth_feature = n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_feature", 5);
        pub_depth_image = n.advertise<sensor_msgs::Image>(PROJECT_NAME + "/vins/depth/depth_image", 5);
        pub_depth_image_depth = n.advertise<sensor_msgs::Image>(PROJECT_NAME + "/vins/depth/depth_image_depth", 5);
        pub_origin_pcdepth_proj_image = n.advertise<sensor_msgs::Image>("/proj_image", 5);
        pub_depth_cloud = n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_cloud", 5);

        pointsArray.resize(num_bins);
        for (int i = 0; i < num_bins; ++i)
            pointsArray[i].resize(num_bins);
    }

    sensor_msgs::ChannelFloat32 get_depth(const ros::Time &stamp_cur, const cv::Mat &edge_img,const cv::Mat &origion_img,
                                          const sensor_msgs::PointCloud2ConstPtr &origion_deskewed_cloud,
                                          const pcl::PointCloud<PointType>::Ptr &depthCloud,
                                          const camodocal::CameraPtr &camera_model,
                                          const sensor_msgs::PointCloudPtr &feature_points)
    {
        vector<geometry_msgs::Point32> features_2d=feature_points->points;
        // 0.1 initialize depth for return
        sensor_msgs::ChannelFloat32 depth_of_point;
        depth_of_point.name = "depth";
        depth_of_point.values.resize(features_2d.size(), -1);

        // 0.2  check if depthCloud available
        if (depthCloud->size() == 0)
            return depth_of_point;

        // 0.3 look up transform at current image time
        try
        {
            
            listener.waitForTransform("camera_init", "camera_link", stamp_cur, ros::Duration(0.01));
            listener.lookupTransform("camera_init", "camera_link", stamp_cur, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("image no tf!!");
            return depth_of_point;
        }

        double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
        xCur = transform.getOrigin().x();
        yCur = transform.getOrigin().y();
        zCur = transform.getOrigin().z();
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(rollCur, pitchCur, yawCur);
        Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

        // 0.4 transform cloud from global frame to camera frame
        pcl::PointCloud<PointType>::Ptr depth_cloud_local(new pcl::PointCloud<PointType>());
        
        pcl::transformPointCloud(*depthCloud, *depth_cloud_local, transNow.inverse());


        try
        {
            listener.waitForTransform("lidar_link", "camera_link", stamp_cur, ros::Duration(0.01));
            listener.lookupTransform("lidar_link", "camera_link", stamp_cur, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("image no tf!!");
            return depth_of_point;
        }

        xCur = transform.getOrigin().x();
        yCur = transform.getOrigin().y();
        zCur = transform.getOrigin().z();
        tf::Matrix3x3 mm(transform.getRotation());
        mm.getRPY(rollCur, pitchCur, yawCur);
        transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
        pcl::PointCloud<PointType>::Ptr pclpc_origion_deskewed_pointcloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr drigion_deskewed_cloud_local(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*origion_deskewed_cloud,*pclpc_origion_deskewed_pointcloud);
        pcl::transformPointCloud(*pclpc_origion_deskewed_pointcloud, *drigion_deskewed_cloud_local, transNow.inverse());

        // 0.5 project undistorted normalized (z) 2d features onto a unit sphere
        pcl::PointCloud<PointType>::Ptr features_3d_sphere(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)features_2d.size(); ++i)
        {
            // normalize 2d feature to a unit sphere
            Eigen::Vector3f feature_cur(features_2d[i].x, features_2d[i].y, features_2d[i].z); // z always equal to 1
            feature_cur.normalize();
            // convert to ROS standard
            PointType p;
            p.x = feature_cur(2);
            p.y = -feature_cur(0);
            p.z = -feature_cur(1);
            p.intensity = -1; // intensity will be used to save depth
            features_3d_sphere->push_back(p);
        }

        // 3. project depth cloud on a range image, filter points satcked in the same region
        float bin_res = 180.0 / (float)num_bins; // currently only cover the space in front of lidar (-90 ~ 90)
        cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));

        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];
            // filter points not in camera view
            if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
                continue;
            // find row id in range image
            float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0; // degrees, bottom -> up, 0 -> 360
            int row_id = round(row_angle / bin_res);
            // find column id in range image
            float col_angle = atan2(p.x, p.y) * 180.0 / M_PI; // degrees, left -> right, 0 -> 360
            int col_id = round(col_angle / bin_res);
            // id may be out of boundary
            if (row_id < 0 || row_id >= num_bins || col_id < 0 || col_id >= num_bins)
                continue;
            // only keep points that's closer
            float dist = pointDistance(p);
            if (dist < rangeImage.at<float>(row_id, col_id))
            {
                rangeImage.at<float>(row_id, col_id) = dist;
                pointsArray[row_id][col_id] = p;
            }
        }

        // 4. extract downsampled depth cloud from range image
        pcl::PointCloud<PointType>::Ptr depth_cloud_local_filter2(new pcl::PointCloud<PointType>());
        for (int i = 0; i < num_bins; ++i)
        {
            for (int j = 0; j < num_bins; ++j)
            {
                if (rangeImage.at<float>(i, j) != FLT_MAX)
                    depth_cloud_local_filter2->push_back(pointsArray[i][j]);
            }
        }
        *depth_cloud_local = *depth_cloud_local_filter2;

#if IF_OFFICIAL
        publishCloud(&pub_depth_cloud, depth_cloud_local, stamp_cur, "vins_body_ros");
#else
        
        publishCloud(&pub_depth_cloud, depth_cloud_local, stamp_cur, "camera_link");
#endif
        // 5 poject depth to feature points in depth image
        
        
        std::vector<std::pair<cv::Point2d, float>> points_2d_with_depth;
        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            // convert points from 3D to 2D
            PointType p = depth_cloud_local->points[i];
            Eigen::Vector3d p_3d(-p.y,-p.z,p.x);
            Eigen::Vector2d p_2d;
            camera_model->spaceToPlane(p_3d, p_2d);
            auto point_2d_with_depth = std::make_pair(cv::Point2d(p_2d(0), p_2d(1)), pointDistance(p));
            points_2d_with_depth.push_back(point_2d_with_depth);
        }
        std::vector<std::pair<cv::Point2d, float>> points_2d_with_depth_origion_pc;
        for (int i = 0; i < (int)drigion_deskewed_cloud_local->size(); ++i)
        {
            // convert points from 3D to 2D
            PointType p = drigion_deskewed_cloud_local->points[i];
            Eigen::Vector3d p_3d(-p.y,-p.z,p.x);
            Eigen::Vector2d p_2d;
            camera_model->spaceToPlane(p_3d, p_2d);
            auto point_2d_with_depth = std::make_pair(cv::Point2d(p_2d(0), p_2d(1)), pointDistance(p));
            if(p.x>0)points_2d_with_depth_origion_pc.push_back(point_2d_with_depth);
        }

        
        // 6 K-D tree search
        std::vector<std::pair<cv::Point2d, float>> features_2d_image_with_depth;
        for (int i = 0; i < (int)features_2d.size(); ++i)
        {
            features_2d_image_with_depth.push_back(std::make_pair(cv::Point2d(feature_points->channels[1].values[i],feature_points->channels[2].values[i]),-1));
        }

        std::vector<std::vector<cv::Point2d>> nearestPointsList = findNearestPoints_and_add_depth(features_2d_image_with_depth, points_2d_with_depth, 3);


        // // ! print for debug
        
        // for (size_t i = 0; i < features_2d_image_with_depth.size(); ++i)
        // {
        //     std::cout << "features_2d_image_with_depth[" << i << "] - Nearest Points in points_2d_with_depth: ";
        //     for (const cv::Point2d& point : nearestPointsList[i])
        //     {
        //         std::cout << "(" << point.x << ", " << point.y << ") ";
        //     }
        //     std::cout << std::endl;
        // }
        // // end debug

        // 7 update depth value for return
        for (int i = 0; i < (int)features_2d_image_with_depth.size(); ++i)
        {
            float s = features_2d_image_with_depth[i].second;
            features_3d_sphere->points[i].x *= s;
            features_3d_sphere->points[i].y *= s;
            features_3d_sphere->points[i].z *= s;
            // the obtained depth here is for unit sphere, VINS estimator needs depth for normalized feature (by value z), (lidar x = camera z)
            features_3d_sphere->points[i].intensity = features_3d_sphere->points[i].x;
            if (features_3d_sphere->points[i].intensity > 1.0)
                depth_of_point.values[i] = features_3d_sphere->points[i].intensity;
        }
        // visualize features in cartesian 3d space (including the feature without depth (default 1))
        
        publishCloud(&pub_depth_feature, features_3d_sphere, stamp_cur, "camera_link");


        // 8 pub depth image (slow!)
        if (pub_depth_image.getNumSubscribers() != 0)
        {
            cv::Mat showImage, circleImage;
            cv::cvtColor(edge_img, showImage, cv::COLOR_GRAY2RGB);
            circleImage = showImage.clone();

            // if debug depth visulization
            // for (int i = 0; i < (int)points_2d_with_depth.size(); ++i)
            // {
            //     float r, g, b;
            //     getColor(points_2d_with_depth[i].second, 50.0, r, g, b);
            //     cv::circle(circleImage, points_2d_with_depth[i].first, 1, cv::Scalar(r, g, b), 1);
            // }

            for (int i = 0; i < (int)features_2d_image_with_depth.size(); ++i)
            {
                if (features_2d_image_with_depth[i].second != -1)
                {
                    float r, g, b;
                    getColor(features_2d_image_with_depth[i].second, 50.0, r, g, b);
                    cv::circle(circleImage, features_2d_image_with_depth[i].first, 4, cv::Scalar(r, g, b), 4);
                    // cv::circle(circleImage, features_2d_image_with_depth[i].first, 2, cv::Scalar(0, 255, 0), 2);
                    for (const cv::Point2d &point : nearestPointsList[i])
                    {
                        cv::line(circleImage, features_2d_image_with_depth[i].first, cv::Point2d(point.x, point.y), cv::Scalar(255, 0, 0), 1, 8);
                    }
                }
                else{
                    cv::circle(circleImage, features_2d_image_with_depth[i].first, 4, cv::Scalar(255, 0, 0), 4);
                }
            }
            cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

            cv_bridge::CvImage bridge;
            bridge.image = showImage;
            bridge.encoding = "rgb8";
            sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
            imageShowPointer->header.stamp = stamp_cur;
            pub_depth_image.publish(imageShowPointer);
        }

        if (pub_depth_image_depth.getNumSubscribers() != 0)
        {
            cv::Mat showImage, circleImage;
            cv::cvtColor(edge_img, showImage, cv::COLOR_GRAY2RGB);
            circleImage = showImage.clone();

            for (int i = 0; i < (int)points_2d_with_depth.size(); ++i)
            {
                float r, g, b;
                getColor(points_2d_with_depth[i].second, 50.0, r, g, b);
                cv::circle(circleImage, points_2d_with_depth[i].first, 1, cv::Scalar(r, g, b), 1);
            }

            cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

            cv_bridge::CvImage bridge;
            bridge.image = showImage;
            bridge.encoding = "rgb8";
            sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
            imageShowPointer->header.stamp = stamp_cur;
            pub_depth_image_depth.publish(imageShowPointer);
        }


        // pub proj image
        // static double last_stamp_cur_us=0;
        static double last_lidar_stamp=0;
        // ROS_WARN(" %lf    %lf",last_stamp_cur_us,stamp_cur.toSec());
        // ROS_WARN(" %lf ",origion_deskewed_cloud.header.stamp.toSec());
        // if (pub_origin_pcdepth_proj_image.getNumSubscribers() != 0 
        //     && last_stamp_cur_us <= origion_deskewed_cloud.header.stamp.toSec() &&  origion_deskewed_cloud.header.stamp.toSec() <= stamp_cur.toSec())
        if (pub_origin_pcdepth_proj_image.getNumSubscribers() != 0 &&
            last_lidar_stamp != origion_deskewed_cloud->header.stamp.toSec())
        {
            cv::Mat showImage, circleImage;
            cv::cvtColor(origion_img, showImage, cv::COLOR_GRAY2RGB);
            circleImage = showImage.clone();

            for (int i = 0; i < (int)points_2d_with_depth_origion_pc.size(); ++i)
            {
                float r, g, b;
                getColor(points_2d_with_depth_origion_pc[i].second, 50.0, r, g, b);
                cv::circle(circleImage, points_2d_with_depth_origion_pc[i].first, 1, cv::Scalar(r, g, b), 1);
            }

            cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

            cv_bridge::CvImage bridge;
            bridge.image = showImage;
            bridge.encoding = "rgb8";
            sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
            imageShowPointer->header.stamp = stamp_cur;
            pub_origin_pcdepth_proj_image.publish(imageShowPointer);
        }
        // last_stamp_cur_us = stamp_cur.toSec();
        last_lidar_stamp =origion_deskewed_cloud->header.stamp.toSec();
        // 

        return depth_of_point;
    }

    void getColor(float p, float np, float &r, float &g, float &b)
    {
        float inc = 6.0 / np;
        float x = p * inc;
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
        if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
            r = 1.0f;
        else if (4 <= x && x <= 5)
            r = x - 4;
        else if (1 <= x && x <= 2)
            r = 1.0f - (x - 1);

        if (1 <= x && x <= 3)
            g = 1.0f;
        else if (0 <= x && x <= 1)
            g = x - 0;
        else if (3 <= x && x <= 4)
            g = 1.0f - (x - 3);

        if (3 <= x && x <= 5)
            b = 1.0f;
        else if (2 <= x && x <= 3)
            b = x - 2;
        else if (5 <= x && x <= 6)
            b = 1.0f - (x - 5);
        r *= 255.0;
        g *= 255.0;
        b *= 255.0;
    }

    std::vector<std::vector<cv::Point2d>> findNearestPoints_and_add_depth(std::vector<std::pair<cv::Point2d, float>>& A, 
                                                                          const std::vector<std::pair<cv::Point2d, float>>& B, 
                                                                          int k)
    {
        cv::Mat BMat(B.size(), 2, CV_32F);
        for (size_t i = 0; i < B.size(); ++i)
        {
            BMat.at<float>(i, 0) = B[i].first.x;
            BMat.at<float>(i, 1) = B[i].first.y;
        }

        cv::flann::KDTreeIndexParams indexParams;
        cv::flann::Index kdTree(BMat, indexParams);

        std::vector<std::vector<cv::Point2d>> nearestPointsList;
        // for (std::pair<cv::Point2d, float> &Apoint_with_depth : A)
        // for (auto Apoint_with_depth = A.begin(); Apoint_with_deptht != A.end(); ++Apoint_with_depth)
        for (size_t i = 0; i < A.size(); ++i)
        {
            cv::Mat query(1, 2, CV_32F);
            query.at<float>(0, 0) = A[i].first.x;
            query.at<float>(0, 1) = A[i].first.y;

            cv::Mat indices, distances;
            kdTree.knnSearch(query, indices, distances, k, cv::flann::SearchParams());

            std::vector<cv::Point2d> nearestPoints;
            std::vector<float> depthes;
            std::vector<std::pair<float,float>> depthes_with_distance;
            bool depth_calc_flag=1;
            for (int i = 0; i < k; ++i)
            {
                int index = indices.at<int>(0, i);

                // std::cout<<"distances "<<distances.at<float>(0, i)<<std::endl;
                if (distances.at<float>(0, i) > 50.0)
                {
                    depth_calc_flag = 0;
                    // continue;
                }
                else
                {
                    depthes.push_back(B[index].second);
                    depthes_with_distance.push_back(std::make_pair(B[index].second,distances.at<float>(0, i)));
                }
                nearestPoints.push_back(B[index].first);
            }
            if(depth_calc_flag)
            {
                A[i].second=std::accumulate(depthes.begin(), depthes.end(), 0.0f)/k;
                // std::pair<float, float> result = kMeansClustering(depthes);
                // A[i].second=result.first;
                // float result = kMeansClustering_retnearest(depthes_with_distance);
                // A[i].second=result;
            }

            nearestPointsList.push_back(nearestPoints);
        }
        return nearestPointsList;
    }

    // K-means clustering function
    std::pair<float, float> kMeansClustering(const std::vector<float>& depthes) {
        // Initialize centroids with two random values from the vector
        float centroid1 = depthes[rand() % depthes.size()];
        float centroid2 = depthes[rand() % depthes.size()];

        // Variables to keep track of cluster membership and sum of values for each cluster
        std::vector<float> cluster1, cluster2;
        float sum1 = 0.0, sum2 = 0.0;

        bool converged = false;
        while (!converged) {
            cluster1.clear();
            cluster2.clear();
            sum1 = 0.0;
            sum2 = 0.0;

            // Assign each point to the nearest centroid
            for (const auto& depth : depthes) {
                float dist1 = std::abs(depth - centroid1);
                float dist2 = std::abs(depth - centroid2);

                if (dist1 < dist2) {
                    cluster1.push_back(depth);
                    sum1 += depth;
                } else {
                    cluster2.push_back(depth);
                    sum2 += depth;
                }
            }

            // Update centroids
            float newCentroid1 = cluster1.empty() ? centroid1 : sum1 / cluster1.size();
            float newCentroid2 = cluster2.empty() ? centroid2 : sum2 / cluster2.size();

            // Check for convergence
            if (newCentroid1 == centroid1 && newCentroid2 == centroid2) {
                converged = true;
            } else {
                centroid1 = newCentroid1;
                centroid2 = newCentroid2;
            }
        }

        // Calculate and return the average of the cluster with smaller mean
        float avg1 = cluster1.empty() ? 0.0 : sum1 / cluster1.size();
        float avg2 = cluster2.empty() ? 0.0 : sum2 / cluster2.size();

        if (avg1 < avg2) {
            return std::make_pair(avg1, avg2);
        } else {
            return std::make_pair(avg2, avg1);
        }
    }

    float kMeansClustering_retnearest(std::vector<std::pair<float,float>> depthes_with_distance) {
        // Initialize centroids with two random values from the vector
        float centroid1 = depthes_with_distance[rand() % depthes_with_distance.size()].first;
        float centroid2 = depthes_with_distance[rand() % depthes_with_distance.size()].first;

        // Variables to keep track of cluster membership and sum of values for each cluster
        std::vector<std::pair<float,float>> cluster1, cluster2;
        float sum1 = 0.0, sum2 = 0.0;

        bool converged = false;
        while (!converged) {
            cluster1.clear();
            cluster2.clear();
            sum1 = 0.0;
            sum2 = 0.0;

            // Assign each point to the nearest centroid
            for (const auto& depth : depthes_with_distance) {
                float dist1 = std::abs(depth.first - centroid1);
                float dist2 = std::abs(depth.first - centroid2);

                if (dist1 < dist2) {
                    cluster1.push_back(depth);
                    sum1 += depth.first;
                } else {
                    cluster2.push_back(depth);
                    sum2 += depth.first;
                }
            }

            // Update centroids
            float newCentroid1 = cluster1.empty() ? centroid1 : sum1 / cluster1.size();
            float newCentroid2 = cluster2.empty() ? centroid2 : sum2 / cluster2.size();

            // Check for convergence
            if (newCentroid1 == centroid1 && newCentroid2 == centroid2) {
                converged = true;
            } else {
                centroid1 = newCentroid1;
                centroid2 = newCentroid2;
            }
        }

        // Calculate and return the average of the cluster with smaller mean
        float avg1 = cluster1.empty() ? 0.0 : sum1 / cluster1.size();
        float avg2 = cluster2.empty() ? 0.0 : sum2 / cluster2.size();

        float ret=0,min=100000;
        if (avg1 < avg2) {
            for(int i=0;i<cluster1.size();++i)
            {
                if(cluster1[i].second<min)
                {
                    min=cluster1[i].second;
                    ret=cluster1[i].first;
                }
            }
            return ret;
        } else {
            for(int i=0;i<cluster2.size();++i)
            {
                if(cluster1[i].second<min)
                {
                    min=cluster2[i].second;
                    ret=cluster2[i].first;
                }
            }
            return ret;
        }
    }
};