#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <thread>
#include <mutex>
#include <chrono>

#include "cloud_info.h"
#include "my_utility.h"

using namespace std;

enum class SensorType
{
  VELODYNE,
  OUSTER,
  ROBOSENSE,
  LIVOX
};

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const &left, smoothness_t const &right)
  {
    return left.value < right.value;
  }
};

using PointXYZIRT = VelodynePointXYZIRT;
std::ofstream lidar_feature_extract_time_file("~/daliti/src/DaLiTI/eskf_lio/Log/lidar_feature_extract_time.txt", std::ios::out | std::ios::app);

class FeatureExtract
{
public:
  ros::NodeHandle nh;
  // Topics
  string pointCloudTopic;
  // Frames
  string lidarFrame;

  // Lidar Sensor Configuration
  SensorType sensor;
  int N_SCAN;
  int Horizon_SCAN;
  int downsampleRate;
  float lidarMinRange;
  float lidarMaxRange;
  int point_filter_num;
  int feature_enabled;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int edgeFeatureMinValidNum;
  int surfFeatureMinValidNum;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<rsPointXYZIRT>::Ptr tmpRSCloudIn;
  pcl::PointCloud<PointType>::Ptr inputCloud;
  pcl::PointCloud<PointType>::Ptr sampleCloud;
  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr extractedCloud;

  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  GC_LOAM::cloud_info cloudInfo;
  double timeScanCur;
  double timeScanEnd;
  cv::Mat rangeMat;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  // voxel filter paprams
  float odometrySurfLeafSize;

  // CPU Params
  int numberOfCores;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;
  ros::Publisher pubFullPoints;

  std_msgs::Header cloudHeader;

  std::vector<int> columnIdnCountVec;

  FeatureExtract()
  {
    nh.param<std::string>("common/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("feature_extract/lidarFrame", lidarFrame, "base_link");

    std::string sensorStr;
    nh.param<std::string>("feature_extract/sensor", sensorStr, "");
    if (sensorStr == "velodyne")
    {
      sensor = SensorType::VELODYNE;
    }
    else if (sensorStr == "livox")
    {
      sensor = SensorType::LIVOX;
      std::cout << "sensor type: livox-" << int(sensor) << std::endl;
    }
    else if (sensorStr == "ouster")
    {
      sensor = SensorType::OUSTER;
    }
    else if (sensorStr == "robosense")
    {
      sensor = SensorType::ROBOSENSE;
    }
    else
    {
      ROS_ERROR_STREAM(
          "Invalid sensor type (must be either 'velodyne' 'ouster' 'robosense' or 'livox'): " << sensorStr);
      ros::shutdown();
    }

    nh.param<int>("feature_extract/N_SCAN", N_SCAN, 16);
    nh.param<int>("feature_extract/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("feature_extract/downsampleRate", downsampleRate, 1);
    nh.param<int>("feature_extract/pointFilterRate", point_filter_num, 1);
    nh.param<float>("feature_extract/lidarMinRange", lidarMinRange, 1.0);
    nh.param<int>("feature_extract/feature_enabled", feature_enabled, 1);
    nh.param<float>("feature_extract/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<float>("feature_extract/edgeThreshold", edgeThreshold, 0.1);
    nh.param<float>("feature_extract/surfThreshold", surfThreshold, 0.1);
    nh.param<int>("feature_extract/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    nh.param<int>("feature_extract/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    nh.param<float>("feature_extract/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);

    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 50, &FeatureExtract::cloudHandler, this);

    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1);
    pubFullPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_filtered", 10);
    pubLaserCloudInfo = nh.advertise<GC_LOAM::cloud_info>("/feature/cloud_info", 1);

    allocateMemory();
    resetParameters();
  }
  void allocateMemory()
  {
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
    tmpRSCloudIn.reset(new pcl::PointCloud<rsPointXYZIRT>());
    inputCloud.reset(new pcl::PointCloud<PointType>());
    sampleCloud.reset(new pcl::PointCloud<PointType>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel = new int[N_SCAN * Horizon_SCAN];

    resetParameters();
  }

  void resetParameters()
  {
    laserCloudIn->clear();
    extractedCloud->clear();
    inputCloud->clear();
    sampleCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    columnIdnCountVec.assign(N_SCAN, 0);
  }




  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    ros::Time t1 = ros::Time::now();
    if (!cachePointCloud(laserCloudMsg))
      return;

    if (feature_enabled)
    {
      projectPointCloud();

      cloudExtraction();

      calculateSmoothness();

      markOccludedPoints();

      extractFeatures();

      publishFeatureCloud();
      resetParameters();
    }
    else
    {
      samplePointCloud();
      publishSampleCloud();
      resetParameters();
    }

    ros::Time t2 = ros::Time::now(); 
    double processing_time_ms = (t2 - t1).toSec() * 1000.0;

    if (lidar_feature_extract_time_file.is_open())
    {
      lidar_feature_extract_time_file << processing_time_ms << std::endl;
    }

  }
#define TEST_LIO_SAM_6AXIS_DATA
  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    sensor_msgs::PointCloud2 currentCloudMsg = *laserCloudMsg;
    double timespan;
    cloudHeader = currentCloudMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();

    if (sensor == SensorType::VELODYNE)
    {
      pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
      inputCloud->points.resize(laserCloudIn->size());
      inputCloud->is_dense = laserCloudIn->is_dense;
      
#ifndef TEST_LIO_SAM_6AXIS_DATA
      timespan = laserCloudIn->points.back().time * 1e-6;
#else
      timespan = laserCloudIn->points.back().time - laserCloudIn->points[0].time;
#endif
      for (size_t i = 0; i < laserCloudIn->size(); i++)
      {
        auto &src = laserCloudIn->points[i];
        auto &dst = inputCloud->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring; //  ring
        dst.normal_z = timespan;
#ifndef TEST_LIO_SAM_6AXIS_DATA
        dst.normal_x = src.time * 1e-6 / timespan;
#else
        dst.normal_x = (src.time + timespan) / timespan;
#endif
      }
#ifdef TEST_LIO_SAM_6AXIS_DATA
      timespan = 0.0;
#endif
      // std::cout << "stamp: " << laserCloudIn->points[0].time << "," << laserCloudIn->points[100].time
      //           << ", " << laserCloudIn->points.back().time << "," << inputCloud->points.back().normal_x
      //           << ", " << inputCloud->points.back().normal_z << std::endl;
    }
    else if (sensor == SensorType::LIVOX)
    {
      pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
      inputCloud->points.resize(laserCloudIn->size());
      inputCloud->is_dense = laserCloudIn->is_dense;
      timespan = laserCloudIn->points.back().time;
      for (size_t i = 0; i < laserCloudIn->size(); i++)
      {
        auto &src = laserCloudIn->points[i];
        auto &dst = inputCloud->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring; //  ring
        dst.normal_z = timespan;
        dst.normal_x = src.time / timespan;
      }
      std::cout << "stamp: " << laserCloudIn->points[0].time << ", " << laserCloudIn->points.back().time << std::endl;
    }
    else if (sensor == SensorType::OUSTER)
    {
      // Convert to Velodyne format
      pcl::fromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
      // pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
      inputCloud->points.resize(tmpOusterCloudIn->size());
      inputCloud->is_dense = tmpOusterCloudIn->is_dense;
      
      // timespan = tmpOusterCloudIn->points.back().t;
      timespan = tmpOusterCloudIn->points[tmpOusterCloudIn->size() - 2].t;
      for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
      {
        auto &src = tmpOusterCloudIn->points[i];
        auto &dst = inputCloud->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring;
        dst.normal_z = timespan * 1e-9f;
        dst.normal_x = src.t / timespan; //        *1e-9f;
      }
      timespan = timespan * 1e-9f;
      // std::cout << std::endl;
      // for (int i = tmpOusterCloudIn->size() - 6; i < tmpOusterCloudIn->size(); i++)
      //     std::cout << tmpOusterCloudIn->points[i].t * 1e-9f << " ,";
      // std::cout << std::endl;
    }
    else if (sensor == SensorType::ROBOSENSE)
    {
      
      pcl::fromROSMsg(currentCloudMsg, *tmpRSCloudIn);
      // inputCloud->points.resize(tmpRSCloudIn->size());
      // inputCloud->is_dense = tmpRSCloudIn->is_dense;

      timespan = tmpRSCloudIn->points[tmpRSCloudIn->size() - 1].timestamp - tmpRSCloudIn->points[0].timestamp;
      std::cout << "fist: " << tmpRSCloudIn->points[1].timestamp
                << ", 100: " << tmpRSCloudIn->points[100].timestamp
                << ",intervel: " << tmpRSCloudIn->points[tmpRSCloudIn->size() - 1].timestamp - tmpRSCloudIn->points[0].timestamp
                << ",t: " << tmpRSCloudIn->points[0].timestamp - currentCloudMsg.header.stamp.toSec()
                << "timespan: " << timespan << ",cloudsize: " << tmpRSCloudIn->size() << std::endl;
      for (size_t i = 0; i < tmpRSCloudIn->size(); i++)
      {
        auto &src = tmpRSCloudIn->points[i];
        if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
          continue;

        PointType dst;
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring;
        dst.normal_z = timespan;
        dst.normal_x = (src.timestamp - tmpRSCloudIn->points[0].timestamp) / timespan;
        inputCloud->push_back(dst);
      }
      // timespan = 0.0;
      cloudHeader.stamp = ros::Time().fromSec(cloudHeader.stamp.toSec() - timespan);
    }
    else
    {
      ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
      ros::shutdown();
    }

    // get timestamp
    timeScanEnd = timeScanCur + timespan; // inputCloud->points.back().normal_x;
    // std::cout << "timeC:" << timeScanCur << "," << timespan << std::endl;

    // check dense flag
    if (inputCloud->is_dense == false)
    {
      ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
      ros::shutdown();
    }

    // check ring channel
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
      ringFlag = -1;
      for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
      {
        if (currentCloudMsg.fields[i].name == "ring")
        {
          ringFlag = 1;
          break;
        }
      }
      if (ringFlag == -1)
      {
        ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        ros::shutdown();
      }
    }

    return true;
  }

  void samplePointCloud()
  {
    sampleCloud->clear();
    int cloudSize = inputCloud->points.size();
    sampleCloud->reserve(cloudSize);
    for (int i = 0; i < cloudSize; ++i)
    {
      if (i % point_filter_num != 0)
        continue;

      PointType thisPoint;
      thisPoint.x = inputCloud->points[i].x;
      thisPoint.y = inputCloud->points[i].y;
      thisPoint.z = inputCloud->points[i].z;
      thisPoint.intensity = inputCloud->points[i].intensity;
      thisPoint.normal_x = inputCloud->points[i].normal_x;
      thisPoint.normal_y = inputCloud->points[i].normal_y;
      thisPoint.normal_z = inputCloud->points[i].normal_z;

      float range = pointDistance(thisPoint);
      if (range < lidarMinRange || range > lidarMaxRange)
        continue;
      sampleCloud->push_back(thisPoint);
    }
    // std::cout << "-------------cloud: " << cloudSize << "  " << sampleCloud->size() << std::endl;
  }

  void projectPointCloud()
  {
    int cloudSize = inputCloud->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
      PointType thisPoint;
      thisPoint.x = inputCloud->points[i].x;
      thisPoint.y = inputCloud->points[i].y;
      thisPoint.z = inputCloud->points[i].z;
      thisPoint.intensity = inputCloud->points[i].intensity;
      thisPoint.normal_x = inputCloud->points[i].normal_x;
      thisPoint.normal_y = inputCloud->points[i].normal_y;
      thisPoint.normal_z = inputCloud->points[i].normal_z;

      float range = pointDistance(thisPoint);
      if (range < lidarMinRange || range > lidarMaxRange)
        continue;

      int rowIdn = inputCloud->points[i].normal_y;
      if (rowIdn < 0 || rowIdn >= N_SCAN)
        continue;

      if (rowIdn % downsampleRate != 0)
        continue;

      int columnIdn = -1;
      if (sensor == SensorType::LIVOX)
      {
        columnIdn = columnIdnCountVec[rowIdn];
        columnIdnCountVec[rowIdn] += 1;
      }
      else
      {
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        static float ang_res_x = 360.0 / float(Horizon_SCAN);
        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
        if (columnIdn >= Horizon_SCAN)
          columnIdn -= Horizon_SCAN;
      }

      if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;

      if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
        continue;
      // TODO: add deskew here

      rangeMat.at<float>(rowIdn, columnIdn) = range;

      int index = columnIdn + rowIdn * Horizon_SCAN;
      fullCloud->points[index] = thisPoint;
    }
  }

  void cloudExtraction()
  {
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i)
    {
      cloudInfo.startRingIndex[i] = count - 1 + 5;

      for (int j = 0; j < Horizon_SCAN; ++j)
      {
        if (rangeMat.at<float>(i, j) != FLT_MAX)
        {
          // mark the points' column index for marking occlusion later
          cloudInfo.pointColInd[count] = j;
          // save range info
          cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
          // save extracted cloud
          extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
          // size of extracted cloud
          ++count;
        }
      }
      cloudInfo.endRingIndex[i] = count - 1 - 5;
    }
  }

  void calculateSmoothness()
  {
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
      float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] + cloudInfo.pointRange[i + 5];

      cloudCurvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

      cloudNeighborPicked[i] = 0;
      cloudLabel[i] = 0;
      // cloudSmoothness for sorting
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i;
    }
  }

  void markOccludedPoints()
  {
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {
      // occluded points
      float depth1 = cloudInfo.pointRange[i];
      float depth2 = cloudInfo.pointRange[i + 1];
      int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

      if (columnDiff < 10)
      {
        // 10 pixel diff in range image
        if (depth1 - depth2 > 0.3)
        {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
      // parallel beam
      float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
      float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

      if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  void extractFeatures()
  {
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++)
    {

      surfaceCloudScan->clear();

      for (int j = 0; j < 6; j++)
      {

        int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
        int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep)
          continue;

        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--)
        {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
          {
            largestPickedNum++;
            if (largestPickedNum <= 20)
            {
              cloudLabel[ind] = 1;
              // extractedCloud->points[ind].normal_z = 1.0; //   for corner
              cornerCloud->push_back(extractedCloud->points[ind]);
            }
            else
            {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++)
        {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
          {

            cloudLabel[ind] = -1;
            cloudNeighborPicked[ind] = 1;

            for (int l = 1; l <= 5; l++)
            {

              int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {

              int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++)
        {
          if (cloudLabel[k] <= 0)
          {
            // extractedCloud->points[k].normal_z = 2.0; //   for surf
            surfaceCloudScan->push_back(extractedCloud->points[k]);
          }
        }
      }

      surfaceCloudScanDS->clear();
      
      // downSizeFilter.setInputCloud(surfaceCloudScan);
      // downSizeFilter.filter(*surfaceCloudScanDS);

      // *surfaceCloud += *surfaceCloudScanDS;
      *surfaceCloud += *surfaceCloudScan;
    }
  }

  void freeCloudInfoMemory()
  {
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
  }

  void publishFeatureCloud()
  {
    // free cloud info memory
    // std::cout << "surf: " << surfaceCloud->size() << std::endl;
    freeCloudInfoMemory();
    // std::cout << "edge: " << cornerCloud->size() << std::endl;
    // save newly extracted features
    cloudInfo.cloud_corner = publishCloud(&pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
    // cloudHeader.stamp = ros::Time().fromSec(timeScanEnd); // lio used
    // publishCloud(&pubFullPoints, extractedCloud, cloudHeader.stamp, lidarFrame);
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
  }

  void publishSampleCloud()
  {
    publishCloud(&pubSurfacePoints, sampleCloud, cloudHeader.stamp, lidarFrame);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eskf_lio");

  FeatureExtract FE;

  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  if (lidar_feature_extract_time_file.is_open())
  {
    lidar_feature_extract_time_file.close();
  }

  return 0;
}