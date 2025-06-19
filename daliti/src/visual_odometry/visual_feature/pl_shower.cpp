#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <sstream>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include "camera_models/CameraFactory.h"
#include "camera_models/CataCamera.h"
#include "camera_models/PinholeCamera.h"
#include "camera_models/EquidistantCamera.h"

#include "parameters.h"

camodocal::CameraPtr m_camera;
cv::Mat undist_map1_, undist_map2_, K_;

ros::Publisher pub_point_line;
sensor_msgs::PointCloudConstPtr linefeature;
void project(cv::Point2f &pt, cv::Mat const &K)
{
  pt.x = K.at<float>(0, 0) * pt.x + K.at<float>(0, 2);
  pt.y = K.at<float>(1, 1) * pt.y + K.at<float>(1, 2);
}

void readIntrinsicParameter(const string &calib_file)
{
    ROS_DEBUG("reading paramerter of camera %s", calib_file.c_str());

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    

}
void callback(const sensor_msgs::PointCloudConstPtr &point_feature_msg,
              const sensor_msgs::PointCloudConstPtr &line_feature_msg,
              const sensor_msgs::ImageConstPtr &img_msg)
{
  
  cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat show_img = ptr->image;
  cv::Mat img1;
  cv::cvtColor(show_img, img1, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < point_feature_msg->points.size(); i++)
  {
    cv::Point endPoint = cv::Point(point_feature_msg->channels[1].values[i], point_feature_msg->channels[2].values[i]);
    cv::circle(img1, endPoint, 2, cv::Scalar(0, 255, 0), 2);
  }

  cv::remap(img1, show_img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
  for (int i = 0; i < line_feature_msg->points.size(); i++)
  {
    cv::Point2f startPoint = cv::Point2f(line_feature_msg->points[i].x, line_feature_msg->points[i].y);
    project(startPoint, K_);
    cv::Point2f endPoint = cv::Point2f(line_feature_msg->channels[1].values[i], line_feature_msg->channels[2].values[i]);
    project(endPoint, K_);
    cv::line(show_img, startPoint, endPoint, cv::Scalar(0, 0, 255), 2, 8);
  }

  sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show_img).toImageMsg();
  pub_point_line.publish(output_msg);
  // cv::namedWindow("LSD matches", CV_WINDOW_NORMAL);
  // cv::imshow( "LSD matches", show_img );
  // cv::waitKey(5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vins");
  ros::NodeHandle nh;
  readParameters(nh);
  for (int i = 0; i < NUM_OF_CAM; i++)
    readIntrinsicParameter(CAM_NAMES[i]);
  std::cout<<"PROJECT_NAME "<<PROJECT_NAME<<std::endl;
  message_filters::Subscriber<sensor_msgs::PointCloud> point_feature_sub(nh, PROJECT_NAME + "/vins/feature/feature", 1000, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::PointCloud> line_feature_sub(nh, PROJECT_NAME + "/vins/feature/linefeature", 1000, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/edge", 1000,ros::TransportHints().tcpNoDelay());
  pub_point_line = nh.advertise<sensor_msgs::Image>(PROJECT_NAME + "/vins/feature/PointLine_image", 1000);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_feature_sub, line_feature_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::Rate loop_rate(30);
  while (nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
