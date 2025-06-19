// /**
//  *  \author     Claus Smitt <claus.smitt@ib.edu.ar.com>
//  *
//  * ROS Wrapper for REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
//  * Copyright (C) 2016  Juan José Tarrio

//  * Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
//  * for a Monocular Camera. In Proceedings of the IEEE International Conference
//  * on Computer Vision (pp. 702-710).

//  * This program is free software; you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation; either version 3 of the License, or
//  * (at your option) any later version.
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  * You should have received a copy of the GNU General Public License
//  * along with this program; if not, write to the Free Software Foundation,
//  * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
//  */
#ifndef REBVO_INCLUDE_REBVO_REBVO_NODELET_H_
#define REBVO_INCLUDE_REBVO_REBVO_NODELET_H_

#include <string>
#include <fstream>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "rebvo/rebvo.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#undef Success
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
namespace rebvo {
class RebvoNodelet: public nodelet::Nodelet {
public:
	RebvoNodelet();
	~RebvoNodelet();

private:
	


    bool systemInited; 

	virtual void onInit();

	/**
	 * Load rebvo parameters
	 */
	void construct();
	/**
	 * edgeMap subscription callback
	 */
	void connectCb();
	/**
	 * edgeMap un-subscription callback
	 */
	void disconnectCb();

	//void voDataHandler(const nav_msgs::Odometry::ConstPtr &voData);
	/**
	 * Intra-process Camera image callback
	 * @param image constant pointer to published image
	 */
	void imageCb(const sensor_msgs::ImageConstPtr &image);


	bool edgeMapPubCb(rebvo::PipeBuffer &edgeMapRebvo);

	ros::NodeHandle nh_, private_nh_; //Private for namespace rebvo
	ros::Subscriber camera_sub_;
	ros::Publisher edgeMap_pub_,image_show_pub_,images_info_pub_,edgedes_pub_;

	//ros::Timer clean_timer_;

	std::string imageTopic_;
    std::string frame_id_cam;
    std::string frame_id_robot;
    std::string dataset;


	std::unique_ptr<rebvo::REBVO> rebvo_;
	std::ofstream outFile;
    cv_bridge::CvImage bridge;

    int rows;
    int cols;
    int count;
    PipeBuffer last_pipe_;
    bool pipiBuffer_init_;

	double k_vlp2cam_[6];
};

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
		std::shared_ptr<rebvo::Image<rebvo::RGB24Pixel> > &imgRebvo);
bool rebvo2Mat(Image<RGB24Pixel>*imgRebvo, cv::Mat& img );

//edgeMap2msg(const revboLib::keyLine * keylines, rebvo::EdgeMapMsg &edgeMap);

}// namespace rebvo

#endif /* REBVO_INCLUDE_REBVO_REBVO_NODELET_H_ */
