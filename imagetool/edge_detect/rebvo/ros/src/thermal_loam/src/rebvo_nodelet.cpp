/**
 *  \author     Claus Smitt <claus.smitt@ib.edu.ar.com>
 *
 * ROS Wrapper for REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
 * Copyright (C) 2016  Juan José Tarrio

 * Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
 * for a Monocular Camera. In Proceedings of the IEEE International Conference
 * on Computer Vision (pp. 702-710).

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */
#include "../../include/rebvo/rebvo_nodelet.h"

#include <sensor_msgs/PointCloud.h>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "TooN/so3.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pluginlib/class_list_macros.h"
#include "rebvo/EdgeMap.h"
#include "sensor_msgs/image_encodings.h"
#include "vloam/cameraParameters.h"

#include "daliti/images_info.h"


#define PI 3.14159265
double mean_tmp = 0;
namespace rebvo {


std::mutex m_origionimage_buf;

std::queue<sensor_msgs::ImageConstPtr> origion_image_buf;

RebvoNodelet::RebvoNodelet() {
}

RebvoNodelet::~RebvoNodelet() {
    camera_sub_.shutdown();
}

void RebvoNodelet::onInit() {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    NODELET_DEBUG("Initializing REBVO nodelet");

    construct();

    rebvo_->setOutputCallback(&RebvoNodelet::edgeMapPubCb, this);

    // TODO initialize REBVO
    if (!rebvo_->Init()) {
        NODELET_ERROR("Error while initializing rebvoLib");
        return;
    }

    edgeMap_pub_ = nh_.advertise<EdgeMap>("rebvo_edgemap", 10,
    		boost::bind(&RebvoNodelet::connectCb, this),
    		boost::bind(&RebvoNodelet::disconnectCb, this));

    private_nh_.param<std::string>("rebvo/image_topic", imageTopic_, "/convert_image");
    private_nh_.param<std::string>("rebvo/frame_id_cam", frame_id_cam, "rebvo_frame_cam");
    private_nh_.param<std::string>("rebvo/frame_id_robot", frame_id_robot, "rebvo_frame_robot");
    private_nh_.param<std::string>("rebvo/dataset", dataset, "rebvo_frame_robot");

    camera_sub_ = nh_.subscribe(imageTopic_, 100, &RebvoNodelet::imageCb, this);

    // outFile.open("/home/wangyu/v1.txt",std::ios::app);

    image_show_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/edge", 5);
    images_info_pub_ = nh_.advertise<daliti::images_info>("/images_info", 5);
    edgedes_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/edgedes", 5);

    count = 0;
    pipiBuffer_init_ = false;
}

void RebvoNodelet::connectCb() {
    if (!camera_sub_ && edgeMap_pub_.getNumSubscribers() > 0) {
    	NODELET_WARN("Connecting to camera topic.");
    }
    if (!camera_sub_) {
        NODELET_WARN("Connecting to camera topic.");
    }
};

void RebvoNodelet::disconnectCb() {
    if (edgeMap_pub_.getNumSubscribers() == 0) {
    	NODELET_WARN("Unsubscribing from camera topic.");
    }
}

void RebvoNodelet::imageCb(const sensor_msgs::ImageConstPtr &image) {
    std::shared_ptr<Image<RGB24Pixel> > imgRebvoPtr;

    if (!rebvo_->requestCustomCamBuffer(imgRebvoPtr,
                                        image->header.stamp.toSec(), 0.001)) {
        NODELET_ERROR("Droping Frame");
        return;
    }
    m_origionimage_buf.lock();
    origion_image_buf.push(image);
    m_origionimage_buf.unlock();

    std::cout << std::fixed;
    std::cout << std::setprecision(7);
    // std::cout << image->header.stamp.toSec() << " "
    //           << "input" << std::endl;
    if (!imgMsg2Rebvo(image, imgRebvoPtr))
        NODELET_ERROR_STREAM("Img msg doesn't match with revbo config, only MONO8 and rgb8 encodings are suported. Img Size:" << image->width << "x" << image->height << " Encoding:" << image->encoding);

    rebvo_->releaseCustomCamBuffer();
}
cv::Mat distance(cv::Mat image) {
    cv::Mat edge_left = image;
    cv::Mat dst1;
    cv::Mat dist = image.clone();
    cv::distanceTransform(cv::Scalar(255) - edge_left, dst1, CV_DIST_L2, 3);  
    cv::normalize(dst1, dst1, 0, 225, CV_MINMAX);                            
    float *p;
    uchar *q;
    for (int i = 0; i < dst1.rows; i++) {
        p = dst1.ptr<float>(i);  
        q = dist.ptr<uchar>(i);  
        for (int j = 0; j < dst1.cols; ++j) {
            int temp = round(p[j]);
            q[j] = 255 - temp;
        }
    }
    return dist;
}

bool RebvoNodelet::edgeMapPubCb(PipeBuffer &edgeMapRebvo) {
    ros::Time msg_stamp;
    msg_stamp.fromSec(edgeMapRebvo.t);
    cv::Mat img(edgeMapRebvo.imgc->Size().h, edgeMapRebvo.imgc->Size().w, CV_8UC1, cv::Scalar(0));
    // cv::Mat img_des(edgeMapRebvo.imgc->Size().h,edgeMapRebvo.imgc->Size().w,CV_16SC6,cv::Scalar(0));
    // rebvo2Mat(edgeMapRebvo.imgc,img);
    int KLcout = 0;
    sensor_msgs::PointCloudPtr Edgedes(new sensor_msgs::PointCloud);
    sensor_msgs::ChannelFloat32 mean_value, desValid, des0, des1, des2, des3, des4;  //  feature id

    Edgedes->header.stamp = msg_stamp;
    Edgedes->header.frame_id = "vins_body_imuhz";
    for (KeyLine &kl : (*edgeMapRebvo.ef)) {
        KLcout++;
        // if(kl.c_p.y<10)
        //  continue;
        // if(kl.c_p.y>270)
        //  continue;
        // if(kl.c_p.x<10)
        //  continue;
        // if(kl.c_p.x>370)
        //  continue;
        // img.at<uchar>(int(kl.c_p.y),int(kl.c_p.x)) =(*edgeMapRebvo.imgc)(int(kl.c_p.x), int(kl.c_p.y)).pix.r;;
        img.at<uchar>(int(kl.c_p.y), int(kl.c_p.x)) = 255;
    }
    // 	for(int i = 5;i<img.rows-5;i++)
    // {
    // 	for(int j = 5;j<img.cols-5;j++)
    // 	{
    // 		int num = 0;
    // 		if(img.at<uchar>(i,j) ==255)
    // 		{
    // 			for(int k = -5;k<5;k++)
    // 			{
    // 				for(int l = -5;l<5;l++)
    // 				{
    // 					if(k==0&&l==0)
    // 						continue;
    // 					if(img.at<uchar>(i+k,j+l)==255)
    // 						num++;
    // 				}
    // 			}
    // 		}
    // 		if(num < 5)
    // 			img.at<uchar>(i,j) =0;
    // 	}
    // }

    // start Mask only for sthereo and hitsz dataset @note
    if(dataset=="sthereo"){
        cv::Mat sthmask(512, 640, CV_8UC1, cv::Scalar(0));
        // generate mask
        for (int y = 0; y < sthmask.rows; y++)
        {
            for (int x = 0; x < sthmask.cols; x++)
            {
                if (y < 400)
                {
                    sthmask.at<uchar>(y, x) = 255;
                }
                else
                {
                    sthmask.at<uchar>(y, x) = 0;
                }
            }
        }
        cv::bitwise_and(img,sthmask,img);      
    }
    else if(dataset=="hitsz"){
        cv::Mat sthmask(480, 640, CV_8UC1, cv::Scalar(0));
        // generate mask
        for (int y = 0; y < sthmask.rows; y++)
        {
            for (int x = 0; x < sthmask.cols; x++)
            {
                if (x > 20 && x < 620)
                {
                    sthmask.at<uchar>(y, x) = 255; 
                }
                else
                {
                    sthmask.at<uchar>(y, x) = 0;
                }
            }
        }
        cv::bitwise_and(img,sthmask,img);      
    }
    // end Mask only for sthereo and hitsz dataset

    // start : add this line only for debug
    // img = cv::Mat::zeros(img.size(), img.type());
    // end : add this line only for debug
    //
    // cv::imwrite("~/edge.png",img);
    //

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    msg->header.stamp = msg_stamp;
    std::ostringstream oss;
    oss<<KLcout;
    std::string edgenum=oss.str();
    msg->header.frame_id = edgenum;
    image_show_pub_.publish(msg);

    daliti::images_info images;
    images.header=msg->header;
    images.edge_image=*msg;
    sensor_msgs::ImageConstPtr origion_image_msg;
    if (!origion_image_buf.empty())
    {
        m_origionimage_buf.lock();
        origion_image_msg = origion_image_buf.front();
        origion_image_buf.pop();
        m_origionimage_buf.unlock();
    }
    images.origion_image=*origion_image_msg;
    images_info_pub_.publish(images);

    // cv::Mat gray;
    // cv::cvtColor(imgLast,gray,cv::COLOR_BGR2GRAY);
    // double mean_tmp = mean(gray)[0];
    //  std::cout<<mean_tmp<<std::endl;
    for (KeyLine &kl : (*edgeMapRebvo.ef)) {
        geometry_msgs::Point32 p;
        p.x = int(kl.c_p.x);
        p.y = int(kl.c_p.y);
        p.z = 1;
        Edgedes->points.push_back(p);
        if (kl.desValid == true) {
            desValid.values.push_back(1);
            des0.values.push_back(kl.des[0]);
            des1.values.push_back(kl.des[1]);
            des2.values.push_back(kl.des[2]);
            des3.values.push_back(kl.des[3]);
            des4.values.push_back(kl.des[4]);
        } else {
            desValid.values.push_back(0);
            des0.values.push_back(0);
            des1.values.push_back(0);
            des2.values.push_back(0);
            des3.values.push_back(0);
            des4.values.push_back(0);
        }
    }
    // mean_value.values.push_back(mean_tmp);
    // Edgedes->channels.push_back(mean_value);
    Edgedes->channels.push_back(desValid);
    Edgedes->channels.push_back(des0);
    Edgedes->channels.push_back(des1);
    Edgedes->channels.push_back(des2);
    Edgedes->channels.push_back(des3);
    Edgedes->channels.push_back(des4);
    edgedes_pub_.publish(Edgedes);

    // // img = distance(img);
    // // cv::distanceTransform(cv::Scalar::all(255)-img,
    // //                     distance,
    // //                     cv::DIST_L2,
    // //                     3);
    // // cv::normalize(distance, distance, 0, 1, cv::NORM_MINMAX);
    // // cv::imshow("1",img);
    // cv::imshow("2",distance(img));
    // cv::waitKey(1);
    return true;
}

bool rebvo2Mat(Image<RGB24Pixel> *imgRebvo, cv::Mat &img) {
    if (imgRebvo->Size().w != img.cols && imgRebvo->Size().h != img.rows)
        return false;
    for (int y = 0; y < imgRebvo->Size().h; ++y) {
        for (int x = 0; x < imgRebvo->Size().w; ++x) {
            img.at<cv::Vec3b>(y, x)[0] = (*imgRebvo)(x, y).pix.r;
            img.at<cv::Vec3b>(y, x)[1] = (*imgRebvo)(x, y).pix.g;
            img.at<cv::Vec3b>(y, x)[2] = (*imgRebvo)(x, y).pix.b;
        }
    }
    return true;
}

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
                  std::shared_ptr<Image<RGB24Pixel> > &imgRebvo) {
    // mean_tmp = 0;
    if (imgMsg->width != imgRebvo->Size().w || imgMsg->height != imgRebvo->Size().h)
        return false;

    if (imgMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0) {
        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x];
                // mean_tmp = mean_tmp + imgMsg->data[imgMsg->step * y + x];
            }
        }
        // mean_tmp = mean_tmp/imgRebvo->Size().h/imgRebvo->Size().w;
    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0) {
        double minv = 0, maxv = 0;
        double *minp = &minv;
        double *maxp = &maxv;
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::minMaxIdx(ptr->image, minp, maxp);  
        // for (int y = 0; y < imgRebvo->Size().h; ++y) {
        // 	for (int x = 0; x < imgRebvo->Size().w; ++x) {
        // 		(*imgRebvo)(x, y).pix.r = (imgMsg->data[imgMsg->step * y + x]- minv)/ (maxv - minv) * 255;
        // 		(*imgRebvo)(x, y).pix.g = (imgMsg->data[imgMsg->step * y + x]- minv)/ (maxv - minv) * 255;
        // 		(*imgRebvo)(x, y).pix.b = (imgMsg->data[imgMsg->step * y + x]- minv)/ (maxv - minv) * 255;
        // 	}
        // }
        int size = 3;
        std::cout << 22222222 << std::endl;

        int sum = 0;
        int mean = 0;
        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                if (y < size || y >= imgRebvo->Size().h - size || x < size || x >= imgRebvo->Size().w - size) {
                    (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x];
                    (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x];
                    (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x];
                } else {
                    for (int r = y - size; r <= y + size; ++r) {
                        for (int c = x - size; c <= x + size; ++c) {
                            sum = imgMsg->data[imgMsg->step * r + c] + sum;
                        }
                    }
                    mean = sum / (7 * 7);
                    // if(mean<minv)
                    //	mean = 0;
                    (*imgRebvo)(x, y).pix.r = mean;
                    (*imgRebvo)(x, y).pix.g = mean;
                    (*imgRebvo)(x, y).pix.b = mean;
                    sum = 0;
                    mean = 0;
                }
            }
        }
    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) {
        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x * 3 + 0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x * 3 + 1];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x * 3 + 2];
            }
        }
    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0) {
        // std::cout<<"3"<<std::endl;
        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x * 3 + 0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x * 3 + 1];
                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x * 3 + 2];
            }
        }

    } else {
        return false;
    }
    return true;
}

void RebvoNodelet::construct() {
    REBVOParameters rebvoPars;

    std::vector<double> transCam2ImuPar;
    std::vector<double> rotCam2ImuPar;

    TooN::Vector<3> transCam2Imu;
    TooN::Matrix<3, 3> rotCam2Imu;

    // Rebvo Detector parameters
    private_nh_.param<double>("rebvo/detector/Sigma0", rebvoPars.Sigma0, 3.56359);
    private_nh_.param<double>("rebvo/detector/KSigma", rebvoPars.KSigma, 1.2599);
    private_nh_.param<int>("rebvo/detector/ReferencePoints", rebvoPars.ReferencePoints, 18000);
    private_nh_.param<int>("rebvo/detector/MaxPoints", rebvoPars.MaxPoints, 20000);
    private_nh_.param<int>("rebvo/detector/TrackPoints", rebvoPars.TrackPoints, rebvoPars.MaxPoints);
    private_nh_.param<double>("rebvo/detector/DetectorThresh", rebvoPars.DetectorThresh, 0.01);
    private_nh_.param<double>("rebvo/detector/DetectorAutoGain", rebvoPars.DetectorAutoGain, 5e-7);
    private_nh_.param<double>("rebvo/detector/DetectorMaxThresh", rebvoPars.DetectorMaxThresh, 0.5);
    private_nh_.param<double>("rebvo/detector/DetectorMinThresh", rebvoPars.DetectorMinThresh, 0.01);  // 0.01
    private_nh_.param<int>("rebvo/detector/DetectorPlaneFitSize", rebvoPars.DetectorPlaneFitSize, 2);
    private_nh_.param<double>("rebvo/detector/DetectorPosNegThresh", rebvoPars.DetectorPosNegThresh, 0.4);
    // private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.001);//mine
    // private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.00526);//parking
    private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.001);  // parking

    // private_nh_.param<double>("rebvo/detector/Sigma0", rebvoPars.Sigma0, 3.56359);
    // private_nh_.param<double>("rebvo/detector/KSigma", rebvoPars.KSigma, 1.2599);
    // private_nh_.param<int>("rebvo/detector/ReferencePoints", rebvoPars.ReferencePoints, 18000);
    // private_nh_.param<int>("rebvo/detector/MaxPoints", rebvoPars.MaxPoints, 20000);
    // private_nh_.param<int>("rebvo/detector/TrackPoints", rebvoPars.TrackPoints, rebvoPars.MaxPoints);
    // private_nh_.param<double>("rebvo/detector/DetectorThresh", rebvoPars.DetectorThresh, 0.01);
    // private_nh_.param<double>("rebvo/detector/DetectorAutoGain", rebvoPars.DetectorAutoGain, 5e-7);
    // private_nh_.param<double>("rebvo/detector/DetectorMaxThresh", rebvoPars.DetectorMaxThresh, 0.5);
    // private_nh_.param<double>("rebvo/detector/DetectorMinThresh", rebvoPars.DetectorMinThresh, 0.01);//0.01
    // private_nh_.param<int>("rebvo/detector/DetectorPlaneFitSize", rebvoPars.DetectorPlaneFitSize, 2);
    // private_nh_.param<double>("rebvo/detector/DetectorPosNegThresh", rebvoPars.DetectorPosNegThresh, 0.4);
    // private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.095259868922420);//0.195259868922420

    private_nh_.param<double>("rebvo/TrackMaper/SearchRange", rebvoPars.SearchRange, 1);
    private_nh_.param<double>("rebvo/TrackMaper/QCutOffNumBins", rebvoPars.QCutOffNumBins, 1);
    private_nh_.param<double>("rebvo/TrackMaper/QCutOffQuantile", rebvoPars.QCutOffQuantile, 0.9);
    private_nh_.param<int>("rebvo/TrackMaper/TrackerIterNum", rebvoPars.TrackerIterNum, 5);
    private_nh_.param<int>("rebvo/TrackMaper/TrackerInitType", rebvoPars.TrackerInitType, 2);
    private_nh_.param<int>("rebvo/TrackMaper/TrackerInitIterNum", rebvoPars.TrackerInitIterNum, 2);
    private_nh_.param<double>("rebvo/TrackMaper/TrackerMatchThresh", rebvoPars.TrackerMatchThresh, 0.5);
    private_nh_.param<int>("rebvo/TrackMaper/MatchNumThresh", (int &)rebvoPars.MatchNumThresh, 0);
    private_nh_.param<double>("rebvo/TrackMaper/ReweigthDistance", rebvoPars.ReweigthDistance, 2);

    // Rebvo camera parameters

    private_nh_.param<float>("rebvo/Camera/ZfX", rebvoPars.z_f_x, 4.616e+02);
    private_nh_.param<float>("rebvo/Camera/ZfY", rebvoPars.z_f_y, 4.603e+02);
    private_nh_.param<float>("rebvo/Camera/PPx", rebvoPars.pp_x, 3.630e+02);
    private_nh_.param<float>("rebvo/Camera/PPy", rebvoPars.pp_y, 248.375);

    private_nh_.param<double>("rebvo/Camera/KcR2", rebvoPars.kc.Kc2, -0.28340811);
    private_nh_.param<double>("rebvo/Camera/KcR4", rebvoPars.kc.Kc4, 0.07395907);
    private_nh_.param<double>("rebvo/Camera/KcR6", rebvoPars.kc.Kc6, 0);
    private_nh_.param<double>("rebvo/Camera/KcP1", rebvoPars.kc.P1, 0.00019359);
    private_nh_.param<double>("rebvo/Camera/KcP2", rebvoPars.kc.P2, 1.76187114e-05);

    private_nh_.param<int>("rebvo/Camera/ImageWidth", (int &)rebvoPars.ImageSize.w, 1);
    private_nh_.param<int>("rebvo/Camera/ImageHeight", (int &)rebvoPars.ImageSize.h, 1);

    cols = rebvoPars.ImageSize.w;
    rows = rebvoPars.ImageSize.h;

    private_nh_.param<double>("rebvo/Camera/FPS", rebvoPars.config_fps, 30);
    private_nh_.param<double>("rebvo/Camera/SoftFPS", rebvoPars.soft_fps, rebvoPars.config_fps);
    private_nh_.param<bool>("rebvo/Camera/UseUndistort", rebvoPars.useUndistort, false);
    private_nh_.param<bool>("rebvo/Camera/Rotate180", rebvoPars.rotatedCam, 0);

    // Copy and set rebvo cam2imu transformation
    if (transCam2ImuPar.size() == 3) {
        for (int i = 0; i < 3; ++i)
            transCam2Imu[i] = transCam2ImuPar[i];
    } else {
        transCam2Imu = TooN::Zeros;
    }

    if (rotCam2ImuPar.size() == 9) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotCam2Imu[i][j] = rotCam2ImuPar[i * 3 + j];
            }
        }
    } else {
        rotCam2Imu = TooN::Identity;
    }

    private_nh_.param<double>("rebvo/transVlp2Cam/tx", k_vlp2cam_[0], 0);
    private_nh_.param<double>("rebvo/transVlp2Cam/ty", k_vlp2cam_[1], 0);
    private_nh_.param<double>("rebvo/transVlp2Cam/tz", k_vlp2cam_[2], 0);
    private_nh_.param<double>("rebvo/transVlp2Cam/rx", k_vlp2cam_[3], 0);
    private_nh_.param<double>("rebvo/transVlp2Cam/ry", k_vlp2cam_[4], 0);
    private_nh_.param<double>("rebvo/transVlp2Cam/rz", k_vlp2cam_[5], 0);

    // rebvo ProcesorConfig parameters
    private_nh_.param<int>("rebvo/ProcesorConfig/SetAffinity", rebvoPars.cpuSetAffinity, 1);
    private_nh_.param<int>("rebvo/ProcesorConfig/Thread1", rebvoPars.cpu0, 1);
    private_nh_.param<int>("rebvo/ProcesorConfig/Thread2", rebvoPars.cpu1, 2);
    private_nh_.param<int>("rebvo/ProcesorConfig/Thread3", rebvoPars.cpu2, 3);

    rebvo_ = std::unique_ptr<REBVO>(new REBVO(rebvoPars));
}
// TODO
//  PLUGINLIB_DECLARE_CLASS(rebvo, RebvoNodelet, rebvo::RebvoNodelet,nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(rebvo::RebvoNodelet, nodelet::Nodelet);

}  // namespace rebvo
