#include "utility.h"

class GetSyncData : public ParamServer
{
public:
    using pcMsg = sensor_msgs::PointCloud2;
    using imgMsg = sensor_msgs::Image;
    using syncPolicy = message_filters::sync_policies::ApproximateTime<pcMsg, imgMsg>;
    using pcMFSub = message_filters::Subscriber<pcMsg>;
    using pcMFSubPtr = boost::shared_ptr<pcMFSub>;
    using imgMFSub = message_filters::Subscriber<imgMsg>;
    using imgMFSubPtr = boost::shared_ptr<imgMFSub>;
    using MFSync = message_filters::Synchronizer<syncPolicy>;
    using MFSyncPtr = boost::shared_ptr<MFSync>;
    GetSyncData();

private:
    void sync_callback(pcMsg::ConstPtr pc, imgMsg::ConstPtr img);

    // ros
    pcMFSubPtr pc_mf_sub_;
    imgMFSubPtr img_mf_sub_;
    MFSyncPtr synchronizer_;

    // thread
    std::mutex mu_;

    double freq_;
};
GetSyncData::GetSyncData() : freq_(10)
{
    // initialization
    pc_mf_sub_ = pcMFSubPtr(new pcMFSub(nh, "/ouster/points", 50));
    img_mf_sub_ = imgMFSubPtr(new imgMFSub(nh, "/convert_image", 50));
    synchronizer_ = MFSyncPtr(new MFSync(syncPolicy(freq_), *pc_mf_sub_, *img_mf_sub_));
    synchronizer_->registerCallback(boost::bind(&GetSyncData::sync_callback, this, _1, _2));
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

void GetSyncData::sync_callback(pcMsg::ConstPtr pc, imgMsg::ConstPtr img)
{
    // lock_guard<mutex> mu_guard(mu_);
    ROS_DEBUG("Get:\npc:\t%f\nimg:\t%f", pc->header.stamp.toSec(), img->header.stamp.toSec());

    // cv_bridge::CvImagePtr ptr;
    // ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    // cv::Mat origImage = ptr->image;

    // pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>);
    // pcl::fromROSMsg(*pc, *pcl_cloud);

    // // 1111111111111111111
    // std::vector<std::pair<cv::Point2d, float>> points_2d_with_depth;
    // for (int i = 0; i < (int)pcl_cloud->size(); ++i)
    // {
    //     // convert points from 3D to 2D
    //     PointType p = pcl_cloud->points[i];
    //     Eigen::Vector3d p_3d(-p.y,-p.z,p.x);
    //     Eigen::Vector2d p_2d;
    //     camera_model->spaceToPlane(p_3d, p_2d);
    //     auto point_2d_with_depth = std::make_pair(cv::Point2d(p_2d(0), p_2d(1)), pointDistance(p));
    //     points_2d_with_depth.push_back(point_2d_with_depth);
    // }

    // cv::Mat showImage, circleImage;
    // cv::cvtColor(origImage, showImage, cv::COLOR_GRAY2RGB);
    // circleImage = showImage.clone();

    // for (int i = 0; i < (int)points_2d_with_depth.size(); ++i)
    // {
    //     float r, g, b;
    //     getColor(points_2d_with_depth[i].second, 50.0, r, g, b);
    //     cv::circle(circleImage, points_2d_with_depth[i].first, 1, cv::Scalar(r, g, b), 1);
    // }

    // cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

    // // cv_bridge::CvImage bridge;
    // // bridge.image = showImage;
    // // bridge.encoding = "rgb8";
    // // sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
    // // imageShowPointer->header.stamp = stamp_cur;
    // // pub_depth_image_depth.publish(imageShowPointer);
    // cv::imshow("showImage",showImage);
    // // 1111111111111111111

    // // // look up transform at current image time
    // // tf::StampedTransform transform;
    // // tf::TransformListener listener;
    // // try
    // // {
    // //     listener.waitForTransform("lidar_link", "camera_link", img->header.stamp, ros::Duration(1));
    // //     listener.lookupTransform("lidar_link", "camera_link", img->header.stamp, transform);
    // // }
    // // catch (tf::TransformException ex)
    // // {
    // //     ROS_WARN("L_C no tf");
    // // }

    // // double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    // // xCur = transform.getOrigin().x();
    // // yCur = transform.getOrigin().y();
    // // zCur = transform.getOrigin().z();
    // // tf::Matrix3x3 m(transform.getRotation());
    // // m.getRPY(rollCur, pitchCur, yawCur);
    // // Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // // Eigen::Matrix4d T_cam_lidar;
    // // T_cam_lidar.setIdentity();
    // // T_cam_lidar<<   1.0, 0.0, 0.0, 0.0308632,
    // //                 0.0, 1.0, 0.0, -0.0413318,
    // //                 0.0, 0.0, 1.0, 1,
    // //                 0.0, 0.0, 0.0, 1.0;
    // // pcl::transformPointCloud(*pcl_cloud,*pcl_cloud,T_cam_lidar);
    // // // pcl::transformPointCloud(*pcl_cloud,*pcl_cloud,transNow.inverse());

    // // for (const pcl::PointXYZ& point : pcl_cloud->points)
    // // {
    // //     // Project the 3D point onto the 2D image plane
    // //     int u = static_cast<int>(point.y / point.x * origImage.cols);
    // //     int v = static_cast<int>(point.z / point.x * origImage.rows);

    // //     // Check if the projected point is within the image boundaries
    // //     if (u >= 0 && u < origImage.cols && v >= 0 && v < origImage.rows)
    // //     {
    // //         // Calculate a color based on the depth (distance)
    // //         uchar color_value = static_cast<uchar>(255 * (point.x - pcl_cloud->points.front().x) /
    // //                                                 (pcl_cloud->points.back().x - pcl_cloud->points.front().x));
    // //         float b,g,r;
    // //         getColor(color_value,1,b,g,r);
    // //         cv::Vec3b color(b, g, r);

    // //         // Set the color in the projected image
    // //         origImage.at<cv::Vec3b>(v, u) = color;
    // //     }
    // // }

    // // cv::imshow("origImage",origImage);
    // cv::waitKey(1);
}
class FailureDetection : public ParamServer
{
public:
    ros::Subscriber SubLidar;
    ros::Subscriber SubOdom;
    ros::Publisher PubFailure;
    nav_msgs::Odometry::ConstPtr odom;
    pcl::PointCloud<PointType>::Ptr prev_lidar_cloud;
    bool between_frames_first_frame = 1;

    FailureDetection()
    {
        PubFailure = nh.advertise<std_msgs::Header>(PROJECT_NAME + "/lidar/status", 1);

        SubLidar = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_undistort", 5, &FailureDetection::lidarHandler, this, ros::TransportHints().tcpNoDelay());
        SubOdom = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &FailureDetection::odomHandler, this, ros::TransportHints().tcpNoDelay());
    }

    void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<PointType>::Ptr lidar_cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *lidar_cloud);

        float icp_noiseScore = 0;
        // ICP per 20 frames
        if (between_frames_first_frame)
        {
            prev_lidar_cloud = lidar_cloud;
            between_frames_first_frame = 0;
            return;
        }
        static int frameCount = 0;
        frameCount++;
        if (frameCount % 20 == 0)
        {
            icp_noiseScore = icp_between_frames(prev_lidar_cloud, lidar_cloud);
        }
        else if ((frameCount + 1) % 20 == 0)
        {
            prev_lidar_cloud = lidar_cloud;
        }
        // end ICP

        // failure checker
        // std::cout<<"lidar_cloud->size()"<<lidar_cloud->size()<<std::endl;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        if (lidar_cloud->size() < 1e4 || icp_noiseScore > 1.0)
        {
            header.frame_id = "lidar data failed";
        }
        else
        {
            header.frame_id = "lidar data valid";
        }
        PubFailure.publish(header);
    }

    void odomHandler(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom = msg;
    }

    float icp_between_frames(pcl::PointCloud<PointType>::Ptr prev_pt, pcl::PointCloud<PointType>::Ptr this_pt)
    {
        TicToc t_icp;

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(10);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(this_pt);
        icp.setInputTarget(prev_pt);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        // if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        // return;

        // Get pose transformation
        // float x, y, z, roll, pitch, yaw;
        // Eigen::Affine3f correctionLidarFrame;
        // correctionLidarFrame = icp.getFinalTransformation();
        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        // pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        // gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();

        ROS_DEBUG("noiseScore: %f icp costed time: %f", noiseScore, t_icp.toc());
        // std::cout<<"noiseScore: "<<noiseScore<<" icp costed time: "<<t_icp.toc()<<std::endl;
        return noiseScore;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daliti");

    FailureDetection failure_detection;

    GetSyncData get_sync_data;

    ROS_INFO("\033[1;32m----> Lidar Failure Detection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
