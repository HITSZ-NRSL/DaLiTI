#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0


// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
sensor_msgs::PointCloud2ConstPtr origion_deskewed_cloud;

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_match;
ros::Publisher pub_restart;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;
double cost_time_total = 0;
long   total_frame_idx = 0;
double cur_img_time = 0;
double image_timestamp_bias = 0.0;

// RECV_LIO_FAIL_FLAG
bool RECV_LIO_FAIL_FLAG=false;


Eigen::Vector3d tmp_ba(0,0,0);
Eigen::Vector3d tmp_bg(0,0,0);
Eigen::Vector3d tmp_g(0,0,9.81);
int init_flag = 0;
std::vector<sensor_msgs::Imu> imu_msg_buffer;
void imu_callback(
        const sensor_msgs::ImuConstPtr &msg) {
    // Wait for the first image to be set.
    if (first_image_flag) return;
    imu_msg_buffer.push_back(*msg);
}
Matrix3d integrateImuData() {
    if (imu_msg_buffer.empty()) {
        std::cout<<"empty IMU!"<<std::endl;
        return Matrix3d::Identity();
    }
    // Find the start and the end limit within the imu msg buffer.
    auto begin_iter = imu_msg_buffer.begin();
    while (begin_iter != imu_msg_buffer.end()) {
        if ((begin_iter->header.stamp.toSec() - last_image_time) < -0.01)
            ++begin_iter;
        else
            break;
    }

    auto end_iter = begin_iter;
    while (end_iter != imu_msg_buffer.end()) {
        if ((end_iter->header.stamp.toSec() - cur_img_time) < 0.005)
            ++end_iter;
        else
            break;
    }

    // Compute the mean angular velocity in the IMU frame.
    Vector3d mean_ang_vel(0.0, 0.0, 0.0);
    Matrix3d Rs;
    for (auto iter = begin_iter; iter < end_iter; ++iter)
    {
        double t0,t1;
        Vector3d gyr_0,gyr_1;
        if(iter == begin_iter)
        {
            gyr_0 = Vector3d(iter->angular_velocity.x,
                                 iter->angular_velocity.y, iter->angular_velocity.z);
            Rs.setIdentity();
            t0 = iter->header.stamp.toSec();
            continue;
        }
        gyr_1 = Vector3d(iter->angular_velocity.x,
                                 iter->angular_velocity.y, iter->angular_velocity.z);
        Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1)-tmp_bg;
        t1 = iter->header.stamp.toSec();
        double dt = t1-t0;

        Vector3d cam0_mean_ang_vel = Ric.transpose() *  un_gyr;
        // Compute the relative rotation.
        Vector3d cam0_angle_axisd = cam0_mean_ang_vel * dt;
        Rs *= AngleAxisd(cam0_angle_axisd.norm(), cam0_angle_axisd.normalized()).toRotationMatrix().transpose();

        //Rs *= deltaQ(un_gyr * dt).toRotationMatrix();
        t0 = t1;
        gyr_0 = gyr_1;
    }
    imu_msg_buffer.erase(imu_msg_buffer.begin(), end_iter);
    return Rs;
}

void data_callback(const std_msgs::Float32MultiArray &data_msg)
{
    tmp_ba(0) = data_msg.data.at(0);
    tmp_ba(1) = data_msg.data.at(1);
    tmp_ba(2) = data_msg.data.at(2);

    tmp_bg(0) = data_msg.data.at(3);
    tmp_bg(1) = data_msg.data.at(4);
    tmp_bg(2) = data_msg.data.at(5);

    tmp_g(0) = data_msg.data.at(6);
    tmp_g(1) = data_msg.data.at(7);
    tmp_g(2) = data_msg.data.at(8);

    init_flag = data_msg.data.at(9);
}


void multi_callback_1(const daliti::images_infoConstPtr &imgs_msg, const sensor_msgs::PointCloudConstPtr &edgedes_msg)
{

}
std::ofstream feature_tracker_time_file("~/daliti/src/DaLiTI/eskf_lio/Log/feature_tracker_time.txt", std::ios::out | std::ios::app);

void multi_callback(const daliti::images_infoConstPtr &imgs_msg, const sensor_msgs::PointCloudConstPtr &edgedes_msg)
{ 
    //cout <<"Image timestamp: " <<img_msg->header.stamp.toSec() << endl;
    //printf("Image timestamp = %lf\r\n" , img_msg->header.stamp.toSec());
    sensor_msgs::Image edge_img_msg=imgs_msg->edge_image;
    sensor_msgs::Image origion_img_msg=imgs_msg->origion_image;
    cur_img_time = edge_img_msg.header.stamp.toSec();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImagePtr ptr_edge_image;
    cv_bridge::CvImagePtr ptr_origion_image;
    if (edge_img_msg.encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = edge_img_msg.header;
        img.height = edge_img_msg.height;
        img.width = edge_img_msg.width;
        img.is_bigendian = edge_img_msg.is_bigendian;
        img.step = edge_img_msg.step;
        img.data = edge_img_msg.data;
        img.encoding = "mono8";
        ptr_edge_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr_edge_image = cv_bridge::toCvCopy(edge_img_msg, sensor_msgs::image_encodings::MONO8);
    ptr_edge_image->header.stamp.fromSec(cur_img_time - image_timestamp_bias);
    cv::Mat edge_img = ptr_edge_image->image;

    ptr_origion_image = cv_bridge::toCvCopy(origion_img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat origion_img=ptr_origion_image->image;
    TicToc t_r;
    std::istringstream iss(edge_img_msg.header.frame_id);
    int edge_num;
    iss>>edge_num;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
        {
            Matrix3d relative_R = integrateImuData();
            Eigen::AngleAxisd debugrotation_vector(relative_R);
            trackerData[i].readImage(ptr_edge_image->image.rowRange(ROW * i, ROW * (i + 1)), edge_img_msg.header.stamp.toSec(),relative_R,edgedes_msg,init_flag,edge_num);
        }
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr_edge_image->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr_edge_image->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }
    
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }
    
   if (PUB_THIS_FRAME)
   {
        
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = edge_img_msg.header;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(edge_img_msg.header.stamp, edge_img, origion_img, origion_deskewed_cloud, depth_cloud_temp, trackerData[0].m_camera, feature_points);
        feature_points->channels.push_back(depth_of_points);

        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr_edge_image = cv_bridge::cvtColor(ptr_edge_image, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr_edge_image->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(edge_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                     //draw speed line
                    
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    
                    // char name[10];
                    // sprintf(name, "%d", trackerData[i].ids[j]);
                    // cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                }
            }
                }
            }

            pub_match.publish(ptr_edge_image->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());

    if (feature_tracker_time_file.is_open())
    {
        feature_tracker_time_file << t_r.toc() << std::endl;
    }

    cost_time_total = t_r.toc() + cost_time_total;
    total_frame_idx++;
    //ROS_INFO("Average front-end cost time: %f", cost_time_total / total_frame_idx );
}

void lidar_status_callback(const std_msgs::Header& lidar_status_msg)
{
    if(lidar_status_msg.frame_id == "lidar data failed")
    {
        RECV_LIO_FAIL_FLAG=1;
    }
    else 
    {
        RECV_LIO_FAIL_FLAG=0;
    }
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. listen to transform
    static tf::TransformListener listener;
#if IF_OFFICIAL
    static tf::StampedTransform transform;   //; T_vinsworld_camera_FLU
#else
    static tf::StampedTransform transform_world_cFLU;   //; T_vinsworld_camera_FLU
    static tf::StampedTransform transform_C_L;    //; T_cameraFLU_imu
#endif
    try{
        listener.waitForTransform("camera_init", "camera_link", laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("camera_init", "camera_link", laser_msg->header.stamp, transform_world_cFLU);
        listener.waitForTransform("camera_link","lidar_link" , laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("camera_link","lidar_link", laser_msg->header.stamp, transform_C_L);
    }
    catch (tf::TransformException ex){
        ROS_WARN("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
#if IF_OFFICIAL
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
#else
    xCur = transform_world_cFLU.getOrigin().x();
    yCur = transform_world_cFLU.getOrigin().y();
    zCur = transform_world_cFLU.getOrigin().z();
    tf::Matrix3x3 m(transform_world_cFLU.getRotation());
#endif
    m.getRPY(rollCur, pitchCur, yawCur);
    //; T_vinswolrd_cameraFLU
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    origion_deskewed_cloud = laser_msg;

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    
#if IF_OFFICIAL
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;
#else
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    double roll, pitch, yaw, x, y, z;
    x = transform_C_L.getOrigin().getX();
    y = transform_C_L.getOrigin().getY();
    z = transform_C_L.getOrigin().getZ();
    tf::Matrix3x3(transform_C_L.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transOffset = pcl::getTransformation(x, y, z, roll, pitch, yaw);
    
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;
#endif

    // 4. filter lidar points (only keep points in camera view)
    
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    if (!RECV_LIO_FAIL_FLAG) // if LIO not failed,save
    {
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);
    } // else do nothing

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 15.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual point Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    readParameters(n);

    // read camera params
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // load fisheye mask to remove features on the boundry
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_ERROR("load fisheye mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // initialize depthRegister (after readParameters())
    depthRegister = new DepthRegister(n);
    message_filters::Subscriber<daliti::images_info> subscriber_imageinfo(n,"/images_info",1000,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> subscriber_edgeimage(n,"/camera/edge",1000,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::PointCloud> subscriber_edgedes(n,"/edgedes",1000,ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<daliti::images_info, sensor_msgs::PointCloud> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subscriber_imageinfo, subscriber_edgedes);  
    sync.registerCallback(boost::bind(&multi_callback, _1, _2));

    // subscriber to image and lidar
    // ros::Subscriber sub_img   = n.subscribe(IMAGE_TOPIC,       5,    img_callback);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 1000, imu_callback);
    ros::Subscriber sub_data = n.subscribe("/Intrinsic", 1000, data_callback); //TODO
    ros::Subscriber sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 100, lidar_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_lio_failed_flag = n.subscribe(LIDAR_STATUS_TOPIC, 5,    lidar_status_callback);
    if (!USE_LIDAR)
        sub_lidar.shutdown();
        
    // messages to vins estimator
    pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);
    pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
    pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);

    // two ROS spinners for parallel processing (image and lidar)
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    if (feature_tracker_time_file.is_open())
    {
        feature_tracker_time_file.close();
    }

    return 0;
}