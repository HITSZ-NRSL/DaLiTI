#include "estimator.h"

#include "utility/visualization.h"
#include <stdio.h>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "./lio_preprocess/preprocess.h"

#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

Estimator estimator;
StatesGroup g_livo_state;
typedef std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> PL_Feature;
std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<PL_Feature> plfeature_buf;
queue<sensor_msgs::PointCloudConstPtr> point_feature_buf;
queue<sensor_msgs::PointCloudConstPtr> line_feature_buf;
queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;

shared_ptr<LIO_Preprocess> p_pre(new LIO_Preprocess());

int init_flag = 0;

// RECV_LIO_FAIL_FLAG
bool RECV_LIO_FAIL_FLAG = true;

// global variable saving the lidar odometry
deque<nav_msgs::Odometry> g_odomQueue;
odometryRegister *odomRegister;

// deque<double>                     g_lidar_time_buffer;
// deque<PointCloudXYZI::Ptr>        g_lidar_buffer;

std::mutex m_buf;
// std::mutex m_lidar_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
ros::Publisher chatter_pub;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    Eigen::Vector3d tmp_g = estimator.g;
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(tmp_Ba(0)); 
    msg.data.push_back(tmp_Ba(1));
    msg.data.push_back(tmp_Ba(2));
    msg.data.push_back(tmp_Bg(0));
    msg.data.push_back(tmp_Bg(1));
    msg.data.push_back(tmp_Bg(2));
    msg.data.push_back(tmp_g(0));
    msg.data.push_back(tmp_g(1));
    msg.data.push_back(tmp_g(2));
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        init_flag = 1;
    else
        init_flag = 0;
    msg.data.push_back(init_flag);
    chatter_pub.publish(msg);

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::pair<PL_Feature, sensor_msgs::PointCloud2ConstPtr>>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::pair<PL_Feature, sensor_msgs::PointCloud2ConstPtr>>> measurements;
    std::unique_lock<std::mutex> lk(m_buf);

    // ROS_INFO("in");

    // while (ros::ok())
    // {
    // ROS_INFO("in while");

    // if (imu_buf.empty() || plfeature_buf.empty())
    if (imu_buf.empty() || (plfeature_buf.empty() && lidar_buf.empty()))
        return measurements;

    // std::cout<<"imu_buf.size() "<<imu_buf.size()<<" plfeature_buf.size() "<<plfeature_buf.size()<<" lidar_buf.size() "<<lidar_buf.size()<<"\r"<<std::endl;
    bool image_first = true;
    if (!plfeature_buf.empty() && !lidar_buf.empty())
    {
        image_first = plfeature_buf.front().first->header.stamp.toSec() < lidar_buf.front()->header.stamp.toSec();
    }
    else if (!plfeature_buf.empty() && lidar_buf.empty())
    {
        image_first = true;
    }
    else if (plfeature_buf.empty() && !lidar_buf.empty())
    {
        image_first = false;
    }
    else
    {
        return measurements;
    }
    if (image_first)
    {
        // ROS_INFO("image feature in");
        // check if imu is later than the current image
        if (!(imu_buf.back()->header.stamp.toSec() > plfeature_buf.front().first->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("imu to feature timestamp error, check if imu is later than the current image");
            return measurements;
        }

        // check if throw img
        if (!(imu_buf.front()->header.stamp.toSec() < plfeature_buf.front().first->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            plfeature_buf.pop();
            // continue;
            return measurements;
        }
        sensor_msgs::PointCloudConstPtr img_pointfeature_msg = plfeature_buf.front().first;
        plfeature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_pointfeature_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");

        std::pair<PL_Feature, sensor_msgs::PointCloud2ConstPtr> img_or_pc_msg;
        img_or_pc_msg.first.first = img_pointfeature_msg;
        img_or_pc_msg.second = nullptr;
        measurements.emplace_back(IMUs, img_or_pc_msg);
    }
    else
    {
        // ROS_INFO("lidar  in");
        // check if imu is later than the current lidar
        if (!(imu_buf.back()->header.stamp.toSec() > lidar_buf.front()->header.stamp.toSec()))
        {
            ROS_WARN("imu to lidar timestamp error, check if imu is later than the current lidar");
            return measurements;
        }

        // check if throw lidar
        if (!(imu_buf.front()->header.stamp.toSec() < lidar_buf.front()->header.stamp.toSec()))
        {
            ROS_WARN("throw lidar, only should happen at the beginning");
            lidar_buf.pop();
            // continue;
            return measurements;
        }

        sensor_msgs::PointCloud2ConstPtr lidar_msg = lidar_buf.front();
        lidar_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        // while (imu_buf.front()->header.stamp.toSec() < lidar_msg->header.stamp.toSec())
        // {
        //     IMUs.emplace_back(imu_buf.front());
        //     imu_buf.pop();
        // }
        
        // IMUs.emplace_back(imu_buf.front());
        // if (IMUs.empty())
        //     ROS_WARN("no imu between two image");

        std::pair<PL_Feature, sensor_msgs::PointCloud2ConstPtr> img_or_pc_msg;
        img_or_pc_msg.first.first = nullptr;
        img_or_pc_msg.first.second = nullptr;
        img_or_pc_msg.second = lidar_msg;
        measurements.emplace_back(IMUs, img_or_pc_msg);
    }

    // }
    // ROS_INFO("out");
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount,
                              estimator.tic[0], Eigen::Quaterniond(estimator.ric[0]), RECV_LIO_FAIL_FLAG);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    m_odom.lock();
    g_odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

// void sync_liostate(StatesGroup &lio_state,const std::deque<nav_msgs::Odometry> odomQueue)//sync_newest_liostate_from_imuf_lio_odom
void sync_liostate() // sync_newest_liostate_from_imuf_lio_odom
{
    if (!g_odomQueue.empty())
    {
        g_livo_state.pos_end << g_odomQueue.back().pose.pose.position.x, g_odomQueue.back().pose.pose.position.y, g_odomQueue.back().pose.pose.position.z;
        Eigen::Quaterniond eigenQuat(g_odomQueue.back().pose.pose.orientation.w, g_odomQueue.back().pose.pose.orientation.x, g_odomQueue.back().pose.pose.orientation.y, g_odomQueue.back().pose.pose.orientation.z);
        g_livo_state.rot_end = eigenQuat.toRotationMatrix();
        g_livo_state.vel_end << g_odomQueue.back().twist.twist.linear.x, g_odomQueue.back().twist.twist.linear.y, g_odomQueue.back().twist.twist.linear.z;

        g_livo_state.bias_g << g_odomQueue.back().pose.covariance[4], g_odomQueue.back().pose.covariance[5], g_odomQueue.back().pose.covariance[6];
        g_livo_state.bias_a << g_odomQueue.back().pose.covariance[1], g_odomQueue.back().pose.covariance[2], g_odomQueue.back().pose.covariance[3];
        g_livo_state.gravity << 0, 0, g_odomQueue.back().pose.covariance[7];
        // g_livo_state.cov = ;
    }
    else
    {
        ROS_DEBUG("g_odomQueue empty!");
        // ROS_WARN("g_odomQueue empty!");
    }
}
void plfeature_callback(const sensor_msgs::PointCloudConstPtr &point_feature_msg, const sensor_msgs::PointCloudConstPtr &line_feature_msg)
{
    m_buf.lock();
    auto plfeaturte = std::make_pair(point_feature_msg, line_feature_msg);
    plfeature_buf.push(plfeaturte);
    m_buf.unlock();
    con.notify_one();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &point_feature_msg)
{
    m_buf.lock();
    sensor_msgs::PointCloudConstPtr line_feature_msg = nullptr;
    auto plfeaturte = std::make_pair(point_feature_msg, line_feature_msg);
    plfeature_buf.push(plfeaturte);
    m_buf.unlock();
    con.notify_one();
}


void lidar_status_callback(const std_msgs::Header &lidar_status_msg)
{
    if (lidar_status_msg.frame_id == "lidar data failed")
    {
        RECV_LIO_FAIL_FLAG = 1;
    }
    else
    {
        RECV_LIO_FAIL_FLAG = 0;
    }
}

void laser_cloud_flat_cbk(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());

    // p_pre->process(msg,ptr);

    // m_lidar_buf.lock();
    // g_lidar_buffer.push_back(ptr);
    // g_lidar_time_buffer.push_back(msg->header.stamp.toSec());
    // m_lidar_buf.unlock();

    m_buf.lock();
    lidar_buf.push(msg);
    m_buf.unlock();
    con.notify_one();
}
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while (!plfeature_buf.empty())
            plfeature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        while (!lidar_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    while (ros::ok())
    {
        // std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::pair<sensor_msgs::PointCloudConstPtr,sensor_msgs::PointCloud2ConstPtr>>> measurements;
        // std::unique_lock<std::mutex> lk(m_buf);
        // con.wait(lk, [&]
        //          { return (measurements = getMeasurements()).size() != 0; });
        // lk.unlock();

        auto measurements = getMeasurements();
        if (measurements.size() == 0)
        {
            continue;
        }

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            if (measurement.second.first.first == nullptr)
                continue;
            auto img_pointfeature_msg = measurement.second.first.first;

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_pointfeature_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2.1 VINS point feature load update
            TicToc t_s;
            ROS_DEBUG("processing vision data with stamp %f \n", img_pointfeature_msg->header.stamp.toSec());
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> pointfeatures;
            for (unsigned int i = 0; i < img_pointfeature_msg->points.size(); i++)
            {
                int v = img_pointfeature_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_pointfeature_msg->points[i].x;
                double y = img_pointfeature_msg->points[i].y;
                double z = img_pointfeature_msg->points[i].z;
                double p_u = img_pointfeature_msg->channels[1].values[i];
                double p_v = img_pointfeature_msg->channels[2].values[i];
                double velocity_x = img_pointfeature_msg->channels[3].values[i];
                double velocity_y = img_pointfeature_msg->channels[4].values[i];
                double depth = img_pointfeature_msg->channels[5].values[i];

                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                pointfeatures[feature_id].emplace_back(camera_id, xyz_uv_velocity_depth);
            }


            // 3. sync liostate to add odomqueue
            sync_liostate();

            // 4. LVI-SAM added a initialization info from lidar to help VINS
            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            
            initialization_info = odomRegister->getOdometry(g_odomQueue, img_pointfeature_msg->header.stamp.toSec() + estimator.td);
            m_odom.unlock();

            // 5. process data (auto-adapted [(L)-I-V] )
            TicToc t_process_data;
            StatesGroup orig_lio_state = g_livo_state;
            g_livo_state = estimator.processData(g_livo_state, RECV_LIO_FAIL_FLAG, pointfeatures, initialization_info, img_pointfeature_msg->header);
            double whole_t = t_s.toc() , t_process = t_process_data.toc();
            // std::cout << ANSI_COLOR_CYAN_BOLD << "whole image handler time :  " << whole_t <<" ms" << "process time :  " << t_process <<" ms" << ANSI_COLOR_RESET << std::endl;

            // 6. Visualization
            std_msgs::Header header = img_pointfeature_msg->header;
            pub_VIO_Odometry(estimator, header);
            pub_LIO_Odometry(orig_lio_state, header, RECV_LIO_FAIL_FLAG);
            pub_fused_Odometry(estimator, g_livo_state, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    readParameters(n);
    estimator.setParameter();

    n.param<int>("/if_write_res_to_bag", if_write_res_to_bag, 0);
    n.param<std::string>("/bag_file_name", bag_file_name, "./");

    n.param<double>("preprocess/blind", p_pre->blind, 0.01);
    n.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    n.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    n.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    n.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);

    if (if_write_res_to_bag)
    {
        init_rosbag_for_recording();
    }

    registerPub(n);

#if IF_OFFICIAL
    odomRegister = new odometryRegister(n);
#else
    Eigen::Vector3d t_lidar_imu = -R_imu_lidar.transpose() * t_imu_lidar;
    odomRegister = new odometryRegister(n, R_imu_lidar.transpose(), t_lidar_imu);
#endif

    registerPub(n);
    chatter_pub = n.advertise<std_msgs::Float32MultiArray>("/Intrinsic", 1000);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 5000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_odom = n.subscribe("/odometry/imu", 5000, odom_callback);
    // message_filters::Subscriber<sensor_msgs::PointCloud> point_feature_sub(n, PROJECT_NAME + "/vins/feature/feature", 1000, ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<sensor_msgs::PointCloud> line_feature_sub(n, PROJECT_NAME + "/vins/feature/linefeature", 1000, ros::TransportHints().tcpNoDelay());
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_feature_sub, line_feature_sub);
    // sync.registerCallback(boost::bind(&plfeature_callback, _1, _2));
    ros::Subscriber sub_point_feature = n.subscribe( PROJECT_NAME + "/vins/feature/feature", 1000, feature_callback);
    ros::Subscriber sub_lio_failed_flag = n.subscribe(PROJECT_NAME + "/lidar/status", 100, lidar_status_callback);
    ros::Subscriber sub_laser_cloud_flat = n.subscribe(LIDAR_TOPIC, 100, laser_cloud_flat_cbk, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 100, restart_callback);
    if (!USE_LIDAR)
        sub_odom.shutdown();

    std::thread measurement_process{process};

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}