#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_lio_odometry ,pub_fused_odometry, pub_latest_odometry, pub_latest_odometry_ros;
ros::Publisher pub_path,pub_fused_path;
ros::Publisher pub_point_cloud, pub_margin_cloud , pub_lines, pub_marg_lines;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;
nav_msgs::Path fused_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

rosbag::Bag record_evluate_bag;
std::string bag_file_name;
int if_write_res_to_bag;

void init_rosbag_for_recording()
{
    std::cout << ANSI_COLOR_GREEN_BG << "Record result to " << bag_file_name << ANSI_COLOR_RESET << std::endl;
    record_evluate_bag.open(bag_file_name.c_str(), rosbag::bagmode::Write);
}

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate", 1000);
    
    pub_latest_odometry_ros = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 1000);
    pub_path = n.advertise<nav_msgs::Path>(PROJECT_NAME + "/vins/odometry/vio_path", 1000);
    pub_fused_path = n.advertise<nav_msgs::Path>(PROJECT_NAME + "/fused_path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/vio_odometry", 1000);
    pub_lio_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/lio/lio_odometry", 1000);
    pub_fused_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/fused_odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/history_cloud", 1000);
    pub_lines = n.advertise<visualization_msgs::Marker>("lines_cloud", 1000);
    pub_marg_lines = n.advertise<visualization_msgs::Marker>("history_lines_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>(PROJECT_NAME + "/vins/odometry/key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/vins/odometry/camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/extrinsic", 1000);

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

tf::Transform transformConversion(const tf::StampedTransform &t)
{
    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = t.getOrigin().x();
    yCur = t.getOrigin().y();
    zCur = t.getOrigin().z();
    tf::Matrix3x3 m(t.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    return tf::Transform(tf::createQuaternionFromRPY(rollCur, pitchCur, yawCur), tf::Vector3(xCur, yCur, zCur));
    ;
}


void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
    const Eigen::Vector3d &V, const std_msgs::Header &header, const int &failureId,
    const Eigen::Vector3d &t_ic, const Eigen::Quaterniond &q_ic, bool recv_lio_fail_flag)
{
    static tf::TransformBroadcaster br;
    static tf::TransformListener listener;
    static double last_align_time = -1;

    // Quternion not normalized
    if (Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z() + Q.w() * Q.w() < 0.99)
        return;


    // imu odometry in camera frame
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);


#if IF_OFFICIAL
    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    tf::Quaternion q_odom_cam(Q.x(), Q.y(), Q.z(), Q.w());
    tf::Quaternion q_cam_to_lidar(0, 1, 0, 0); // mark: camera - lidar
    tf::Quaternion q_odom_ros = q_odom_cam * q_cam_to_lidar;
    tf::quaternionTFToMsg(q_odom_ros, odometry.pose.pose.orientation);
    pub_latest_odometry_ros.publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    tf::Transform t_w_body = tf::Transform(q_odom_ros, tf::Vector3(P.x(), P.y(), P.z()));
    tf::StampedTransform trans_world_vinsbody_ros = tf::StampedTransform(
        t_w_body, header.stamp, "vins_world", "vins_body_ros");
    br.sendTransform(trans_world_vinsbody_ros);
#else
    
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    //; R_odom_imu
    tf::Quaternion q_odom_imu(Q.x(), Q.y(), Q.z(), Q.w());   
    Eigen::Quaterniond q_imu_lidar(R_imu_lidar);
    //; R_imu_lidar
    tf::Quaternion q_imu_lidar_tf(q_imu_lidar.x(), q_imu_lidar.y(), q_imu_lidar.z(), q_imu_lidar.w());   
    //; R_odom_lidar = R_odom_imu * R_imu_lidar
    tf::Quaternion q_odom_lidar = q_odom_imu * q_imu_lidar_tf;
    //; t_dodom_lidar = R_odom_imu * t_imu_lidar + t_odom_imu
    Eigen::Vector3d t_odom_lidar = Q * t_imu_lidar + P;  
    odometry.pose.pose.position.x = t_odom_lidar.x();
    odometry.pose.pose.position.y = t_odom_lidar.y();
    odometry.pose.pose.position.z = t_odom_lidar.z();
    tf::quaternionTFToMsg(q_odom_lidar, odometry.pose.pose.orientation);
    pub_latest_odometry_ros.publish(odometry);

    
    tf::Transform t_w_body = tf::Transform(q_odom_imu, tf::Vector3(P.x(), P.y(), P.z()));
    tf::StampedTransform trans_world_vinsBody = tf::StampedTransform(
        t_w_body, header.stamp, "vins_world", "vins_body_imuhz");
    br.sendTransform(trans_world_vinsBody);

    
    
    tf::Transform transform;
    tf::Quaternion q;
    
    
    //; R_imu_cFLU = R_imu_cam * R_cam_camFLU
    Eigen::Quaterniond q_i_cFLU = q_ic * Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5);
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setW(q_i_cFLU.w());
    q.setX(q_i_cFLU.x());
    q.setY(q_i_cFLU.y());
    q.setZ(q_i_cFLU.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_body_imuhz", "vins_cameraFLU"));
#endif

    
    if (1)
    {
        
        static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        // if (header.stamp.toSec() - last_align_time > 0.01)
        // {
        try
        {
            
            
            
            

            // if(!recv_lio_fail_flag)
            if(1)
            {
                tf::StampedTransform T_odom_lidar;
                listener.lookupTransform("camera_init", "aft_mapped", ros::Time(0), T_odom_lidar);
                tf::Transform t_w_lidar = tf::Transform(q_odom_lidar, tf::Vector3(t_odom_lidar.x(), t_odom_lidar.y(), t_odom_lidar.z()));
                tf::StampedTransform T_vinsworld_lidar = tf::StampedTransform(
                    t_w_lidar, header.stamp, "vinsworld", "lidar");
                //; T_odom_vinsworld = T_odom_lidar * T_vinsworld_lidar.inverse()
                t_odom_world = transformConversion(T_odom_lidar) * 
                        transformConversion(T_vinsworld_lidar).inverse();
                // last_align_time = header.stamp.toSec();
            }
        }
        catch (tf::TransformException ex)
        {
        }
        // }
        br.sendTransform(tf::StampedTransform(
            t_odom_world, header.stamp, "camera_init", "vins_world"));
    }
    else
    {
        
        static tf::Transform t_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        br.sendTransform(tf::StampedTransform(t_static, header.stamp, "camera_init", "vins_world"));
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pub_VIO_Odometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        if (if_write_res_to_bag)
        {
            record_evluate_bag.write(pub_odometry.getTopic(), ros::Time::now(), odometry);
        }

        static double path_save_time = -1;
        if (header.stamp.toSec() - path_save_time > 0.02)
        {
            path_save_time = header.stamp.toSec();
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            pub_path.publish(path);
        }
    }
}

void pub_LIO_Odometry(const StatesGroup & state, const std_msgs::Header &header, int fail_flag = 0 )
{
    if(!fail_flag)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "camera_init";
        odometry.child_frame_id = "camera_init";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(state.rot_end);
        odometry.pose.pose.position.x = state.pos_end(0);
        odometry.pose.pose.position.y = state.pos_end(1);
        odometry.pose.pose.position.z = state.pos_end(2);
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x =  state.vel_end(0);
        odometry.twist.twist.linear.y =  state.vel_end(1);
        odometry.twist.twist.linear.z =  state.vel_end(2);
        pub_lio_odometry.publish(odometry);
    
        if (if_write_res_to_bag)
        {
            record_evluate_bag.write(pub_lio_odometry.getTopic(), ros::Time::now(), odometry);
        }
    }
    else
    {

    }
}


void pub_fused_Odometry(const Estimator &estimator, const StatesGroup & state, const std_msgs::Header &header)
{

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "camera_init";
        odometry.child_frame_id = "camera_init";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(state.rot_end);
        odometry.pose.pose.position.x = state.pos_end(0);
        odometry.pose.pose.position.y = state.pos_end(1);
        odometry.pose.pose.position.z = state.pos_end(2);
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x =  state.vel_end(0);
        odometry.twist.twist.linear.y =  state.vel_end(1);
        odometry.twist.twist.linear.z =  state.vel_end(2);
        pub_fused_odometry.publish(odometry);
    

        if (if_write_res_to_bag)
        {
            record_evluate_bag.write(pub_fused_odometry.getTopic(), ros::Time::now(), odometry);
        }
        
        static double path_save_time = -1;

        geometry_msgs::PoseStamped pose_stamped;
        if (header.stamp.toSec() - path_save_time > 0.02)
        {
            path_save_time = header.stamp.toSec();
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "camera_init";
            pose_stamped.pose = odometry.pose.pose;
            fused_path.header = header;
            fused_path.header.frame_id = "camera_init";
            fused_path.poses.push_back(pose_stamped);
            pub_fused_path.publish(fused_path);
        }
        // if (if_write_res_to_bag)
        // {
        //     record_evluate_bag.write(pub_path.getTopic(), ros::Time::now(), path);
        // }

    }
}



void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_key_poses.getNumSubscribers() == 0)
        return;

    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_camera_pose_visual.getNumSubscribers() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_point_cloud.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);

            if (it_per_id.lidar_depth_flag == false)
                intensity_channel.values.push_back(0);
            else
                intensity_channel.values.push_back(1);
        }

        point_cloud.channels.push_back(intensity_channel);
        pub_point_cloud.publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                margin_cloud.points.push_back(p);

                if (it_per_id.lidar_depth_flag == false)
                    intensity_channel.values.push_back(0);
                else
                    intensity_channel.values.push_back(1);
            }
        }

        margin_cloud.channels.push_back(intensity_channel);
        pub_margin_cloud.publish(margin_cloud);
    }
}
visualization_msgs::Marker marg_lines_cloud;  
std::list<visualization_msgs::Marker> marg_lines_cloud_last10frame;
void pubLinesCloud(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.header.frame_id = "camera_init";
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines.id = 0; //key_poses_id++;
    lines.scale.x = 0.03;
    lines.scale.y = 0.03;
    lines.scale.z = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;

    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//
//        if (!(used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.is_triangulation == false)
            continue;

        //std::cout<< "used num: " <<used_num<<" line id: "<<it_per_id.feature_id<<std::endl;

        int imu_i = it_per_id.start_frame;

        Vector3d pc, nc, vc;
        // pc = it_per_id.line_plucker.head(3);
        // nc = pc.cross(vc);
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   
        Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
        Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

//
//        if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//            continue;

        //std::cout <<"visual: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n\n";
        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
        Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;


/*
        Vector3d diff_1 = it_per_id.ptw1 - w_pts_1;
        Vector3d diff_2 = it_per_id.ptw2 - w_pts_2;
        if(diff_1.norm() > 1 || diff_2.norm() > 1)
        {
            std::cout <<"visual: "<<it_per_id.removed_cnt<<" "<<it_per_id.all_obs_cnt<<" " << it_per_id.feature_id <<"\n";// << it_per_id.line_plucker <<"\n\n" << it_per_id.line_plk_init <<"\n\n";
            std::cout << it_per_id.Rj_ <<"\n" << it_per_id.tj_ <<"\n\n";
            std::cout << estimator.Rs[imu_i] <<"\n" << estimator.Ps[imu_i] <<"\n\n";
            std::cout << obs <<"\n\n" << it_per_id.obs_j<<"\n\n";

        }

        w_pts_1 = it_per_id.ptw1;
        w_pts_2 = it_per_id.ptw2;
*/
/*
        Vector3d w_pts_1 =  estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + estimator.Ps[imu_i];
        Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + estimator.Ps[imu_i];

        Vector3d d = w_pts_1 - w_pts_2;
        if(d.norm() > 4.0 || d.norm() < 2.0)
            continue;
*/
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines.points.push_back(p);

    }
    //std::cout<<" viewer lines.size: " <<lines.points.size() << std::endl;
    pub_lines.publish(lines);



//    marg_lines_cloud_oneframe.header = header;
//    marg_lines_cloud_oneframe.header.frame_id = "camera_init";
//    marg_lines_cloud_oneframe.ns = "lines";
//    marg_lines_cloud_oneframe.type = visualization_msgs::Marker::LINE_LIST;
//    marg_lines_cloud_oneframe.action = visualization_msgs::Marker::ADD;
//    marg_lines_cloud_oneframe.pose.orientation.w = 1.0;
//    marg_lines_cloud_oneframe.lifetime = ros::Duration();
//
//    //marg_lines_cloud.id = 0; //key_poses_id++;
//    marg_lines_cloud_oneframe.scale.x = 0.05;
//    marg_lines_cloud_oneframe.scale.y = 0.05;
//    marg_lines_cloud_oneframe.scale.z = 0.05;
//    marg_lines_cloud_oneframe.color.g = 1.0;
//    marg_lines_cloud_oneframe.color.a = 1.0;

//////////////////////////////////////////////
    // all marglization line
    marg_lines_cloud.header = header;
    marg_lines_cloud.header.frame_id = "camera_init";
    marg_lines_cloud.ns = "lines";
    marg_lines_cloud.type = visualization_msgs::Marker::LINE_LIST;
    marg_lines_cloud.action = visualization_msgs::Marker::ADD;
    marg_lines_cloud.pose.orientation.w = 1.0;
    marg_lines_cloud.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    //marg_lines_cloud.id = 0; //key_poses_id++;
    marg_lines_cloud.scale.x = 0.05;
    marg_lines_cloud.scale.y = 0.05;
    marg_lines_cloud.scale.z = 0.05;
    marg_lines_cloud.color.r = 1.0;
    marg_lines_cloud.color.a = 1.0;
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
//        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.linefeature_per_frame.size() <= 2
            && it_per_id.is_triangulation == true )

        //if (it_per_id.is_triangulation == true )
        {
            int imu_i = it_per_id.start_frame;

            Vector3d pc, nc, vc;
            // pc = it_per_id.line_plucker.head(3);
            // nc = pc.cross(vc);
            nc = it_per_id.line_plucker.head(3);
            vc = it_per_id.line_plucker.tail(3);
            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   
            Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
            Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
            Vector2d ln = ( p11.cross(p21) ).head(2);     
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d( 0, 0, 0 );

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

//            if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//                continue;
//
            double length = (e1-e2).norm();
            if(length > 10) continue;

            //std::cout << e1 <<"\n\n";
            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
            Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

            //w_pts_1 = it_per_id.ptw1;
            //w_pts_2 = it_per_id.ptw2;

            geometry_msgs::Point p;
            p.x = w_pts_1(0);
            p.y = w_pts_1(1);
            p.z = w_pts_1(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
            p.x = w_pts_2(0);
            p.y = w_pts_2(1);
            p.z = w_pts_2(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
        }
    }
//    if(marg_lines_cloud_oneframe.points.size() > 0)
//        marg_lines_cloud_last10frame.push_back(marg_lines_cloud_oneframe);
//
//    if(marg_lines_cloud_last10frame.size() > 50)
//        marg_lines_cloud_last10frame.pop_front();
//
//    marg_lines_cloud.points.clear();
//    list<visualization_msgs::Marker>::iterator itor;
//    itor = marg_lines_cloud_last10frame.begin();
//    while(itor != marg_lines_cloud_last10frame.end())
//    {
//        for (int i = 0; i < itor->points.size(); ++i) {
//            marg_lines_cloud.points.push_back(itor->points.at(i));
//        }
//        itor++;
//    }

//    ofstream foutC("/home/hyj/catkin_ws/src/VINS-Mono/config/euroc/landmark.txt");
//    for (int i = 0; i < marg_lines_cloud.points.size();) {
//
//        geometry_msgs::Point pt1 = marg_lines_cloud.points.at(i);
//        geometry_msgs::Point pt2 = marg_lines_cloud.points.at(i+1);
//        i = i + 2;
//        foutC << pt1.x << " "
//              << pt1.y << " "
//              << pt1.z << " "
//              << pt2.x << " "
//              << pt2.y << " "
//              << pt2.z << "\n";
//    }
//    foutC.close();
    pub_marg_lines.publish(marg_lines_cloud);

}



/**
 * @brief 发布估计的TF坐标变换，这个非常重要
 * 
 * @param[in] estimator 
 * @param[in] header 
 */
void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_world", "vins_body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_body", "vins_camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    if (pub_keyframe_pose.getNumSubscribers() == 0 && pub_keyframe_point.getNumSubscribers() == 0)
        return;

    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_keyframe_pose.publish(odometry);

        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point.publish(point_cloud);
    }
}