#include "parameters.h"

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
std::string LIDAR_TOPIC;
double ROW, COL;
double TD, TR;

int USE_LIDAR;


#if IF_OFFICIAL

#else

Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); 
Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();
#endif

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    fsSettings["imu_topic"] >> IMU_TOPIC;

    fsSettings["lidar_topic"] >> LIDAR_TOPIC;

    fsSettings["use_lidar"] >> USE_LIDAR;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("Image dimention: ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_INFO(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_INFO(" Fix extrinsic param.");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl
                                         << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl
                                         << TIC[0].transpose());
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    fsSettings.release();

#if IF_OFFICIAL

#else
    
    std::vector<double> t_imu_lidar_V;
    std::vector<double> R_imu_lidar_V;
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicTranslation", t_imu_lidar_V, std::vector<double>());
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicRotation", R_imu_lidar_V, std::vector<double>());
    t_imu_lidar = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(t_imu_lidar_V.data(), 3, 1);
    Eigen::Matrix3d R_tmp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R_imu_lidar_V.data(), 3, 3);
    ROS_ASSERT(abs(R_tmp.determinant()) > 0.9);   
    R_imu_lidar = Eigen::Quaterniond(R_tmp).normalized().toRotationMatrix();
    
    ROS_WARN_STREAM("vins-estimator have read params");
    // ROS_WARN_STREAM("=vins-estimator read R_imu_lidar : =====================");
    // std::cout << R_imu_lidar << std::endl;
    // ROS_WARN_STREAM("=vins-estimator read t_imu_lidar : =====================");
    // std::cout << t_imu_lidar.transpose() << std::endl;
#endif

    usleep(100);
}
