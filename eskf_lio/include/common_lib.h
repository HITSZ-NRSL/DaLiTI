#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eskf_lio/States.h>
#include <eskf_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include "my_utility.h"

// #define DEBUG_PRINT
using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.8099)    // Gravaty const in GuangDong/China
#define DIM_OF_STATES (24) // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_OF_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (1)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) std::vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))

typedef eskf_lio::Pose6D Pose6D;
// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
Eigen::Vector3d Zero3d(0, 0, 0);
Eigen::Vector3f Zero3f(0, 0, 0);
// Eigen::Vector3d Lidar_offset_to_IMU(0.05512, 0.02226, 0.0297); // Horizon
Eigen::Vector3d Lidar_offset_to_IMU(0.0, 0.0, -0.0); // Avia

struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double observation_end_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct StatesGroup //  r p R_LI T_LI v bg ba g
{
    StatesGroup()
    {
        this->rot_end = Eigen::Matrix3d::Identity();
        this->pos_end = Zero3d;
        this->R_L_I = Eigen::Matrix3d::Identity();
        this->T_L_I = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g = Zero3d;
        this->bias_a = Zero3d;
        this->gravity = Zero3d;
        this->cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
    };

    StatesGroup(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->R_L_I = b.R_L_I;
        this->T_L_I = b.T_L_I;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
    };

    StatesGroup &operator=(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->R_L_I = b.R_L_I;
        this->T_L_I = b.T_L_I;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
        return *this;
    };

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.R_L_I = this->R_L_I * Exp(state_add(6, 0), state_add(7, 0), state_add(8, 0));
        a.T_L_I = this->T_L_I + state_add.block<3, 1>(9, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(12, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(15, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(18, 0);
        a.gravity = this->gravity + state_add.block<3, 1>(21, 0);
        a.cov = this->cov;
        return a;
    };

    StatesGroup operator+(const StatesGroup &state_2)
    {
        StatesGroup ret;
        ret.rot_end = this->rot_end * state_2.rot_end;
        ret.pos_end = this->pos_end + state_2.pos_end;
        ret.R_L_I = this->R_L_I * state_2.R_L_I;
        ret.T_L_I = this->T_L_I + state_2.T_L_I;
        ret.vel_end = this->vel_end + state_2.vel_end;
        ret.bias_g = this->bias_g;
        ret.bias_a = this->bias_a;
        ret.gravity = this->gravity;
        ret.cov = this->cov;
        return ret;
    };
    //end add

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->R_L_I = this->R_L_I * Exp(state_add(6, 0), state_add(7, 0), state_add(8, 0));
        this->T_L_I += state_add.block<3, 1>(9, 0);
        this->vel_end += state_add.block<3, 1>(12, 0);
        this->bias_g += state_add.block<3, 1>(15, 0);
        this->bias_a += state_add.block<3, 1>(18, 0);
        this->gravity += state_add.block<3, 1>(21, 0);
        return *this;
    };

    StatesGroup &operator+=(const StatesGroup &state_add)
    {
        this->rot_end = this->rot_end * state_add.rot_end.transpose();
        this->pos_end += state_add.pos_end;
        this->R_L_I = this->R_L_I * state_add.R_L_I;
        this->T_L_I +=state_add.T_L_I;
        this->vel_end += state_add.vel_end;
        this->bias_g = state_add.bias_g;
        this->bias_a = state_add.bias_a;
        this->gravity = state_add.gravity;
        return *this;
    };
    // end add

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        Eigen::Matrix3d rotLI(b.R_L_I.transpose() * this->R_L_I);
        a.block<3, 1>(0, 0) = Log(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = Log(rotLI);
        a.block<3, 1>(9, 0) = this->T_L_I - b.T_L_I;
        a.block<3, 1>(12, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(15, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(18, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(21, 0) = this->gravity - b.gravity;
        return a;
    };

    
    StatesGroup operator*(const double &scale)
    {
        StatesGroup a;
        
        Eigen::Vector3d so3 = Log(this->rot_end);      
        a.rot_end = Exp(so3(0) * scale, so3(1) * scale, so3(2) * scale);                  
        a.pos_end = this->pos_end * scale;
        a.R_L_I = this->R_L_I;
        a.T_L_I = this->T_L_I * scale;
        a.vel_end = this->vel_end * scale;
        a.bias_g = this->bias_g * scale;
        a.bias_a = this->bias_a * scale;
        a.gravity = this->gravity * scale;
        a.cov = this->cov;
        return a;
    };
    

    void display(std::string str = std::string("State: "))
    {
        Eigen::Matrix< double, 3, 1, Eigen::DontAlign >  angle_axis = Log(this->rot_end) * 57.3;
        printf("%s |", str.c_str());
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", this->pos_end(0), this->pos_end(1), this->pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", this->vel_end(0), this->vel_end(1), this->vel_end(2));
        printf("(%.4f, %.4f, %.4f) | ", this->bias_g(0), this->bias_g(1), this->bias_g(2));
        printf("(%.4f, %.4f, %.4f) \n", this->bias_a(0), this->bias_a(1), this->bias_a(2));
    }

    Eigen::Matrix3d rot_end;                                 // the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // the estimated position at the end lidar point (world frame)
    Eigen::Matrix3d R_L_I;                                   // Rotation from Lidar frame L to IMU frame I
    Eigen::Vector3d T_L_I;                                   // Translation from Lidar frame L to IMU frame I
    Eigen::Vector3d vel_end;                                 // the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // gyroscope bias
    Eigen::Vector3d bias_a;                                  // accelerator bias
    Eigen::Vector3d gravity;                                 // the estimated gravity acceleration
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
};

template <typename T>
T rad2deg(T radians)
{
    return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees)
{
    return degrees * PI_M / 180.0;
}

float calc_dist(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

template <typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

#endif
