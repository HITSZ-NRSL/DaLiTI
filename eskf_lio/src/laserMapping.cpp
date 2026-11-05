// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <algorithm>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <common_lib.h>

#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

#include <eskf_lio/States.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include "tool_color_printf.hpp"
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>


/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
//?/ Global Parameters
#define INIT_TIME (0)
#define LASER_POINT_COV (0.0015)
#define NUM_MATCH_POINTS (5)

std::string root_dir = ROOT_DIR;

int iterCount = 0;
int NUM_MAX_ITERATIONS = 0;

int effct_feat_num = 0;

double filter_size_surf_min;

double cube_len = 0.0;
double observation_end_time = 0.0;
double last_timestamp_lidar = -1;
double last_timestamp_imu = -1;

double res_mean_last = 0.05;
double total_distance = 0.0;
auto position_last = Zero3d;

int dynamic_effect_featurepoints_threshold = 100;
long lidar_frame_counter_num=0;
int featptsThreshold = 300;

int recv_n;

double zeta_l = 0.0;
double zeta_t = 0.0;
double beta = 0.05;
double Nla = 65536;
double Mta = 307200;
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/



/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
//?/ global flags
bool lidar_pushed = false;
bool flg_exit = false;
bool flg_reset = false;
bool flg_first_scan = true;
bool dense_map_en, flg_EKF_inited = false, flg_EKF_converged = false, extrinsic_est_en = true, EKF_stop_flg = 0;
bool RECV_LIO_FAIL_FLAG = true;

/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/// IMU relative variables
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
/// Buffers for measurements
std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/




/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/

// surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI()); 
PointCloudXYZI::Ptr feats_down(new PointCloudXYZI()); 
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI()); 
PointCloudXYZI::Ptr coeffSel(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf; 

// all points
PointCloudXYZI::Ptr laserCloudFullRes2(new PointCloudXYZI());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZI>());
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/



KD_TREE<PointType> ikdtree;

Eigen::Vector3f XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
Eigen::Vector3f XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
V3D pos_lid;

/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
//?/ estimator inputs and output
MeasureGroup Measures;
StatesGroup state;
// Posterior of the previous frame (anchor of the VINS motion measurement).
StatesGroup last_post_state;
double last_post_time = -1.0;
bool have_last_post = false;
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
//?/ Thermal-inertial odometry (TIS / VINS) used as a motion measurement while the LiDAR is degraded.
// The VINS IMU-propagated odometry is buffered with its timestamps so that its motion can be taken
// exactly between two LiDAR frame ends, expressed in the VINS body frame (independent of the drift
// of the VINS world frame).
struct VinsSample
{
    double t;
    Eigen::Vector3d p;    // IMU position in vins_world
    Eigen::Quaterniond q; // IMU orientation in vins_world
    Eigen::Vector3d v;    // IMU velocity in vins_world
};
std::deque<VinsSample> g_vins_buf;
int g_tis_failure_id = -1;
bool tis_online = false;

bool vio_fusion_enable = true;
double vio_fusion_max_zeta_l = 0.5; // fuse when the last normalized LiDAR factor is below this, or the LiDAR update is stopped
bool fusion_debug_log = false;      // per-frame fusion diagnostics (one line per LiDAR frame)
double vio_min_weight = 0.1;        // floor of the VINS weight
double vio_chi2_gate = 22.46;       // chi2 99.9 %: reject VINS motion inconsistent with the filter (0 = off)
// Non-holonomic constraint for ground robots (used only while the LiDAR is degraded): the body-frame
// lateral velocity is lever * yaw_rate and the vertical velocity is zero; VINS then only supplies the
// forward motion. Off by default (handheld / aerial platforms violate it).
bool nhc_enable = false;
double nhc_lever = 0.12;            // [m] lateral velocity = nhc_lever * body yaw rate (IMU ahead of the turn centre)
double nhc_sigma_lat = 0.08;        // [m/s]
double nhc_sigma_vert = 0.05;       // [m/s]
double nhc_vio_lat_scale = 5.0;     // VINS lateral / vertical noise = this x forward noise when the NHC is on
// Gravity-referenced roll/pitch correction while the LiDAR is degraded (complementary filter on the specific force,
// with the filter's own acceleration removed). Without it, roll/pitch drift with the gyro bias (3-4 deg in 140 s of
// smoke), the forward motion leaks into z and the tilt is locked into the map after the smoke.
bool tilt_correction_enable = true;
double tilt_tau = 10.0;             // [s] time constant of the correction
double tilt_max_dyn = 2.0;          // [m/s^2] skip frames whose specific force deviates more than this from g
double tilt_max_acc = 0.6;          // [m/s^2] skip frames with a larger filter acceleration (manoeuvres, update jumps)
double vio_pos_sigma = 0.03;        // [m] noise of the per-frame VINS displacement
double vio_vel_sigma = 0.10;        // [m/s] noise of the VINS body-frame velocity
double vio_max_speed = 3.0;         // [m/s] gate on the VINS displacement / velocity
double vio_wait_max = 0.03;         // [s] max wall time to wait for VINS samples that cover the frame end
bool lidar_weight_by_zeta = true;   // inflate the LiDAR point noise by 1 / zeta_l_norm
double lidar_min_weight = 0.05;
double prev_zeta_l_norm = 1.0, prev_zeta_t_norm = 0.0;

const int QUEUE_SIZE = 10;
std::queue<int> effct_feat_numQueue;
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
// start of the functions
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------*/


void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

// project lidar frame to world
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

vector<BoxPointType> cub_needrm;
int kdtree_delete_counter = 0;
double kdtree_delete_time = 0.0;

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();

    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

/*-------------------------------------------------------------------------------------------------------------------*/
//?/ call back funcs
long lidar_cnt = 0;
void feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    lidar_frame_counter_num++;
    if(lidar_frame_counter_num>100)
    {
        dynamic_effect_featurepoints_threshold = featptsThreshold;
    }
    mtx_buffer.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    // if (msg->data.size() > 1e5)
    // {
        lidar_buffer.push_back(msg);
        last_timestamp_lidar = msg->header.stamp.toSec();
    // }
    std::cout << ANSI_COLOR_GREEN << "get lidar: " << last_timestamp_lidar << ",lidar size: " << msg->data.size() << ANSI_COLOR_RESET << std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_cnt ++;
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // std::cout << "got imu: " << timestamp << " imu size " << imu_buffer.size() << std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void tis_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
    tis_online = true;
    const int failure_id = static_cast<int>(std::llround(msg_in->pose.covariance[0]));
    if (failure_id != g_tis_failure_id)
    {
        if (g_tis_failure_id >= 0)
            ROS_WARN("VINS reset detected (%d -> %d); clearing the VINS motion buffer.", g_tis_failure_id, failure_id);
        g_tis_failure_id = failure_id;
        g_vins_buf.clear();
    }
    VinsSample s;
    s.t = msg_in->header.stamp.toSec();
    const auto &pp = msg_in->pose.pose.position;
    const auto &qq = msg_in->pose.pose.orientation;
    const auto &vv = msg_in->twist.twist.linear;
    s.p << pp.x, pp.y, pp.z;
    s.q = Eigen::Quaterniond(qq.w, qq.x, qq.y, qq.z);
    s.v << vv.x, vv.y, vv.z;
    const double qn = s.q.norm();
    if (!s.p.allFinite() || !s.v.allFinite() || !std::isfinite(qn) || qn < 0.5)
        return;
    s.q.normalize();
    if (!g_vins_buf.empty() && s.t <= g_vins_buf.back().t)
    {
        if (s.t < g_vins_buf.back().t - 1.0)
            g_vins_buf.clear(); // time jumped backwards (bag restarted)
        else
            return;             // duplicate / out of order
    }
    g_vins_buf.push_back(s);
    while (!g_vins_buf.empty() && g_vins_buf.front().t < s.t - 5.0)
        g_vins_buf.pop_front();
}

// VINS state at time t (linear interpolation / slerp). False if t is not covered by the buffer.
bool vinsAt(const double t, VinsSample &out)
{
    if (g_vins_buf.size() < 2 || t < g_vins_buf.front().t || t > g_vins_buf.back().t)
        return false;
    auto it = std::lower_bound(g_vins_buf.begin(), g_vins_buf.end(), t,
                               [](const VinsSample &a, const double tt) { return a.t < tt; });
    if (it == g_vins_buf.begin())
    {
        out = *it;
        return true;
    }
    const VinsSample &b = *it;
    const VinsSample &a = *(it - 1);
    const double span = b.t - a.t;
    if (span > 0.1)
        return false; // gap in the VINS stream
    const double r = span > 1e-9 ? (t - a.t) / span : 0.0;
    out.t = t;
    out.p = a.p + r * (b.p - a.p);
    out.v = a.v + r * (b.v - a.v);
    out.q = a.q.slerp(r, b.q);
    return true;
}

// EKF update with the thermal-inertial (VINS) motion between the previous frame end t0 and this
// frame end t1. Measurements: position = previous posterior + previous attitude * VINS displacement
// (taken in the VINS body frame at t0), and velocity = attitude * VINS body-frame velocity at t1.
// Both are invariant to the drift of the VINS world frame. Only position and velocity are corrected:
// the attitude keeps coming from the gyro, which drifts far less than the thermal VIO in smoke.
// Masked EKF update: only the rows listed in `rows` of the state are corrected (Joseph form).
template <int M>
bool maskedUpdate(StatesGroup &st, const Eigen::Matrix<double, M, DIM_OF_STATES> &H, const Eigen::Matrix<double, M, 1> &y,
                  const Eigen::Matrix<double, M, M> &R, const std::vector<std::pair<int, int>> &rows, double *d2_out = nullptr)
{
    const Eigen::Matrix<double, M, M> S = H * st.cov * H.transpose() + R;
    const Eigen::Matrix<double, M, M> S_inv = S.inverse();
    if (d2_out)
        *d2_out = y.dot(S_inv * y);
    Eigen::Matrix<double, DIM_OF_STATES, M> K = st.cov * H.transpose() * S_inv;
    Eigen::Matrix<double, DIM_OF_STATES, M> Km = Eigen::Matrix<double, DIM_OF_STATES, M>::Zero();
    for (const auto &r : rows)
        Km.middleRows(r.first, r.second) = K.middleRows(r.first, r.second);
    const Eigen::Matrix<double, DIM_OF_STATES, 1> dx = Km * y;
    if (!dx.allFinite())
        return false;
    st += dx;
    const Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> IKH =
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() - Km * H;
    st.cov = IKH * st.cov * IKH.transpose() + Km * R * Km.transpose();
    return true;
}

// Non-holonomic constraint (ground robot): body lateral velocity = lever * yaw rate, vertical velocity = 0.
bool applyNhc(StatesGroup &st, const double yaw_rate)
{
    const M3D Rt = st.rot_end.transpose();
    Eigen::Matrix<double, 2, DIM_OF_STATES> H = Eigen::Matrix<double, 2, DIM_OF_STATES>::Zero();
    H.block<1, 3>(0, 12) = Rt.row(1);
    H.block<1, 3>(1, 12) = Rt.row(2);
    const V3D vb = Rt * st.vel_end;
    Eigen::Matrix<double, 2, 1> y(nhc_lever * yaw_rate - vb(1), -vb(2));
    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
    R(0, 0) = nhc_sigma_lat * nhc_sigma_lat;
    R(1, 1) = nhc_sigma_vert * nhc_sigma_vert;
    return maskedUpdate<2>(st, H, y, R, {{12, 3}});
}

// Complementary roll/pitch correction: rotate the attitude so that the measured specific force (world frame,
// minus the filter acceleration) moves toward +z by a fraction dt / tau of the error. Yaw is untouched.
bool tiltCorrection(StatesGroup &st, const MeasureGroup &meas, const double acc_scale, const V3D &acc_world, const double dt)
{
    if (meas.imu.empty() || dt <= 0)
        return false;
    V3D f_b(Zero3d);
    for (const auto &m : meas.imu)
        f_b += V3D(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    f_b = f_b / double(meas.imu.size()) * acc_scale - st.bias_a;
    if (acc_world.norm() > tilt_max_acc)
        return false; // a horizontal velocity jump barely changes |g_up| but tilts its direction
    const V3D g_up = st.rot_end * f_b - acc_world; // should be (0, 0, +g)
    if (!g_up.allFinite() || std::fabs(g_up.norm() - G_m_s2) > tilt_max_dyn)
        return false;
    const V3D u = g_up.normalized(), ez(0, 0, 1);
    const V3D axis = u.cross(ez);
    const double s_ang = axis.norm();
    if (s_ang < 1e-6)
        return true;
    const double ang = std::atan2(s_ang, u.dot(ez)) * std::min(1.0, dt / tilt_tau);
    st.rot_end = Eigen::AngleAxisd(ang, axis / s_ang).toRotationMatrix() * st.rot_end;
    return true;
}

// returns 1 = fused, -1 = VINS samples do not cover [t0, t1], -2 = rejected by the speed gate, -3 = numerical failure,
// -4 = rejected by the chi2 consistency gate
int fuseVinsMotion(StatesGroup &st, const StatesGroup &post_prev, const double t0, const double t1,
                   const double weight, Eigen::Vector3d &dp_body_out)
{
    const double wait_start = ros::WallTime::now().toSec();
    while (ros::ok() && (g_vins_buf.empty() || g_vins_buf.back().t < t1) &&
           ros::WallTime::now().toSec() - wait_start < vio_wait_max)
    {
        ros::spinOnce(); // VINS samples are produced from the same IMU stream: wait a few ms for them
        usleep(500);
    }
    VinsSample a, b;
    const double dt = t1 - t0;
    if (dt <= 1e-3 || dt > 0.5 || !vinsAt(t0, a) || !vinsAt(t1, b))
        return -1;
    const Eigen::Vector3d dp_body = a.q.conjugate() * (b.p - a.p);
    const Eigen::Vector3d v_body = b.q.conjugate() * b.v;
    if (!dp_body.allFinite() || !v_body.allFinite() || dp_body.norm() > vio_max_speed * dt || v_body.norm() > vio_max_speed)
    {
        ROS_WARN_THROTTLE(2.0, "Rejecting implausible VINS motion: displacement %.3f m in %.3f s, speed %.3f m/s.",
                          dp_body.norm(), dt, v_body.norm());
        return -2;
    }
    dp_body_out = dp_body;
    const double w = std::max(weight, vio_min_weight);
    Eigen::Matrix<double, 6, DIM_OF_STATES> H = Eigen::Matrix<double, 6, DIM_OF_STATES>::Zero();
    H.block<3, 3>(0, 3).setIdentity();  // position
    H.block<3, 3>(3, 12).setIdentity(); // velocity
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    // the anchor (previous posterior) is uncertain too: add its covariance to the displacement noise
    // Body-frame anisotropic noise: with the non-holonomic constraint the IMU+NHC carry the lateral / vertical
    // motion and the VINS mainly supplies the forward motion (its vertical drift caused metres of z error).
    const double k_lv = nhc_enable ? nhc_vio_lat_scale : 1.0;
    const V3D sp2 = V3D(1.0, k_lv * k_lv, k_lv * k_lv) * (vio_pos_sigma * vio_pos_sigma / w);
    const V3D sv2 = V3D(1.0, k_lv * k_lv, k_lv * k_lv) * (vio_vel_sigma * vio_vel_sigma / w);
    R.block<3, 3>(0, 0) = post_prev.rot_end * sp2.asDiagonal() * post_prev.rot_end.transpose() + post_prev.cov.block<3, 3>(3, 3);
    R.block<3, 3>(3, 3) = st.rot_end * sv2.asDiagonal() * st.rot_end.transpose();
    Eigen::Matrix<double, 6, 1> y;
    y.head<3>() = post_prev.pos_end + post_prev.rot_end * dp_body - st.pos_end;
    y.tail<3>() = st.rot_end * v_body - st.vel_end;
    const Eigen::Matrix<double, 6, 6> S = H * st.cov * H.transpose() + R;
    const Eigen::Matrix<double, 6, 6> S_inv = S.inverse();
    const double d2 = y.dot(S_inv * y);
    if (vio_chi2_gate > 0 && d2 > vio_chi2_gate)
    {
        ROS_WARN_THROTTLE(2.0, "Rejecting VINS motion by the chi2 gate: d2 %.1f (dp %.3f m vs filter %.3f m).", d2,
                 dp_body.norm(), (post_prev.rot_end.transpose() * (st.pos_end - post_prev.pos_end)).norm());
        return -4;
    }
    Eigen::Matrix<double, DIM_OF_STATES, 6> K = st.cov * H.transpose() * S_inv;
    for (int r = 0; r < DIM_OF_STATES; ++r)
        if (!((r >= 3 && r < 6) || (r >= 12 && r < 15)))
            K.row(r).setZero();
    const Eigen::Matrix<double, DIM_OF_STATES, 1> dx = K * y;
    if (!dx.allFinite())
        return -3;
    st += dx;
    const Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> IKH =
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() - K * H;
    st.cov = IKH * st.cov * IKH.transpose() + K * R * K.transpose(); // Joseph form (K rows are masked)
    return 1;
}

void tn_cbk(const sensor_msgs::Image::ConstPtr &msg_in)
{
    std::istringstream iss(msg_in->header.frame_id);
    iss>>recv_n;

    double alpha_t = recv_n / Mta;
    zeta_t = 2.0 / (1.0 + std::exp(-alpha_t)) - 1;
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
/*-------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------*/
double sync_lidar_timeout = 0;
double last_scan_duration = 0.1; // scan period, used for scans without points
double last_integrated_time = 0;
bool sync_has_first_frame = 0;

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {

        if(sync_has_first_frame)
        {
            sync_lidar_timeout = ros::Time::now().toSec() - last_integrated_time;
        }
        else
        {
            sync_lidar_timeout = 0;
        }

        return false;
    }

    sync_has_first_frame = 1;
    sync_lidar_timeout = 0;
    last_integrated_time = ros::Time::now().toSec();

    /*** push lidar frame ***/
    if (!lidar_pushed)
    {
        meas.lidar.reset(new PointCloudXYZI());
        pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        // A scan can be empty when the LiDAR is fully blocked (dense smoke): points.back() on it crashed the node.
        if (meas.lidar->points.empty())
            observation_end_time = meas.lidar_beg_time + last_scan_duration;
        else
        {
            observation_end_time = meas.lidar_beg_time + meas.lidar->points.back().normal_z;
            const double d = meas.lidar->points.back().normal_z;
            if (d > 0.02 && d < 0.5)
                last_scan_duration = d;
        }
        meas.observation_end_time = observation_end_time;
        // std::cout << "timespan: " << meas.lidar->points.back().normal_z << ", point end ratio: " << meas.lidar->points.back().normal_x << std::endl;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < observation_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < observation_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > observation_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // if (meas.imu.empty()) return false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<observation_end_time<<std::endl;
    return true;
}
/*-------------------------------------------------------------------------------------------------------------------*/


vector<PointVector> Nearest_Points;
double filter_size_map_min = 0;
int feats_down_size = 0; 
int add_point_size = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void saveTrajectoryTUMformat(std::ostream &fout, std::string &stamp, Eigen::Vector3d &xyz, Eigen::Quaterniond &xyzw)
{
    fout << stamp << " " << xyz[0] << " " << xyz[1] << " " << xyz[2] << " " << xyzw.x() << " " << xyzw.y() << " " << xyzw.z() << " " << xyzw.w() << std::endl;
}

void saveTrajectoryTUMformat(std::ostream &fout, std::string &stamp, double x, double y, double z, double qx, double qy, double qz, double qw)
{
    fout << stamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    std::string imu_topic;
    std::string tis_topic;
    std::string lidar_feature_topic;
    bool save_tum_traj;
    bool ikdtree_async_rebuild;
    std::string trajectory_output_dir;
    std::string trajectory_run_id;

    /*** variables initialize ***/
    nh.param<std::string>("common/imuTopic", imu_topic, "/livox/imu");
    nh.param<std::string>("common/tisTopic", tis_topic, "/daliti/vins/odometry/vio_odometry");
    nh.param<std::string>("common/lidar_feature_topic", lidar_feature_topic, "/laser_cloud_surf");
    nh.param<int>("common/featptsThreshold", featptsThreshold, 100);
    nh.param<bool>("mapping/dense_map_enable", dense_map_en, false);
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 10);
    nh.param<double>("mapping/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("mapping/cube_side_length", cube_len, 1000);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
    nh.param<bool>("mapping/save_tum_traj", save_tum_traj, false);
    nh.param<bool>("mapping/ikdtree_async_rebuild", ikdtree_async_rebuild, true);
    nh.param<std::string>("mapping/trajectory_output_dir", trajectory_output_dir, std::string(ROOT_DIR) + "Log");
    nh.param<std::string>("mapping/trajectory_run_id", trajectory_run_id, "");
    nh.param<bool>("mapping/vio_fusion_enable", vio_fusion_enable, true);
    nh.param<bool>("mapping/fusion_debug_log", fusion_debug_log, false);
    nh.param<double>("mapping/vio_min_weight", vio_min_weight, 0.1);
    nh.param<double>("mapping/vio_chi2_gate", vio_chi2_gate, 22.46);
    nh.param<bool>("mapping/nhc_enable", nhc_enable, false);
    nh.param<double>("mapping/nhc_lever", nhc_lever, 0.12);
    nh.param<double>("mapping/nhc_sigma_lat", nhc_sigma_lat, 0.08);
    nh.param<double>("mapping/nhc_sigma_vert", nhc_sigma_vert, 0.05);
    nh.param<double>("mapping/nhc_vio_lat_scale", nhc_vio_lat_scale, 5.0);
    nh.param<bool>("mapping/tilt_correction_enable", tilt_correction_enable, true);
    nh.param<double>("mapping/tilt_tau", tilt_tau, 10.0);
    nh.param<double>("mapping/tilt_max_dyn", tilt_max_dyn, 2.0);
    nh.param<double>("mapping/tilt_max_acc", tilt_max_acc, 0.6);
    nh.param<double>("mapping/vio_fusion_max_zeta_l", vio_fusion_max_zeta_l, 0.5);
    nh.param<double>("mapping/vio_pos_sigma", vio_pos_sigma, 0.03);
    nh.param<double>("mapping/vio_vel_sigma", vio_vel_sigma, 0.10);
    nh.param<double>("mapping/vio_max_speed", vio_max_speed, 3.0);
    nh.param<double>("mapping/vio_wait_max", vio_wait_max, 0.03);
    nh.param<bool>("mapping/lidar_weight_by_zeta", lidar_weight_by_zeta, true);
    nh.param<double>("mapping/lidar_min_weight", lidar_min_weight, 0.05);
    bool gravity_align = true;
    double gyr_bias_rw = 5.0e-5, acc_bias_rw = 1.5e-3, init_bg_std = 1.0e-3, init_ba_std = 5.0e-2, init_grav_std = 1.0e-2;
    nh.param<bool>("mapping/gravity_align", gravity_align, true);
    nh.param<double>("mapping/gyr_bias_rw", gyr_bias_rw, 5.0e-5);
    nh.param<double>("mapping/acc_bias_rw", acc_bias_rw, 1.5e-3);
    nh.param<double>("mapping/init_bg_std", init_bg_std, 1.0e-3);
    nh.param<double>("mapping/init_ba_std", init_ba_std, 5.0e-2);
    nh.param<double>("mapping/init_grav_std", init_grav_std, 1.0e-2);
    nh.param<double>("common/beta", beta, 0.05);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    std::cout << "R: \n"
              << Lidar_R_wrt_IMU << std::endl;
    std::cout << "T: " << Lidar_T_wrt_IMU.transpose() << std::endl;

    ROS_INFO("\033[1;32m----> eskf_lio mapping Started.\033[0m");

    ros::Subscriber sub_pcl = nh.subscribe(lidar_feature_topic, 20000, feat_points_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 20000, imu_cbk);
    ros::Subscriber sub_tis = nh.subscribe<nav_msgs::Odometry>(tis_topic, 20000, tis_cbk);
    ros::Subscriber sub_n = nh.subscribe<sensor_msgs::Image> ("/camera/edge", 20000, tn_cbk);
    ros::Subscriber sub_lio_failed_flag = nh.subscribe("/daliti/lidar/status", 100, lidar_status_callback);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubLaserCloudFullResImuFrame = nh.advertise<sensor_msgs::PointCloud2>("/cloud_undistort", 100);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    /*** variables definition ***/

    double deltaT, deltaR, first_lidar_time = 0;
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    nav_msgs::Odometry odomAftMapped;

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    std::shared_ptr<ImuProcess> p_imu(new ImuProcess());
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->gravity_align = gravity_align;
    p_imu->gyr_bias_rw = gyr_bias_rw;
    p_imu->acc_bias_rw = acc_bias_rw;
    p_imu->init_bg_std = init_bg_std;
    p_imu->init_ba_std = init_ba_std;
    p_imu->init_grav_std = init_grav_std;
    nh.param<double>("mapping/init_still_gyro", p_imu->init_still_gyro, 0.02);
    ROS_INFO("fusion: vio_fusion %d (max zeta_l %.2f, sigma p %.3f m v %.3f m/s, max speed %.2f m/s), lidar_weight_by_zeta %d, gravity_align %d, nhc %d, tilt %d",
             int(vio_fusion_enable), vio_fusion_max_zeta_l, vio_pos_sigma, vio_vel_sigma, vio_max_speed,
             int(lidar_weight_by_zeta), int(gravity_align), int(nhc_enable), int(tilt_correction_enable));
    ikdtree.set_async_rebuild(ikdtree_async_rebuild);
    ROS_INFO("ikd-Tree rebuild mode: %s", ikdtree_async_rebuild ? "asynchronous" : "synchronous deterministic");

    //  create folder
    std::string command = "mkdir -p " + std::string(ROOT_DIR) + "Log";
    system(command.c_str());
    /*** debug record ***/
    std::ofstream fout_pre, fout_out;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~" << ROOT_DIR << " file opened" << std::endl;
    else
        std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << std::endl;

    std::ofstream fout_traj;
    std::string trajectory_file_path;
    if (save_tum_traj)
    {
        if (trajectory_run_id.empty())
        {
            const std::time_t now = std::time(nullptr);
            std::tm local_time;
            localtime_r(&now, &local_time);
            std::ostringstream generated_run_id;
            generated_run_id << std::put_time(&local_time, "%Y%m%d_%H%M%S") << "_" << getpid();
            trajectory_run_id = generated_run_id.str();
        }

        if (!trajectory_output_dir.empty() && trajectory_output_dir.back() != '/')
            trajectory_output_dir += "/";
        trajectory_file_path = trajectory_output_dir + "daliti_" + trajectory_run_id + ".tum";
        fout_traj.open(trajectory_file_path, std::ios::out);
        if (fout_traj.is_open())
            ROS_INFO_STREAM("Saving TUM trajectory to " << trajectory_file_path);
        else
            ROS_ERROR_STREAM("Failed to open TUM trajectory file: " << trajectory_file_path);
    }

    std::ofstream processing_time_file("~/daliti/src/DaLiTI/eskf_lio/Log/iekf_processing_time.txt", std::ios::out);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();
        while (sync_packages(Measures))
        {
            // start record time
            auto start_time = std::chrono::high_resolution_clock::now();

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
            }
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            p_imu->Process(Measures, state, feats_undistort, EKF_stop_flg);

            int vio_status = 0;
            double lidar_weight_used = 1.0;
            Eigen::Vector3d vio_dp_body(Zero3d);
            StatesGroup state_propagat(state);
            pos_lid = state_propagat.pos_end + state_propagat.rot_end * state_propagat.T_L_I;

            // Before the first map: nothing to do without points. Afterwards an empty scan (LiDAR blocked by
            // smoke) is processed as a degenerate frame: IMU prediction, thermal-inertial / NHC / tilt updates, a pose.
            const bool empty_scan = feats_undistort == NULL || feats_undistort->empty();
            if (empty_scan && (ikdtree.Root_Node == nullptr || !have_last_post))
            {
                std::cout << "No point, not ready for odometry, skip this scan" << std::endl;
                continue;
            }

            std::cout << ANSI_COLOR_YELLOW_BOLD << "-------------------------------------------------------------" << ANSI_COLOR_RESET << std::endl;
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Compute the euler angle ***/
            V3D euler_cur = RotMtoEuler(state.rot_end);

            std::cout << "pre-integrated states, r:" << euler_cur.transpose() * 57.3 << " ,p:"
                      << state.pos_end.transpose() << " ,v:" << state.vel_end.transpose() << " ,bg:"
                      << state.bias_g.transpose() << " ,ba:" << state.bias_a.transpose() << std::endl;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the features of new frame ***/
            if (empty_scan)
                feats_down->clear();
            else
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down);
            }

            feats_down_size = feats_down->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                // build the first map only from a valid state and a reasonably dense scan
                if (feats_down->points.size() > 100 && state.pos_end.allFinite() && state.rot_end.allFinite() &&
                    state.vel_end.allFinite())
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();

            std::cout << ANSI_COLOR_GREEN << "[ mapping ]: Raw feature num: " << feats_undistort->points.size()
                      << " downsamp num: " << feats_down_size << " Map num: " << featsFromMapNum << ANSI_COLOR_RESET << std::endl;

            /*** ICP and iterated Kalman filter update ***/
            PointCloudXYZI::Ptr coeffSel_tmpt(new PointCloudXYZI(*feats_down)); 
            PointCloudXYZI::Ptr feats_down_updated(new PointCloudXYZI(*feats_down));
            std::vector<double> res_last(feats_down_size, 1000.0); // initial

            if (featsFromMapNum >= 5)
            {

                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);

                Nearest_Points.resize(feats_down_size);

                std::vector<bool> point_selected_surf(feats_down_size, true);

                int rematch_num = 0;
                bool rematch_en = 0;
                flg_EKF_converged = 0;
                deltaR = 0.0;
                deltaT = 0.0;

                for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++)
                {

                    laserCloudOri->clear();
                    coeffSel->clear();

                    /** closest surface search and residual computation **/
                    // omp_set_num_threads(4);
                    // #pragma omp parallel for
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        PointType &point_body = feats_down->points[i];
                        PointType &point_world = feats_down_updated->points[i];

                        /* transform to world frame */
                        V3D p_body(point_body.x, point_body.y, point_body.z);
                        V3D p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);
                        // pointBodyToWorld(&point_body, &point_world);
                        point_world.x = p_global(0);
                        point_world.y = p_global(1);
                        point_world.z = p_global(2);
                        point_world.intensity = point_body.intensity;

                        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

                        auto &points_near = Nearest_Points[i]; 

                        if (iterCount == 0 || rematch_en)
                        {
                            /** Find the closest surfaces in the map **/
                            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

                            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                                     : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                  : true;
                        }

                        if (point_selected_surf[i] == false)
                            continue;

                        VF(4)
                        pabcd;
                        point_selected_surf[i] = false;
                        if (esti_plane(pabcd, points_near, 0.1f)) 
                        {
                            // loss fuction
                            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); 
                            // if(fabs(pd2) > 0.1) continue;
                            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); 

                            if ((s > 0.9))
                            {

                                point_selected_surf[i] = true;
                                coeffSel_tmpt->points[i].x = pabcd(0);
                                coeffSel_tmpt->points[i].y = pabcd(1);
                                coeffSel_tmpt->points[i].z = pabcd(2);
                                coeffSel_tmpt->points[i].intensity = pd2;

                                res_last[i] = std::abs(pd2);
                            }
                        }
                    }

                    double total_residual = 0.0;
                    effct_feat_num = 0;
                    std::cout << ANSI_COLOR_MAGENTA << "coeffSel_tmpt->points.size(): " << coeffSel_tmpt->points.size() << std::endl;
                    for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
                    {
                        if (point_selected_surf[i] && (res_last[i] <= 2.0)) 
                        {
                            laserCloudOri->push_back(feats_down->points[i]);
                            coeffSel->push_back(coeffSel_tmpt->points[i]);
                            total_residual += res_last[i];
                            effct_feat_num++;
                        }
                    }


                    effct_feat_numQueue.push(effct_feat_num);


                    if (effct_feat_numQueue.size() > QUEUE_SIZE)
                    {
                        effct_feat_numQueue.pop();
                    }

                    EKF_stop_flg = false;
                    std::queue<int> tempQueue = effct_feat_numQueue; 
                    while (!tempQueue.empty())
                    {
                        if (tempQueue.front() <= dynamic_effect_featurepoints_threshold)
                        {
                            EKF_stop_flg = true;
                            // std::cout << ANSI_COLOR_GREEN_BG << "LiDAR STOP" << ANSI_COLOR_RESET << std::endl;
                            break;
                        }
                        tempQueue.pop();
                    }

                    // Degradation-aware LiDAR weight: the point noise is inflated by 1 / zeta_l_norm, so a
                    // partially degraded LiDAR is still used, but the thermal-inertial measurement keeps a say.
                    double laser_point_cov = LASER_POINT_COV;
                    if (lidar_weight_by_zeta && tis_online && lidar_cnt >= 100)
                    {
                        const double zl = 2.0 / (1.0 + std::exp(-effct_feat_num / (beta * Nla))) - 1;
                        const double zn = (zl + zeta_t) > 1e-12 ? zl / (zl + zeta_t) : 1.0;
                        lidar_weight_used = std::max(lidar_min_weight, std::min(1.0, zn));
                        laser_point_cov = LASER_POINT_COV / lidar_weight_used;
                    }
                    // if (effct_feat_num < 50)
                    // {
                    //     // TODO: do something
                    //     ROS_WARN("No Enough Effective points! May catch error.");
                    //     std::cout << ANSI_COLOR_RED << "EKF STOP FLG" << ANSI_COLOR_RESET << std::endl;
                    //     EKF_stop_flg = true;
                    // }
                    // else
                    // {
                    //     std::cout << ANSI_COLOR_RED << "EKF RUN FLG" << ANSI_COLOR_RESET << std::endl;
                    //     EKF_stop_flg = false;
                    // }

                    res_mean_last = total_residual / effct_feat_num;
                    std::cout << ANSI_COLOR_GREEN << "[-- mapping --]: Effective feature num: " << effct_feat_num
                              << " res_mean_last " << res_mean_last << ANSI_COLOR_RESET << std::endl;
                    // fout
                    fout_out << std::setw(10) << Measures.lidar_beg_time - first_lidar_time << " " << effct_feat_num
                             << " " << res_mean_last << " " << flg_EKF_converged << " " << EKF_stop_flg << " " << coeffSel_tmpt->points.size()<<" "<<RECV_LIO_FAIL_FLAG << " "<<recv_n << std::endl;


                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    Eigen::MatrixXd Hsub(effct_feat_num, 12);
                    Eigen::VectorXd meas_vec(effct_feat_num);
                    Hsub.setZero();

                    // omp_set_num_threads(4);
                    // #pragma omp parallel for
                    for (int i = 0; i < effct_feat_num; i++)
                    {
                        const PointType &laser_p = laserCloudOri->points[i];
                        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z); 
                        M3D point_be_crossmat; 
                        point_be_crossmat << SKEW_SYM_MATRX(point_this_be); 
                        V3D point_this = state.R_L_I * point_this_be + state.T_L_I; 
                        M3D point_crossmat;
                        point_crossmat << SKEW_SYM_MATRX(point_this);

                        /*** get the normal vector of closest surface/corner ***/
                        const PointType &norm_p = coeffSel->points[i]; 
                        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z); 

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        //  FIXME:
                        
                        
                        V3D C(state.rot_end.transpose() * norm_vec); 
                        V3D A(point_crossmat * C); 
                        if (extrinsic_est_en)
                        {
                            V3D B(point_be_crossmat * state.R_L_I.transpose() * C);
                            Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
                        }
                        else
                            Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        meas_vec(i) = -norm_p.intensity; 
                        
                    }

                    Eigen::Vector3d rot_add, t_add;
                    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
                    Eigen::MatrixXd K(DIM_OF_STATES, effct_feat_num);

                    /*** Iterative Kalman Filter Update ***/
                    if (!flg_EKF_inited && !EKF_stop_flg)
                    {
                        std::cout << ANSI_COLOR_YELLOW << "------------------------------------------------" << std::endl;
                        std::cout << "EKF start" << std::endl;
                        std::cout << "------------------------------------------------" << ANSI_COLOR_RESET << std::endl;
                        /*** only run in initialization period ***/
                        Eigen::MatrixXd H_init(Eigen::Matrix<double, 15, DIM_OF_STATES>::Zero());
                        Eigen::MatrixXd z_init(Eigen::Matrix<double, 15, 1>::Zero());
                        H_init.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
                        z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
                        z_init.block<3, 1>(3, 0) = -state.pos_end;
                        z_init.block<3, 1>(6, 0) = -Log(state.R_L_I);
                        z_init.block<3, 1>(9, 0) = -state.T_L_I;

                        auto H_init_T = H_init.transpose();
                        auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + 0.0001 * Eigen::Matrix<double, 12, 12>::Identity()).inverse(); 
                        solution = K_init * z_init;

                        solution.block<12, 1>(0, 0).setZero();
                        state += solution;
                        state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * state.cov; 
                    }
                    else if (!EKF_stop_flg)
                    {
                        auto &&Hsub_T = Hsub.transpose();
                        H_T_H.block<12, 12>(0, 0) = Hsub_T * Hsub;
                        
                        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
                            (H_T_H + (state.cov / laser_point_cov).inverse()).inverse();
                        K = K_1.block<DIM_OF_STATES, 12>(0, 0) * Hsub_T;                
                                                                                        

                        // auto K_in = (H_T_H + state.cov.inverse() * LASER_POINT_COV).inverse() * Hsub_T;
                        // auto K_in = (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse() * Hsub_T;

                        // solution = K * meas_vec;
                        // state += solution;

                        auto vec = state_propagat - state;
                        // solution = K * (meas_vec - Hsub * vec.block<6,1>(0,0));
                        // state = state_propagat + solution;

                        solution = K * meas_vec + vec - K * Hsub * vec.block<12, 1>(0, 0);
                        state += solution; 

                        rot_add = solution.block<3, 1>(0, 0);
                        t_add = solution.block<3, 1>(3, 0);

                        flg_EKF_converged = false;

                        if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))
                        {
                            flg_EKF_converged = true;
                        }

                        deltaR = rot_add.norm() * 57.3;
                        deltaT = t_add.norm() * 100;

                    }
                    else
                    {
                        // LiDAR degenerate: no LiDAR update this frame. Keep the IMU prediction, which was
                        // already corrected by the thermal-inertial motion measurement when available.
                        state = state_propagat;
                        flg_EKF_inited = false; // no LiDAR covariance update below
                    }
                    euler_cur = RotMtoEuler(state.rot_end);
                    std::cout << "flg_EKF_converged: " << flg_EKF_converged << std::endl;
                    std::cout << "update: R" << euler_cur.transpose() * 57.3 << " p " << state.pos_end.transpose() << " v " << state.vel_end.transpose() << " bg" << state.bias_g.transpose() << " ba" << state.bias_a.transpose() << std::endl;
                    std::cout << "dR & dT: " << deltaR << " " << deltaT << " res norm:" << res_mean_last << std::endl;
                    state.display();
                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                    {
                        rematch_en = true; 
                        rematch_num++;
                        std::cout << "rematch_num: " << rematch_num << std::endl;
                    }

                    /*** Convergence Judgements and Covariance Update ***/
                    if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    {
                        if (flg_EKF_inited)
                        {
                            /*** Covariance Update ***/
                            G.block<DIM_OF_STATES, 12>(0, 0) = K * Hsub;
                            state.cov = (I_STATE - G) * state.cov; 
                            total_distance += (state.pos_end - position_last).norm();
                            position_last = state.pos_end;

                            std::cout << ANSI_COLOR_YELLOW << "position: " << state.pos_end.transpose() << " total distance: " << total_distance
                                      << ", extT: " << state.T_L_I.transpose() << ", extR: " << state.R_L_I << ANSI_COLOR_RESET << std::endl;
                        }

                        break;
                    }
                    else if (EKF_stop_flg)
                    {
                        position_last = state.pos_end;
                        std::cout << ANSI_COLOR_BLUE << "position: " << state.pos_end.transpose() << " total distance: " << total_distance
                                  << ", extT: " << state.T_L_I.transpose() << ", extR: " << state.R_L_I << ANSI_COLOR_RESET << std::endl;
                        break;
                    }
                }


                // Degradation factors (logged for the Fig. 7/8 style analysis and used to weight the
                // measurements of the next frame). They no longer interpolate between state increments.
                {
                    const double alpha_l = effct_feat_num / (beta * Nla);
                    zeta_l = 2.0 / (1.0 + std::exp(-alpha_l)) - 1;
                    double zeta_l_norm = (zeta_l + zeta_t) > 1e-12 ? zeta_l / (zeta_l + zeta_t) : 1.0;
                    if (lidar_cnt < 100)
                        zeta_l_norm = 1;
                    ROS_INFO("zeta_l: %f, zeta_t: %f, zeta_l_norm: %f", zeta_l, zeta_t, zeta_l_norm);
                    prev_zeta_l_norm = zeta_l_norm;
                    prev_zeta_t_norm = 1.0 - zeta_l_norm;
                }

                /*** thermal-inertial motion measurement (after the LiDAR update, so the LiDAR data association
                 *** still starts from the IMU prediction). Its weight is the normalized thermal factor, so it
                 *** dominates only where the LiDAR is degraded or geometrically unconstrained. ***/
                const bool lidar_degraded = EKF_stop_flg || prev_zeta_l_norm < vio_fusion_max_zeta_l;
                if (nhc_enable && lidar_degraded && lidar_cnt >= 100)
                    applyNhc(state, p_imu->angvel_last(2));
                // only while the LiDAR update is stopped: in partially degraded frames the LiDAR keeps the attitude
                // consistent with the map and its corrections make the velocity difference a poor acceleration estimate
                if (tilt_correction_enable && EKF_stop_flg && have_last_post && lidar_cnt >= 100)
                {
                    const double dt_f = Measures.observation_end_time - last_post_time;
                    if (dt_f > 1e-3 && dt_f < 0.5)
                        tiltCorrection(state, Measures, p_imu->accScale(), (state.vel_end - last_post_state.vel_end) / dt_f, dt_f);
                }
                if (vio_fusion_enable && tis_online && have_last_post && lidar_cnt >= 100 && lidar_degraded)
                {
                    vio_status = fuseVinsMotion(state, last_post_state, last_post_time, Measures.observation_end_time,
                                                prev_zeta_t_norm, vio_dp_body);
                    if (vio_status == 1)
                        euler_cur = RotMtoEuler(state.rot_end);
                }
                {
                    // one line when the degraded (thermal-inertial) mode is entered or left (rate-limited)
                    static bool was_degraded = false;
                    static double last_mode_msg = -1.0;
                    const bool degraded_now = lidar_cnt >= 100 && lidar_degraded;
                    if (degraded_now != was_degraded && Measures.observation_end_time - last_mode_msg > 2.0)
                    {
                        if (degraded_now)
                            ROS_INFO("LiDAR degraded (zeta_l_norm %.2f, LiDAR update %s): IMU + thermal-inertial motion%s.",
                                     prev_zeta_l_norm, EKF_stop_flg ? "stopped" : "down-weighted", nhc_enable ? " + NHC" : "");
                        else
                            ROS_INFO("LiDAR recovered (zeta_l_norm %.2f).", prev_zeta_l_norm);
                        was_degraded = degraded_now;
                        last_mode_msg = Measures.observation_end_time;
                    }
                    if (fusion_debug_log)
                    {
                        // LIO displacement of this frame in the body frame of the previous posterior (compare with vio_dp)
                        const V3D lio_dp = have_last_post ? V3D(last_post_state.rot_end.transpose() * (state.pos_end - last_post_state.pos_end)) : V3D(Zero3d);
                        ROS_INFO("fusion: lidar_stop %d vio %d lidar_w %.3f vio_dp %.4f %.4f %.4f lio_dp %.4f %.4f %.4f bg %.5f %.5f %.5f",
                                 int(EKF_stop_flg), vio_status, lidar_weight_used, vio_dp_body(0), vio_dp_body(1), vio_dp_body(2),
                                 lio_dp(0), lio_dp(1), lio_dp(2), state.bias_g(0), state.bias_g(1), state.bias_g(2));
                    }
                }
                std::cout << ANSI_COLOR_GREEN_BOLD << "[ mapping ]: iteration count: " << iterCount + 1 << ANSI_COLOR_RESET << std::endl;

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
//?/ save trajectory, state estimation update finished.
                V3D ext_euler = RotMtoEuler(state.R_L_I);
                fout_pre << std::setw(10) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << state.pos_end.transpose()
                         << " " << ext_euler.transpose() << " " << state.T_L_I.transpose()
                         << " " << state.vel_end.transpose() << " " << state.bias_g.transpose()
                         << " " << state.bias_a.transpose() << " " << state.gravity.transpose()
                         << " " << feats_undistort->points.size() << std::endl;

                if (save_tum_traj)
                {
                    // Stamp = time of the state (end of the scan), as on /aft_mapped_to_init. A scan whose last point
                    // time is too late (seen in smoke) moves the state ahead of the following scan ends; rows are
                    // written only with strictly increasing stamps so the file stays a valid TUM trajectory.
                    static long long last_tum_stamp_us = -1;
                    const long long stamp_us = std::llround(Measures.observation_end_time * 1e6);
                    if (fout_traj.is_open() && stamp_us > last_tum_stamp_us)
                    {
                        last_tum_stamp_us = stamp_us;
                        std::string tstamp = std::to_string(Measures.observation_end_time);
                        Eigen::Quaterniond q_curr = Eigen::Quaterniond(state.rot_end);
                        q_curr.normalize();
                        saveTrajectoryTUMformat(fout_traj, tstamp, state.pos_end, q_curr);
                    }
                }
                last_post_state = state;
                last_post_time = Measures.observation_end_time;
                have_last_post = true;
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
//?/ start of mapping

                /*** add new frame points to map ikdtree ***/
                if (!EKF_stop_flg) 
                {
                    map_incremental();
                }

                {
                    PointVector().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;
                }
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
            }
            

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
//?/ pub slam results here

            /******* Publish current frame points in world coordinates:  *******/
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? (*feats_undistort) : (*feats_down);

            int laserCloudFullResNum = laserCloudFullRes2->points.size();

            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            {
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    RGBpointBodyToWorld(&laserCloudFullRes2->points[i], &temp_point);
                    laserCloudFullResColor->push_back(temp_point);
                }

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(Measures.observation_end_time); // ros::Time::now(); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.frame_id = "camera_init";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }


            /******* Publish current frame points in IMU coordinates:  *******/
            sensor_msgs::PointCloud2 laserCloudFullRes4;
            pcl::toROSMsg(*feats_undistort, laserCloudFullRes4);
            laserCloudFullRes4.header.stamp = ros::Time().fromSec(Measures.observation_end_time); // ros::Time::now(); //.fromSec(last_timestamp_lidar);
            laserCloudFullRes4.header.frame_id = "/aft_mapped";
            pubLaserCloudFullResImuFrame.publish(laserCloudFullRes4);

            /******* Publish Effective points *******/
            {
                laserCloudFullResColor->clear();
                pcl::PointXYZI temp_point;
                for (int i = 0; i < effct_feat_num; i++)
                {
                    RGBpointBodyToWorld(&laserCloudOri->points[i], &temp_point);
                    laserCloudFullResColor->push_back(temp_point);
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(Measures.observation_end_time);
                laserCloudFullRes3.header.frame_id = "camera_init";
                pubLaserCloudEffect.publish(laserCloudFullRes3);
            }

            /******* Publish Maps:  *******/
            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg(*featsFromMap, laserCloudMap);
            laserCloudMap.header.stamp = ros::Time().fromSec(Measures.observation_end_time);
            ;
            laserCloudMap.header.frame_id = "camera_init";
            pubLaserCloudMap.publish(laserCloudMap);

            /******* Publish Odometry ******/
            // publish the final state (the blended orientation used to differ from the published one)
            Eigen::Quaterniond q_pub(state.rot_end);
            q_pub.normalize();
            geometry_msgs::Quaternion geoQuat;
            geoQuat.x = q_pub.x();
            geoQuat.y = q_pub.y();
            geoQuat.z = q_pub.z();
            geoQuat.w = q_pub.w();
            odomAftMapped.header.frame_id = "camera_init";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time().fromSec(Measures.observation_end_time);
            odomAftMapped.pose.pose.orientation.x = geoQuat.x;
            odomAftMapped.pose.pose.orientation.y = geoQuat.y;
            odomAftMapped.pose.pose.orientation.z = geoQuat.z;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = state.pos_end(0);
            odomAftMapped.pose.pose.position.y = state.pos_end(1);
            odomAftMapped.pose.pose.position.z = state.pos_end(2);
            // imuPreintegration reads covariance[0] == 1 as "degenerate" and then weakens this correction
            odomAftMapped.pose.covariance[0] = EKF_stop_flg ? 1.0 : 0.0;

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                            odomAftMapped.pose.pose.position.y,
                                            odomAftMapped.pose.pose.position.z));
            q.setW(odomAftMapped.pose.pose.orientation.w);
            q.setX(odomAftMapped.pose.pose.orientation.x);
            q.setY(odomAftMapped.pose.pose.orientation.y);
            q.setZ(odomAftMapped.pose.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "/aft_mapped"));

            msg_body_pose.header.stamp = ros::Time().fromSec(Measures.observation_end_time);
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = state.pos_end(0);
            msg_body_pose.pose.position.y = state.pos_end(1);
            msg_body_pose.pose.position.z = state.pos_end(2);
            msg_body_pose.pose.orientation.x = geoQuat.x;
            msg_body_pose.pose.orientation.y = geoQuat.y;
            msg_body_pose.pose.orientation.z = geoQuat.z;
            msg_body_pose.pose.orientation.w = geoQuat.w;

            /******* Publish Path ********/
            msg_body_pose.header.frame_id = "camera_init";
            path.poses.push_back(msg_body_pose);
            pubPath.publish(path);
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

            // end record time
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            std::cout << "Processing time for this frame: " << duration << " ms" << std::endl;

            if (processing_time_file.is_open())
            {
                processing_time_file << std::fixed << std::setprecision(6) << Measures.lidar_beg_time << " " << duration << " ms" << std::endl;
            }

            // Do not process a backlog of lidar frames against one stale TIS
            // snapshot. Drain callbacks at every lidar-frame boundary so the
            // next frame sees all VINS/IMU propagation available at that
            // point, independent of how many lidar packets were buffered.
            ros::spinOnce();
        }
        status = ros::ok();
        rate.sleep();
    }

    // close the file after the loop
    if (processing_time_file.is_open())
    {
        processing_time_file.close();
    }

    return 0;
}
