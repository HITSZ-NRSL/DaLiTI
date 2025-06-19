#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "utility/line_geometry.h" 

class FeaturePerFrame
{
public:
  FeaturePerFrame(const Eigen::Matrix<double, 8, 1> &_point, double td)
  {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    depth = _point(7);
    cur_td = td;
  }
  double cur_td;
  Vector3d point;
  Vector2d uv;
  Vector2d velocity;
  double z;
  bool is_used;
  double parallax;
  MatrixXd A;
  VectorXd b;
  double dep_gradient;
  double depth; // lidar depth, initialized with -1 from feature points in feature tracker node
};

class FeaturePerId
{
public:
  const int feature_id;
  int start_frame;
  vector<FeaturePerFrame> feature_per_frame;

  int used_num;
  bool is_outlier;
  bool is_margin;
  double estimated_depth;
  bool lidar_depth_flag;
  int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

  Vector3d gt_p;

  FeaturePerId(int _feature_id, int _start_frame, double _measured_depth)
      : feature_id(_feature_id), start_frame(_start_frame),
        used_num(0), estimated_depth(-1.0), lidar_depth_flag(false), solve_flag(0)
  {
    if (_measured_depth > 0)
    {
      estimated_depth = _measured_depth;
      lidar_depth_flag = true;
    }
    else
    {
      estimated_depth = -1;
      lidar_depth_flag = false;
    }
  }

  int endFrame();
};

class lineFeaturePerFrame
{
public:
    lineFeaturePerFrame(const Vector4d &line)
    {
        lineobs = line;
    }
    lineFeaturePerFrame(const Vector8d &line)
    {
        lineobs = line.head<4>();
        lineobs_R = line.tail<4>();
    }
    Vector4d lineobs;   
    Vector4d lineobs_R;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};
class lineFeaturePerId
{
public:
    const int feature_id;
    int start_frame;

    
    
    vector<lineFeaturePerFrame> linefeature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    bool is_triangulation;
    Vector6d line_plucker;

    Vector4d obs_init;
    Vector4d obs_j;
    Vector6d line_plk_init; // used to debug
    Vector3d ptw1;  // used to debug
    Vector3d ptw2;  // used to debug
    Eigen::Vector3d tj_;   // tij
    Eigen::Matrix3d Rj_;
    Eigen::Vector3d ti_;   // tij
    Eigen::Matrix3d Ri_;
    int removed_cnt;
    int all_obs_cnt;    

    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    lineFeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), solve_flag(0),is_triangulation(false)
    {
        removed_cnt = 0;
        all_obs_cnt = 1;
    }

    int endFrame();
};

class FeatureManager
{
  public:
  FeatureManager(Matrix3d _Rs[]);

  void setRic(Matrix3d _ric[]);

  void clearState();

  int getFeatureCount();

  int getLineFeatureCount();
  MatrixXd getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
  void setLineOrth(MatrixXd x, Vector3d Ps[], Matrix3d Rs[],Vector3d tic[], Matrix3d ric[]);
  MatrixXd getLineOrthVectorInCamera();
  void setLineOrthInCamera(MatrixXd x);

  double reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w );
  void removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
  void removeLineOutlier();
  bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &points, double td);
  void debugShow();
  vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

  // void updateDepth(const VectorXd &x);
  void setDepth(const VectorXd &x);
  void removeFailures();
  void clearDepth(const VectorXd &x);
  VectorXd getDepthVector();
  void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
  void triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
  void triangulateLine(double baseline);  // stereo line
  void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
  void removeBack();
  void removeFront(int frame_count);
  void removeOutlier();
  list<FeaturePerId> feature;
  list<lineFeaturePerId> linefeature;
  int last_track_num;

private:
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  const Matrix3d *Rs;
  Matrix3d ric[NUM_OF_CAM];
};

#endif