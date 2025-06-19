/******************************************************************************

   REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
   Copyright (C) 2016  Juan José Tarrio

   Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
   for a Monocular Camera. In Proceedings of the IEEE International Conference
   on Computer Vision (pp. 702-710).

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

 *******************************************************************************/

#ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include "edge_finder.h"
#include <mutex>
#include "UtilLib/NormalDistribution.h"
// #include "opencv2/core/core.hpp"
#ifdef Success
  #undef Success
#endif
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace  rebvo{

class edge_tracker : public edge_finder
{
public:
    using edge_finder::edge_finder;
    int search_match(KeyLine &k,TooN::Vector <3> Vel,TooN::Matrix <3,3> RVel,TooN::Matrix <3,3> BackRot,double min_thr_mod,double min_thr_ang,double max_radius, double loc_uncertainty);
    double EstimateQuantile(double s_rho_min, double s_rho_max, double percentile, int n);
    int Regularize_1_iter(double thresh);
};

}


#endif // EDGE_TRACKER_H
