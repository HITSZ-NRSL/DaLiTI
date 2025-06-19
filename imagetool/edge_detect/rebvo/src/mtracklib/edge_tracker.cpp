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



#include "mtracklib/edge_tracker.h"

#include <iostream>

#include <stdio.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>

const bool complex_regularization=false;

using namespace TooN;
namespace  rebvo{

//********************************************************************************
// EstimateQuantile() uses histograms to estimate an uncertainty threshold
// for a certain quatile
//********************************************************************************

double edge_tracker::EstimateQuantile(double s_rho_min,     //Starting uncertainty on the histo
                                      double s_rho_max,     //Final uncertainty on the histo
                                      double percentile,    //Quantile threshold
                                      int n){               //Number of bins on the histo


    int histo[n];


    for(int i=0;i<n;i++){
        histo[i]=0;
    }

    for(int ikl=0;ikl<kn;ikl++){
        int i=n*(kl[ikl].s_rho-s_rho_min)/(s_rho_max-s_rho_min);    //histogram position

        i=i>n-1?n-1:i;
        i=i<0?0:i;

        histo[i]++;     //count
    }

    double s_rho=1e3;

    for(int i=0,a=0;i<n;i++){

        if(a>percentile*kn){    //count in the histogram until a certain percentile
                                //of the total number of keylines
            s_rho=(double)i*(s_rho_max-s_rho_min)/(double)n+s_rho_min;
            break;

        }

        a+=histo[i];
    }

    return s_rho;

}

}
