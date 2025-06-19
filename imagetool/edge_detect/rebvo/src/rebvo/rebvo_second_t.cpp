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

#include <iostream>
#include "rebvo/rebvo.h"
#include "mtracklib/kfvo.h"
#include <mutex>

using namespace std;

namespace  rebvo{

void  REBVO::SecondThread(REBVO *cf){

    using namespace TooN;


    COND_TIME_DEBUG(double dtp;)

    double error_vel=0;                 //Estimated error for the velocity
    double error_score=0;               //Residual energy from the tracking

    Vector <3> V=Zeros,W=Zeros,Pos=Zeros;   //Estimated Velocity, Rotation and Position
    Matrix<3,3> R=Identity,Pose=Identity;   //Rotation Matrix and Global Rotation


    Matrix <3,3> P_V=Identity*1e50,P_W=Identity*1e-10;

    //**** Set cpu Afinity for this thread *****

    if(cf->params.cpuSetAffinity){
        if(!REBVO::setAffinity(cf->params.cpu1)){
            std::cout <<"REBVO: Cannot set cpu affinity on the second thread";
            cf->quit=true;
        }
    }


    //***** Launch Thread 2 Pipeline thread ******

    std::thread Thr2(ThirdThread,cf);

    COND_TIME_DEBUG(util::interval_list tlist;)

    COND_TIME_DEBUG(util::timer t_loop;)

    //****** Dummy processing of the first frame ******
	{
    	PipeBuffer &new_buf=cf->pipe.RequestBuffer(1);
    	if(new_buf.quit){
        	cf->pipe.ReleaseBuffer(1);
        	PipeBuffer &old_buf=cf->pipe.RequestBuffer(2);
        	old_buf.quit=false;
        	cf->pipe.ReleaseBuffer(2);
        	cf->quit=true;

    	}else{
    		cf->pipe.ReleaseBuffer(1);
    	}
	}
    //****** Main loop *******
    while(!cf->quit){


        COND_TIME_DEBUG(dtp=t_loop.stop();std::cout << "1st exit" << std::endl;)

        // 2 pipeline buffers are used in this thread
        PipeBuffer &new_buf=cf->pipe.RequestBuffer(1);  //This represent the newest edge-map
        PipeBuffer &old_buf=cf->pipe.RequestBuffer(2);  //This represents the old edgemap

        if(new_buf.quit){                               //If quit flag on the new buffer,
            old_buf.quit=true;                          //release quit on the old for thread 2
            cf->pipe.ReleaseBuffer(1);
            cf->pipe.ReleaseBuffer(2);
            break;
        }

        COND_TIME_DEBUG(t_loop.start();)
        COND_TIME_DEBUG(tlist.clear();)

        COND_TIME_DEBUG(tlist.push_new();)
        //Estimate uncertainity cut-off for some quantile (90 percent typicaly)
        double s_rho_q=old_buf.ef->EstimateQuantile(RHO_MIN,RHO_MAX,cf->params.QCutOffQuantile,cf->params.QCutOffNumBins);
        COND_TIME_DEBUG(tlist.push_new();)

        new_buf.gt->build_field(*new_buf.ef,cf->params.SearchRange,new_buf.ef->getThresh());

        //***** Use the Tracker to obtain Velocity and Rotation  ****
                 //Regular procesing
        COND_TIME_DEBUG(tlist.push_new();)

        #ifdef USE_NE10
        TooN::Matrix<6,6,float> W_X;
        new_buf.gt->Minimizer_RV<float>(V,W,P_V,P_W,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,cf->params.TrackerInitType,cf->params.ReweigthDistance,error_vel,error_score,s_rho_q,cf->params.MatchNumThresh,cf->params.TrackerInitIterNum,W_X);
        #else        
        TooN::Matrix<6,6,double> W_X;
        new_buf.gt->Minimizer_RV<double>(V,W,P_V,P_W,*old_buf.ef,cf->params.TrackerMatchThresh,cf->params.TrackerIterNum,cf->params.TrackerInitType,cf->params.ReweigthDistance,error_vel,error_score,s_rho_q,cf->params.MatchNumThresh,cf->params.TrackerInitIterNum,W_X);
        #endif

        COND_TIME_DEBUG(tlist.push_new();)
        //Estimate position and pose incrementally
        cf->pipe.ReleaseBuffer(1);
        cf->pipe.ReleaseBuffer(2);
    }
    Thr2.join();
    return;
}
}
