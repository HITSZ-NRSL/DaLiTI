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


#ifndef REBVO_H
#define REBVO_H


#include <string>
#include <TooN/TooN.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <mutex>
#include "UtilLib/pipeline.h"
#include "mtracklib/edge_tracker.h"
#include "mtracklib/global_tracker.h"
#include "mtracklib/keyframe.h"
#include "VideoLib/customcam.h"
#include <functional>
#ifdef Success
  #undef Success
#endif
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace  rebvo{



constexpr int CBUFSIZE=0x08;
constexpr int CCAMBUFSIZE=0x04;

//#define TIME_DEBUG

#ifdef TIME_DEBUG
#define COND_TIME_DEBUG(arg) arg
#else
#define COND_TIME_DEBUG(arg)
#endif



struct REBVOParameters{

    //Camara parameters

    Size2D ImageSize;                   //Frame size
    float z_f_x;                        //Camera XY focal length
    float z_f_y;
    float pp_y;                         //Camera principal point
    float pp_x;
    cam_model::rad_tan_distortion kc;   //Distortion parameters

    double config_fps;                  //Frames per second
    double soft_fps;                    //Forced frames per second
    bool useUndistort;                  //Use undistortion
    bool rotatedCam;                    //Rotate camera 180deg

    //simcam and simulation parameters
    std::string SimFile;                    //SimCam video file
    //Processor parameters

    int cpuSetAffinity;             //Switch to set afinity on off
    int cpu0;                       //Processor threads affiniy
    int cpu1;
    int cpu2;

    //Detector parameters

    double Sigma0;                  //The scale and multiplier used for DoG calculation (Sigma1~Sigma0*k_sigma)
    double KSigma;

    int DetectorPlaneFitSize;       //Window size for plane fitting to the DoG = (DetectorPlaneFitSize*2+1)^2
    double DetectorPosNegThresh;    //Max percentual difference for DoG nonmaximal suppresion
    double DetectorDoGThresh;       //Relation between DoG threshold and Gradient threshold ~1/Sigma0^4

    int ReferencePoints;            //Reference to the number of points when autothreshold
    int TrackPoints;                //Reference to the number of points used for track
    int MaxPoints;                  //Absolute maximun number of points

    double DetectorThresh;          //Manual theshold
    double DetectorAutoGain;        //Auto threshold gain, 0=manual
    double DetectorMaxThresh;       //Limits for autothreshold
    double DetectorMinThresh;


    //Tracker-Mapper Parameters
    double SearchRange;                      //Pixel range for tracking and mapping

    double QCutOffNumBins;                  //Number of bins on the histogram for percentile calculation
    double QCutOffQuantile;                 //Percentile of the KLs to use

    int TrackerIterNum;                     //Tracker number of iterations
    int TrackerInitIterNum;                 //Double ititialization iteration number
    int TrackerInitType;                    //Tracker Initialization prior (0=zero,1=last frame,2=try both)
    double TrackerMatchThresh;              //Tracking thesh on the scalar product
    double ReweigthDistance;                //Reweigh error hubber residual

    uint MatchNumThresh;                    //Minimun number of matches for tracking

};


//Structure to save variables of the different estimation stages

//Pipeline state buffer

struct PipeBuffer{

    sspace * ss;
    global_tracker *gt;
    edge_tracker *ef;
    Image<RGB24Pixel> *imgc;
    Image<float> *img;

    double t;
    double dt;

    double s_rho_p;

    double dtp0;
    double dtp1;

    double K;
    double Kp;
    double RKp;

    int p_id;
    bool quit;
};


//typedef bool outputCallback(PipeBuffer &data);


class REBVO
{

    REBVOParameters params;

    std::thread Thr0;
    bool InitOK=true;

    //Pipeline and multithead

    std::atomic_bool    quit;
    Pipeline <PipeBuffer> pipe;

    int snap_n=0;
    std::atomic_bool system_reset;


    std::atomic_bool frame_by_frame;
    std::atomic_bool frame_by_frame_advance;


    //Custom cam pipeline
    Pipeline <customCam::CustomCamPipeBuffer> cam_pipe;
    
    //camera model
    cam_model cam;

    std::mutex call_mutex;
    std::function<bool(PipeBuffer &)> outputFunc;

    bool callCallBack(PipeBuffer & pbuf){
        std::lock_guard<std::mutex> locker(call_mutex);

        if(outputFunc)
            return outputFunc(pbuf);
        return true;
    }

    void construct();

    VideoCam * initCamera();

    static bool setAffinity(int cpu){
        cpu_set_t cpusetp;

        CPU_ZERO(&cpusetp);
        CPU_SET(cpu, &cpusetp);
        return pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpusetp) == 0;
    }


public:
    bool	Init();
    static void ThirdThread(REBVO *cf);
    static void SecondThread(REBVO *cf);
    static void	FirstThr(REBVO *cf);

    bool CleanUp();
    REBVO(const char *configFile);
    REBVO(const REBVOParameters &parameters);
    ~REBVO();

    void Reset(){system_reset=true;}

    bool Running(){return !quit;}

    /**
     * @brief request a ptr to an Image object to push an image on the pipeline of the custom camera
     * @param ptr to store the buffer where to copy the data
     * @param time_stamp: image timestamp (must be sync with imu)
     * @param timeout_secs: waiting timeout
     * @return buffer taken correctly
     */
    bool requestCustomCamBuffer(std::shared_ptr<Image <RGB24Pixel> > &ptr,double time_stamp,double timeout_secs=0){

        customCam::CustomCamPipeBuffer *ccpb=cam_pipe.RequestBufferTimeoutable(0,timeout_secs);
        if(ccpb==nullptr)
            return false;
        ptr=(*ccpb).img;
        (*ccpb).timestamp=time_stamp;
        return true;
    }

    //release the pointer for image loading on the customcam buffer

    /**
     * @brief release the buffer after copy
     */
    void releaseCustomCamBuffer(){

        cam_pipe.ReleaseBuffer(0);
    }

    //request a ptr to an Image object to push image on the pipeline of the custom camera
    //set a callback funtion to be called on the third thread with a reference to the pipebuffer containing all the algorithm output
    //call with nullptr to release callback
    /**
     * @brief set a callback method to be called on the third thread with a reference to a pipebuffer containing all the algorithm output,
     * call with nullptr to release callback
     * @param method: method to bind to. Must take a PipeBuffer reference as argument.
     * @param obj: object
     */
    template <typename T>
    void setOutputCallback(bool(T::*method)(PipeBuffer &), T * obj){
        std::lock_guard<std::mutex> locker(call_mutex);
        outputFunc=std::bind(method,obj,std::placeholders::_1);
    }
    /**
     * @brief set a callback method to be called on the third thread with a reference to a pipebuffer containing all the algorithm output,
     * call with nullptr to release callback
     * @param func: function to bind to. Must take a PipeBuffer reference as argument.
     */

    void setOutputCallback(bool(*func)(PipeBuffer &) ){
        std::lock_guard<std::mutex> locker(call_mutex);
        outputFunc=std::bind(func,std::placeholders::_1);
    }

    /**
     * @brief Check is everything OK
     * @return
     */
	bool isInitOk() const {
		return InitOK;
    }
};

}

#endif // REBVO_H
