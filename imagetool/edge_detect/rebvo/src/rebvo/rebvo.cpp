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
#include "UtilLib/configurator.h"


int total=0;

using namespace std;
namespace  rebvo{


REBVO::REBVO(const char *configFile)
    :quit(true),pipe(CBUFSIZE,4),cam_pipe(CCAMBUFSIZE,2),outputFunc(nullptr)
{

    Configurator config;
    //Leo el archivo de configuracion
    if(!(InitOK=config.ParseConfigFile(configFile,false)))
        return;

    InitOK&=config.GetConfigByName("Camera","ImageWidth",params.ImageSize.w,true);
    InitOK&=config.GetConfigByName("Camera","ImageHeight",params.ImageSize.h,true);

    InitOK&=config.GetConfigByName("Camera","ZfX",params.z_f_x,true);
    InitOK&=config.GetConfigByName("Camera","ZfY",params.z_f_y,true);

    InitOK&=config.GetConfigByName("Camera","PPx",params.pp_x,true);
    InitOK&=config.GetConfigByName("Camera","PPy",params.pp_y,true);

    InitOK&=config.GetConfigByName("Camera","KcR2",params.kc.Kc2,true);
    InitOK&=config.GetConfigByName("Camera","KcR4",params.kc.Kc4,true);
    InitOK&=config.GetConfigByName("Camera","KcR6",params.kc.Kc6,true);
    InitOK&=config.GetConfigByName("Camera","KcP1",params.kc.P1,true);
    InitOK&=config.GetConfigByName("Camera","KcP2",params.kc.P2,true);
    InitOK&=config.GetConfigByName("Camera","UseUndistort",params.useUndistort,false);
    InitOK&=config.GetConfigByName("Camera","Rotate180",params.rotatedCam,true);

    InitOK&=config.GetConfigByName("Camera","FPS",params.config_fps,true);
    if(!config.GetConfigByName("Camera","SoftFPS",params.soft_fps,true))
    params.soft_fps=params.config_fps;

    InitOK&=config.GetConfigByName("ProcesorConfig","SetAffinity",params.cpuSetAffinity,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT1",params.cpu0,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT2",params.cpu1,true);
    InitOK&=config.GetConfigByName("ProcesorConfig","CamaraT3",params.cpu2,true);

    InitOK&=config.GetConfigByName("Detector","Sigma0",params.Sigma0,true);
    InitOK&=config.GetConfigByName("Detector","KSigma",params.KSigma,true);
    InitOK&=config.GetConfigByName("Detector","ReferencePoints",params.ReferencePoints,true);
    InitOK&=config.GetConfigByName("Detector","MaxPoints",params.MaxPoints,true);
    InitOK&=config.GetConfigByName("Detector","TrackPoints",params.TrackPoints,true);
    InitOK&=config.GetConfigByName("Detector","DetectorThresh",params.DetectorThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorAutoGain",params.DetectorAutoGain,true);

    InitOK&=config.GetConfigByName("Detector","DetectorMaxThresh",params.DetectorMaxThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorMinThresh",params.DetectorMinThresh,true);

    InitOK&=config.GetConfigByName("Detector","DetectorPlaneFitSize",params.DetectorPlaneFitSize,true);
    InitOK&=config.GetConfigByName("Detector","DetectorPosNegThresh",params.DetectorPosNegThresh,true);
    InitOK&=config.GetConfigByName("Detector","DetectorDoGThresh",params.DetectorDoGThresh,true);

    InitOK&=config.GetConfigByName("TrackMaper","SearchRange",params.SearchRange,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffNumBins",params.QCutOffNumBins,true);
    InitOK&=config.GetConfigByName("TrackMaper","QCutOffQuantile",params.QCutOffQuantile,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerIterNum",params.TrackerIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitIterNum",params.TrackerInitIterNum,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerInitType",params.TrackerInitType,true);
    InitOK&=config.GetConfigByName("TrackMaper","TrackerMatchThresh",params.TrackerMatchThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","MatchNumThresh",params.MatchNumThresh,true);
    InitOK&=config.GetConfigByName("TrackMaper","ReweigthDistance",params.ReweigthDistance,true);


    cam=cam_model({params.pp_x,params.pp_y},{params.z_f_x,params.z_f_y},params.kc,params.ImageSize);
    construct();
}

REBVO::REBVO(const REBVOParameters &parameters)
    :params(parameters),quit(true),pipe(CBUFSIZE,4),
      cam_pipe(CCAMBUFSIZE,2),
      cam({params.pp_x,params.pp_y},{params.z_f_x,params.z_f_y},params.kc,params.ImageSize),
      outputFunc(nullptr)
{
    construct();
}

void REBVO::construct(){

    //***** Imu grabber contruct (if necesary) *****//
    for(customCam::CustomCamPipeBuffer &pb:cam_pipe)
        pb.img=std::shared_ptr<Image<RGB24Pixel> >(new Image<RGB24Pixel>(params.ImageSize));

    //***** PipeLine init ******
    //Initializes every object in the pipeline buffer
    //for exchange between threads

    for(PipeBuffer &pbuf : pipe){

        pbuf.ss=new sspace(params.Sigma0,params.KSigma,cam.sz,3);
        pbuf.ef=new edge_tracker(cam,255*3);
        pbuf.gt=new global_tracker (pbuf.ef->GetCam());
        pbuf.img=new Image<float>(cam.sz);
        pbuf.imgc=new Image<RGB24Pixel>(cam.sz);
        pbuf.t=0;
    }
    return;
}
bool REBVO::Init(){

    if(!InitOK)
        return false;

    system_reset=false;
    quit=false;
    frame_by_frame=false;
    frame_by_frame_advance=false;
    Thr0=std::thread (FirstThr,this);
    cout<<"rebvo init ok!=================="<<endl;
    return true;

}

bool REBVO::CleanUp(){

    quit=true;
    Thr0.join();
    return true;
}


REBVO::~REBVO(){

    if(!quit)
        CleanUp();

    for(PipeBuffer &pbuf : pipe){

        delete pbuf.ss;
        delete pbuf.ef;
        delete pbuf.gt;
        delete pbuf.img;
        delete pbuf.imgc;
    }
}

}
