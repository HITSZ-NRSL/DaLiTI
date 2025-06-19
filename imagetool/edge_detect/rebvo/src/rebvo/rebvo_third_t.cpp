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



using namespace std;

namespace  rebvo{

void REBVO::ThirdThread(REBVO *cf){
    bool quit=false;
    /*****   Set cpu Afinity of the thread   ******/

    if(cf->params.cpuSetAffinity){
        if(!REBVO::setAffinity(cf->params.cpu2)){
            std::cout <<"REBVO: Cannot set cpu affinity on the third thread";
            cf->quit=true;
        }
    }

    /****** Init buffer for pla video save (no container) if needed ******/
    while(!cf->quit && !quit){

        PipeBuffer pbuf=cf->pipe.RequestBuffer(3);          //Request buffer for player 3

        if(pbuf.quit){
            cf->pipe.ReleaseBuffer(3);
            break;
        }
        cf->callCallBack(pbuf);
        cf->pipe.ReleaseBuffer(3);

    }
    cf->quit=true;

    return;


}

}
