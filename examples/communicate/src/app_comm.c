//
// Created by mfxjx on 2023/3/29.
//

#include "communicate.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "communicate.h"
#include "aideck.h"
#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"


#include "debug.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "cpx.h"
#include "crtp.h"



void appMain(){

    vTaskDelay(M2T(20000));
    //create test coords
    coordinate_t coords[10];
    DEBUG_PRINT("Coords created\n");
    for(int i=0;i<10;i++){
        coords[i].x=i;
        coords[i].y=i+1;
        coords[i].z=i+2;
    }
    //cpx send
    //cpxInit();
    cpxInternalRouterInit();
    cpxExternalRouterInit();
    while(1)
    {
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32,CPX_T_GAP8,CPX_F_APP,&cpxPacket.route);
        cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
        memcpy(cpxPacket.data, coords, cpxPacket.dataLength);
        bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
        //vTaskDelay(M2T(30000));
        DEBUG_PRINT("Send %s\n",flag==false?"timeout":"success");
        
    }
    

    
}