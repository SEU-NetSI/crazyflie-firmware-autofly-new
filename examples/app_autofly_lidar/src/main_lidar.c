#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"

#include <string.h>
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"
#define COORDS_LENGTH 5

void appMain()
{
    //create test coords
    coordinate_t coords1[5];
    for(int i=0;i<5;i++){
        coords1[i].x=i;
        coords1[i].y=i+1;
        coords1[i].z=i+2;
    }
    coordinate_t coords2[2];
    for(int i=0;i<2;i++){
        coords2[i].x=i+1;
        coords2[i].y=i+2;
        coords2[i].z=i+3;
    }
    coordinate_t coord3;
    coord3.x=3;
    coord3.y=4;
    coord3.z=5;
    bool flag=0;
    uint16_t seq=0;
    while(1){
        flag=SendReq(coords1,MappingReq,seq);
        DEBUG_PRINT("sent MappingReq %s\n",flag==false?"Failed":"Success");
        seq++;
        vTaskDelay(M2T(1000));
        flag=SendReq(coords2,PathReq,seq);
        DEBUG_PRINT("sent PathReq %s\n",flag==false?"Failed":"Success");
        seq++;
        vTaskDelay(M2T(1000));
        flag=SendReq(&coord3,ExploreReq,seq);
        DEBUG_PRINT("sent ExploreReq %s\n",flag==false?"Failed":"Success");
        seq++;
        vTaskDelay(M2T(1000));
    }
}