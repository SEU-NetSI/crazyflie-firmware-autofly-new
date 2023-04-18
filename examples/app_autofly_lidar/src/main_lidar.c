#include <string.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "param.h"
#include "range.h"
#include "log.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "auxiliary_tool.h"
#include "config_autofly.h"
#include "communicate.h"

// handle mapping request
coordinate_pair_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH];
uint8_t mappingRequestPayloadCur = 0;
uint16_t mappingRequestSeq = 0;
void appendMappingRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint)
{
    mappingRequestPayload[mappingRequestPayloadCur].startPoint = *startPoint;
    mappingRequestPayload[mappingRequestPayloadCur].endPoint = *endPoint;
    mappingRequestPayloadCur++;

    if (mappingRequestPayloadCur == MAPPING_REQUEST_PAYLOAD_LENGTH)
    {
        mappingRequestSeq++;
        bool flag = sendMappingRequest(mappingRequestPayload, MAPPING_REQUEST_PAYLOAD_LENGTH, mappingRequestSeq);
        mappingRequestPayloadCur = 0;
        DEBUG_PRINT("[STM32-LiDAR]Send mapping request %s, seq: %d, payloadLength: %d\n",flag == false ? "Failed" : "Successfully", mappingRequestSeq, MAPPING_REQUEST_PAYLOAD_LENGTH);
        DEBUG_PRINT("[STM32-LiDAR]First coordinate pair: (%d, %d, %d), (%d, %d, %d)\n", 
            mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
            mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);
    }
}

// handle explore request
uint16_t exploreRequestSeq = 0;
void setExploreRequestPayload(coordinate_t* startPoint)
{
    exploreRequestSeq++;
    bool flag = sendExploreRequest(startPoint, exploreRequestSeq);
    DEBUG_PRINT("[STM32-LiDAR]Send explore request %s, seq: %d\n",flag == false ? "Failed" : "Successfully", exploreRequestSeq);
    DEBUG_PRINT("[STM32-LiDAR]startPoint coordinate: (%d, %d, %d)\n", startPoint[0].x, startPoint[0].y, startPoint[0].z);
}

// handle path request
uint16_t pathRequestSeq = 0;
void setPathRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint)
{
    pathRequestSeq++;
    coordinate_pair_t pathRequestPayload;
    pathRequestPayload.startPoint = *startPoint;
    pathRequestPayload.endPoint = *endPoint;
    bool flag = sendPathRequest(&pathRequestPayload, pathRequestSeq);
    DEBUG_PRINT("[STM32-LiDAR]Send path request %s, seq: %d\n",flag == false ? "Failed" : "Successfully", pathRequestSeq);
    DEBUG_PRINT("[STM32-LiDAR]startPoint: (%d, %d, %d), endPoint: (%d, %d, %d)\n", 
        startPoint[0].x, startPoint[0].y, startPoint[0].z, 
        endPoint[0].x, endPoint[0].y, endPoint[0].z);
}

void appMain()
{
    vTaskDelay(M2T(10000));
    example_measure_t measurement;
    coordinate_t startPoint = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinateF_t startPointF = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinate_t endPoint;
    coordinateF_t endPointF;
    
    // circularly get measurement and send to edge-computing uav
    while (1) 
    {
        vTaskDelay(M2T(1000));
        // set start point
        startPointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
        startPointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
        startPointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
        startPoint.x = startPointF.x;
        startPoint.y = startPointF.y;
        startPoint.z = startPointF.z;
        
        // set measurement
        get_measurement(&measurement);
        if (startPointF.z < TOP) {
            measurement.data[4] = TOP - startPointF.z;
        } else {
            measurement.data[4] = 0;
        }
        if (startPointF.z > BOTTOM) {
            measurement.data[5] = startPointF.z - BOTTOM;
        } else {
            measurement.data[5] = 0;
        }
        
        // set end point
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            if (cal_Point(&measurement, &startPointF, dir, &endPointF))
            {
                endPoint.x = endPointF.x;
                endPoint.y = endPointF.y;
                endPoint.z = endPointF.z;

                // add (startPoint, endPoint) to mappingRequestPayload
                appendMappingRequestPayload(&startPoint, &endPoint);
            }
        }
    }
}