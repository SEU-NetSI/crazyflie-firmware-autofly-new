#include <string.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "param.h"
#include "range.h"
#include "log.h"

#include "radiolink.h"
#include "configblock.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"
#include "measure_tool.h"
#include "config_autofly.h"
#include "crtp_commander_high_level.h"

#define DEBUG_PRINT_ENABLED 1

// handle mapping request
mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
static explore_resp_payload_t responsePayload;
bool flag_explore = true;
uint8_t mappingRequestPayloadCur = 0;
uint16_t mappingRequestSeq = 0;
uint16_t exploreRequestSeq = 0;

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    // uint8_t rssi = p->rssi;
    explore_resp_packet_t exploreRespPacket;
    memcpy(&exploreRespPacket, &p->data, sizeof(explore_resp_packet_t));
    if (exploreRespPacket.destinationId != getSourceId() || exploreRespPacket.packetType != EXPLORE_RESP || exploreRespPacket.seq != exploreRequestSeq)
    {
        return;
    }
    // DEBUG_PRINT("[LiDAR-STM32]P2P: Receive explore response from: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", 
    //     exploreRespPacket.sourceId, rssi, exploreRespPacket.packetType, exploreRespPacket.seq);
    memcpy(&responsePayload, &p->data[5], sizeof(explore_resp_payload_t));
    flag_explore = true;
    // TODO:使用 xQueueSend 来将下一步坐标存入队列
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool isSameNode(coordinate_t* coordinate1, coordinate_t* coordinate2)
{
    return (coordinate1->x / TREE_RESOLUTION == coordinate2->x / TREE_RESOLUTION) && 
        (coordinate1->y / TREE_RESOLUTION == coordinate2->y / TREE_RESOLUTION) && 
        (coordinate1->z / TREE_RESOLUTION == coordinate2->z / TREE_RESOLUTION);
}

void appendMappingRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint, uint8_t payloadLengthAdaptive)
{
    // merge coordinatePair
    for (int i = 0; i < mappingRequestPayloadCur; i++)
    {
        if (isSameNode(startPoint, &mappingRequestPayload[i].startPoint) && 
            isSameNode(endPoint, &mappingRequestPayload[i].endPoint))
        {
            mappingRequestPayload[i].mergedNums++;
            if (mappingRequestPayload[i].mergedNums < (LOG_ODDS_OCCUPIED - LOG_ODDS_FREE) / LOG_ODDS_DIFF_STEP) {
                return;
            }
        }
    }

    // append coordinatePair to payload
    mappingRequestPayload[mappingRequestPayloadCur].startPoint = *startPoint;
    mappingRequestPayload[mappingRequestPayloadCur].endPoint = *endPoint;
    mappingRequestPayload[mappingRequestPayloadCur].mergedNums = 1;
    mappingRequestPayloadCur++;

    if (mappingRequestPayloadCur >= MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT || mappingRequestPayloadCur >= payloadLengthAdaptive)
    {
        // package explore request
        mappingRequestSeq++;
        bool flag = sendMappingRequest(mappingRequestPayload, mappingRequestPayloadCur, mappingRequestSeq);
        mappingRequestPayloadCur = 0;

        // print debug info
        DEBUG_PRINT("[LiDAR-STM32]P2P: Send mapping request %s, seq: %d, payloadLength: %d\n", 
            flag == false ? "Failed" : "Successfully", mappingRequestSeq, mappingRequestPayloadCur);
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[LiDAR-STM32]P2P: Mapping request payload: \n");
            for (int i = 0; i < mappingRequestPayloadCur; i++)
            {
                DEBUG_PRINT("[LiDAR-STM32]P2P: coordinatePair %d: (%d, %d, %d), (%d, %d, %d), mergedNums: %d\n", 
                    i, 
                    mappingRequestPayload[i].startPoint.x, 
                    mappingRequestPayload[i].startPoint.y, 
                    mappingRequestPayload[i].startPoint.z,
                    mappingRequestPayload[i].endPoint.x, 
                    mappingRequestPayload[i].endPoint.y, 
                    mappingRequestPayload[i].endPoint.z,
                    mappingRequestPayload[i].mergedNums);
                vTaskDelay(50);
            }
            DEBUG_PRINT("\n");
        }
    }
}

void MoveTo(float x, float y, float z)
{   
    crtpCommanderHighLevelGoTo((double)(x - OFFSET_X) / 100, (double)(y - OFFSET_Y) / 100, (double)(z - OFFSET_Z) / 100, 0, 0.5, 0);
    vTaskDelay(M2T(MOVE_DELAY));
}

// handle explore request
void setExploreRequestPayload(coordinate_t* startPoint, example_measure_t* measurement)
{
    // package explore request
    exploreRequestSeq++;
    explore_req_payload_t exploreRequestPayload = {*startPoint, *measurement};
    bool flag = sendExploreRequest(&exploreRequestPayload, exploreRequestSeq);

    // print debug info
    DEBUG_PRINT("[LiDAR-STM32]P2P: Send explore request %s, seq: %d\n", 
        flag == false ? "Failed" : "Successfully", exploreRequestSeq);
    if (DEBUG_PRINT_ENABLED)
    {
        DEBUG_PRINT("[Edge-STM32]P2P: Explore request payload: \n");
        DEBUG_PRINT("[Edge-STM32]P2P: startPoint: (%d, %d, %d)\n", 
            exploreRequestPayload.startPoint.x, 
            exploreRequestPayload.startPoint.y, 
            exploreRequestPayload.startPoint.z);
        DEBUG_PRINT("[Edge-STM32]P2P: data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
            (double)exploreRequestPayload.measurement.data[0], 
            (double)exploreRequestPayload.measurement.data[1], 
            (double)exploreRequestPayload.measurement.data[2], 
            (double)exploreRequestPayload.measurement.data[3], 
            (double)exploreRequestPayload.measurement.data[4], 
            (double)exploreRequestPayload.measurement.data[5]);
        DEBUG_PRINT("[Edge-STM32]P2P: roll: %.2f, pitch: %.2f, yaw: %.2f\n\n", 
            (double)exploreRequestPayload.measurement.roll, 
            (double)exploreRequestPayload.measurement.pitch, 
            (double)exploreRequestPayload.measurement.yaw);
        vTaskDelay(50);
    }
}

void setMapping(coordinateF_t* currentF, example_measure_t* measurement, uint8_t payloadLengthAdaptive){
    coordinate_t end_point,start_point;
    coordinateF_t item_end;
    start_point.x = (int)(currentF->x);
    start_point.y = (int)(currentF->y);
    start_point.z = (int)(currentF->z);
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(measurement, currentF, dir, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            appendMappingRequestPayload(&start_point, &end_point, payloadLengthAdaptive);
        }
    }
}

void appMain()
{
    vTaskDelay(M2T(10000));
    coordinate_t start_pointI;
    coordinateF_t start_pointF;
    example_measure_t measurement;
    TickType_t time = xTaskGetTickCount();
    while(1){
        vTaskDelay(M2T(100));
        get_Current_point(&start_pointF);
        get_measurement(&measurement,&start_pointF);
        start_pointI.x = (int)(start_pointF.x);
        start_pointI.y = (int)(start_pointF.y);
        start_pointI.z = (int)(start_pointF.z);
        if(flag_explore){
            MoveTo((float)responsePayload.endPoint.x, (float)responsePayload.endPoint.y, (float)responsePayload.endPoint.z);
            flag_explore = false;
            get_Current_point(&start_pointF);
            get_measurement(&measurement,&start_pointF);
            start_pointI.x = (int)(start_pointF.x);
            start_pointI.y = (int)(start_pointF.y);
            start_pointI.z = (int)(start_pointF.z);
            setExploreRequestPayload(&start_pointI, &measurement);
            time = xTaskGetTickCount();
        }
        else if(xTaskGetTickCount() - time >= 500){
            // 超时重发
            setExploreRequestPayload(&start_pointI, &measurement);
            time = xTaskGetTickCount();
        }
        setMapping(&start_pointF, &measurement, MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT);
    }
}
