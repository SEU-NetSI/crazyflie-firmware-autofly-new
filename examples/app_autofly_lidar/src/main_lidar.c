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
#define MAX_EXPLORE 100
#define MAX_MAPPING 500
// mappingReq and exploreResp cache
uint8_t mappingRequestPayloadCur = 0;
mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
bool canExplore = FALSE;
static explore_resp_payload_t responsePayload;
// seq for each packet type
uint16_t mappingRequestSeq = 0;
uint16_t exploreRequestSeq = 0;
uint16_t metricsRequestSeq = 0;
// counter for each packet type
uint16_t mappingRequestCount = 0;
uint16_t exploreRequestCount = 0;
uint16_t exploreResponseCount = 0;

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    explore_resp_packet_t exploreRespPacket;
    memcpy(&exploreRespPacket, &p->data, sizeof(explore_resp_packet_t));

    // Ignore unexpected packet
    if (exploreRespPacket.destinationId != getSourceId()) return;
    if (exploreRespPacket.packetType != EXPLORE_RESP) {
        // DEBUG_PRINT("[LiDAR-STM32]P2P: Receive packet type: %d, ignore it\n", 
        //     exploreRespPacket.packetType);
        return;
    }
    if (exploreRespPacket.seq != exploreRequestSeq) {
        // DEBUG_PRINT("[LiDAR-STM32]P2P: Receive explore response seq: %d, expected: %d, ignore it\n", 
        //     exploreRespPacket.seq, 
        //     exploreRequestSeq);
        return;
    }
    exploreResponseCount++;
    responsePayload = exploreRespPacket.exploreResponsePayload;
    // memcpy(&responsePayload, &p->data[5], sizeof(explore_resp_payload_t));

    // Print debug info
    DEBUG_PRINT("[LiDAR-STM32]P2P: Receive explore response from: %d, to: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", 
        exploreRespPacket.sourceId, 
        exploreRespPacket.destinationId, 
        rssi, 
        exploreRespPacket.packetType, 
        exploreRespPacket.seq
        );
    DEBUG_PRINT("endPoint: (%d, %d, %d)\n\n", 
        exploreRespPacket.exploreResponsePayload.endPoint.x, 
        exploreRespPacket.exploreResponsePayload.endPoint.y, 
        exploreRespPacket.exploreResponsePayload.endPoint.z
        );
    if (DEBUG_PRINT_ENABLED)
    {
        DEBUG_PRINT("[LiDAR-STM32]P2P: Explore response payload: \n");
        DEBUG_PRINT("[LiDAR-STM32]P2P: endPoint: (%d, %d, %d)\n\n", 
            responsePayload.endPoint.x, 
            responsePayload.endPoint.y, 
            responsePayload.endPoint.z);
    }

    // Set explore available flag
    canExplore = TRUE;
}

void ListeningInit()
{
    DEBUG_PRINT("[LiDAR-STM32]P2P: Listening init\n");
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
        if (!flag) {
            DEBUG_PRINT("[LiDAR-STM32]P2P: Send mapping request failed\n");
        }
        mappingRequestCount++;
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
                vTaskDelay(M2T(DELAY_PRINT));
            }
            DEBUG_PRINT("\n");
        }
    }
}

void MoveTo(float x, float y, float z)
{   
    crtpCommanderHighLevelGoTo((double)(x - OFFSET_X) / 100, (double)(y - OFFSET_Y) / 100, (double)(z - OFFSET_Z) / 100, 0, 0.5, 0);
    // unset explore available flag
    canExplore = FALSE;
    vTaskDelay(M2T(DELAY_MOVE));
}

// handle explore request
void setExploreRequestPayload(coordinate_t* startPoint, example_measure_t* measurement, bool flag_timeout)
{
    // package explore request
    if (!flag_timeout)
        exploreRequestSeq++;
    explore_req_payload_t exploreRequestPayload = {*startPoint, *measurement};
    bool flag = sendExploreRequest(&exploreRequestPayload, exploreRequestSeq);
    exploreRequestCount++;
    if (!flag) {
        DEBUG_PRINT("[LiDAR-STM32]P2P: Send explore request failed\n");
    }
    if (DEBUG_PRINT_ENABLED)
    {
        DEBUG_PRINT("[LiDAR-STM32]P2P: Explore request payload: \n");
        DEBUG_PRINT("[LiDAR-STM32]P2P: startPoint: (%d, %d, %d)\n", 
            exploreRequestPayload.startPoint.x, 
            exploreRequestPayload.startPoint.y, 
            exploreRequestPayload.startPoint.z);
        DEBUG_PRINT("[LiDAR-STM32]P2P: data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
            (double)exploreRequestPayload.measurement.data[0], 
            (double)exploreRequestPayload.measurement.data[1], 
            (double)exploreRequestPayload.measurement.data[2], 
            (double)exploreRequestPayload.measurement.data[3], 
            (double)exploreRequestPayload.measurement.data[4], 
            (double)exploreRequestPayload.measurement.data[5]);
        DEBUG_PRINT("[LiDAR-STM32]P2P: roll: %.2f, pitch: %.2f, yaw: %.2f\n\n", 
            (double)exploreRequestPayload.measurement.roll, 
            (double)exploreRequestPayload.measurement.pitch, 
            (double)exploreRequestPayload.measurement.yaw);
        vTaskDelay(M2T(DELAY_PRINT));
    }
}

void setMetricsRequestPayload()
{
    metrics_req_payload_t metricsRequestPayload = { mappingRequestCount, exploreRequestCount, exploreResponseCount };
    metricsRequestSeq++;
    bool flag = sendMetricsRequest(&metricsRequestPayload, metricsRequestSeq);
    DEBUG_PRINT("[LiDAR-STM32]P2P: Send metrics request %s\n", flag == false ? "Failed" : "Successfully");
    DEBUG_PRINT("[LiDAR-STM32]mReqC: %d, eReqC: %d, eRespC: %d\n", 
        mappingRequestCount, exploreRequestCount, exploreResponseCount);
}

void setMapping(coordinateF_t* currentF, example_measure_t* measurement, uint8_t payloadLengthAdaptive){
    coordinate_t end_point, start_point;
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
    vTaskDelay(M2T(DELAY_TAKEOFF));
    crtpCommanderHighLevelTakeoff(0.4, 2.0);
    vTaskDelay(M2T(DELAY_START));
    coordinate_t start_pointI;
    coordinateF_t start_pointF;
    example_measure_t measurement;
    TickType_t time = xTaskGetTickCount();
    ListeningInit();
    while (1) 
    {
        vTaskDelay(M2T(DELAY_MAPPING));
        // get explore request payload
        get_Current_point(&start_pointF);
        get_measurement(&measurement,&start_pointF);
        start_pointI.x = (int)(start_pointF.x);
        start_pointI.y = (int)(start_pointF.y);
        start_pointI.z = (int)(start_pointF.z);

        if (exploreRequestSeq > MAX_EXPLORE){
            crtpCommanderHighLevelLand(0, 0.5);
            vTaskDelay(M2T(DELAY_MOVE));
            setMetricsRequestPayload();
            vTaskDelay(M2T(DELAY_PRINT));
            setMetricsRequestPayload();
            vTaskDelay(M2T(DELAY_PRINT));
            setMetricsRequestPayload();
            vTaskDelay(M2T(DELAY_PRINT));
            setMetricsRequestPayload();
            vTaskDelay(M2T(DELAY_PRINT));
            setMetricsRequestPayload();
            vTaskDelay(M2T(DELAY_PRINT));
            break;
        }
        if (canExplore)
        {
            MoveTo((float)responsePayload.endPoint.x, (float)responsePayload.endPoint.y, (float)responsePayload.endPoint.z);
            // get explore request payload
            get_Current_point(&start_pointF);
            get_measurement(&measurement, &start_pointF);
            start_pointI.x = (int)(start_pointF.x);
            start_pointI.y = (int)(start_pointF.y);
            start_pointI.z = (int)(start_pointF.z);
            setExploreRequestPayload(&start_pointI, &measurement, false);
            canExplore = false;
            // reset time
            time = xTaskGetTickCount();
        }
        // Not receive explore response and timeout
        else if (xTaskGetTickCount() - time >= TIMEOUT_EXPLORE_RESP) 
        {
            setExploreRequestPayload(&start_pointI, &measurement,true);
            canExplore = false;
            // reset time
            time = xTaskGetTickCount();
        }
        setMapping(&start_pointF, &measurement, MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT);
        // set mapping request payload
    }
}


// packet loss experiment
// Remember to clost other directions' measurement except front
// void appMain()
// {
//     vTaskDelay(M2T(DELAY_START));
//     coordinateF_t start_pointF;
//     example_measure_t measurement;
//     TickType_t time = xTaskGetTickCount();
//     ListeningInit();
//     while (1) 
//     {
//         vTaskDelay(M2T(DELAY_MAPPING));
//         // get explore request payload
//         get_Current_point(&start_pointF);
//         get_measurement(&measurement, &start_pointF);

//         if (mappingRequestSeq > MAX_MAPPING){
//             vTaskDelay(M2T(DELAY_MOVE));
//             setMetricsRequestPayload();
//             vTaskDelay(M2T(DELAY_PRINT));
//             setMetricsRequestPayload();
//             vTaskDelay(M2T(DELAY_PRINT));
//             setMetricsRequestPayload();
//             vTaskDelay(M2T(DELAY_PRINT));
//             setMetricsRequestPayload();
//             vTaskDelay(M2T(DELAY_PRINT));
//             setMetricsRequestPayload();
//             vTaskDelay(M2T(DELAY_PRINT));
//             break;
//         }
//         // set mapping request payload
//         setMapping(&start_pointF, &measurement, MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT);
//     }
// }
