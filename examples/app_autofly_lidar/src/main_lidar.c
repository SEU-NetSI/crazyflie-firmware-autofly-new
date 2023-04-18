#include <string.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "param.h"
#include "range.h"
#include "log.h"
#include <stdbool.h>
#include "radiolink.h"
#include "configblock.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"

#define DEBUG_PRINT_ENABLED 1
#define TIMEOUT 500

enum {
    LISTEN_STATE, // 监听状态
    SEND_STATE, // 发送状态
    WAIT_STATE // 等待状态
} state = LISTEN_STATE;

uint32_t last_send_time = 0; // 上一次发送的时间戳
bool response_received = false; // 是否收到响应
static uint16_t seq = 0; // 序列号
static uint16_t respSeq = 0; // 响应序列号

// P2P监听
void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("[LiDAR-STM32]Callback called!");

    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t respType = p->data[1];
    respSeq = p->data[2];
    
    //状态切换
    if(respSeq == seq && state==WAIT_STATE){
        response_received = true;
    }

    static coordinate_t responsePayload[RESPONSE_PAYLOAD_LENGTH];
    memcpy(responsePayload, &p->data[3], sizeof(coordinate_t) * RESPONSE_PAYLOAD_LENGTH);
    // TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[LiDAR-STM32]Receive P2P response from: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", sourceId, rssi, respType, respSeq);
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

// handle mapping request
mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
uint8_t mappingRequestPayloadCur = 0;
uint16_t mappingRequestSeq = 0;

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
        if (isSameNode(startPoint, &mappingRequestPayload[i].coordinatePair.startPoint) && 
            isSameNode(endPoint, &mappingRequestPayload[i].coordinatePair.endPoint))
        {
            mappingRequestPayload[i].mergedNums++;
            if (mappingRequestPayload[i].mergedNums < (LOG_ODDS_OCCUPIED - LOG_ODDS_FREE) / LOG_ODDS_DIFF_STEP) {
                return;
            }
        }
    }

    // append coordinatePair to payload
    mappingRequestPayload[mappingRequestPayloadCur].coordinatePair.startPoint = *startPoint;
    mappingRequestPayload[mappingRequestPayloadCur].coordinatePair.endPoint = *endPoint;
    mappingRequestPayload[mappingRequestPayloadCur].mergedNums = 1;
    mappingRequestPayloadCur++;

    if (mappingRequestPayloadCur >= MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT || mappingRequestPayloadCur >= payloadLengthAdaptive)
    {
        // package explore request
        mappingRequestSeq++;
        bool flag = sendMappingRequest(mappingRequestPayload, mappingRequestPayloadCur, mappingRequestSeq);
        mappingRequestPayloadCur = 0;

        // print debug info
        DEBUG_PRINT("[LiDAR-STM32]Send mapping request %s, seq: %d, payloadLength: %d\n", 
            flag == false ? "Failed" : "Successfully", mappingRequestSeq, mappingRequestPayloadCur);
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[LiDAR-STM32]Mapping request payload: \n");
            for (int i = 0; i < mappingRequestPayloadCur; i++)
            {
                DEBUG_PRINT("[LiDAR-STM32]Coordinate pair %d: (%d, %d, %d), (%d, %d, %d), mergedNums: %d\n", 
                    i, 
                    mappingRequestPayload[i].coordinatePair.startPoint.x, 
                    mappingRequestPayload[i].coordinatePair.startPoint.y, 
                    mappingRequestPayload[i].coordinatePair.startPoint.z,
                    mappingRequestPayload[i].coordinatePair.endPoint.x, 
                    mappingRequestPayload[i].coordinatePair.endPoint.y, 
                    mappingRequestPayload[i].coordinatePair.endPoint.z,
                    mappingRequestPayload[i].mergedNums);
                vTaskDelay(50);
            }
        }
    }
}

// handle explore request
uint16_t exploreRequestSeq = 0;
void setExploreRequestPayload(coordinate_t* startPoint, example_measure_t* measurement)
{
    // package explore request
    exploreRequestSeq++;
    explore_req_payload_t exploreRequestPayload = {*startPoint, *measurement};
    bool flag = sendExploreRequest(&exploreRequestPayload, exploreRequestSeq);

    // print debug info
    DEBUG_PRINT("[LiDAR-STM32]Send explore request %s, seq: %d\n", 
        flag == false ? "Failed" : "Successfully", exploreRequestSeq);
    if (DEBUG_PRINT_ENABLED)
    {
        DEBUG_PRINT("[LiDAR-STM32]Explore request payload: startPoint: (%d, %d, %d), measurement: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)\n", 
            exploreRequestPayload.startPoint.x, exploreRequestPayload.startPoint.y, exploreRequestPayload.startPoint.z, 
            (double)exploreRequestPayload.measurement.data[0], (double)exploreRequestPayload.measurement.data[1], (double)exploreRequestPayload.measurement.data[2],
            (double)exploreRequestPayload.measurement.data[3], (double)exploreRequestPayload.measurement.data[4], (double)exploreRequestPayload.measurement.data[5],
            (double)exploreRequestPayload.measurement.yaw, (double)exploreRequestPayload.measurement.pitch, (double)exploreRequestPayload.measurement.roll);
        vTaskDelay(50);
    }
}

// 自动状态机
void state_machine_task() {
    while(1) {
        switch(state) {
            case LISTEN_STATE:
            vTaskDelay(1000);
            DEBUG_PRINT("I'm listening!\n");
            ListeningInit();
            state = SEND_STATE; // 转为发送状态，等待下一次触发
            // action
            break;
            case SEND_STATE:
                vTaskDelay(1000);
                DEBUG_PRINT("I'm sending!\n");
                seq++;
                //Send();
                last_send_time = xTaskGetTickCount(); // 记录发送时间
                state = WAIT_STATE;
                break;
            case WAIT_STATE:
                vTaskDelay(1000);
                DEBUG_PRINT("I'm waiting!\n");
                if(response_received) { // 如果收到响应，转为监听状态
                    state = LISTEN_STATE;
                    response_received = false;
                } else if(xTaskGetTickCount() - last_send_time > TIMEOUT) { // 如果超时，重新发送
                    state = SEND_STATE;
                }
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 避免CPU占用率过高
        }
    }

    void appMain(){
        xTaskCreate(state_machine_task, "state_machine_task", 2*configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        state=LISTEN_STATE;
    }
// void appMain()
// {
//     vTaskDelay(M2T(10000));
//     example_measure_t measurement;
//     coordinate_t startPoint = {OFFSET_X, OFFSET_Y, OFFSET_Z};
//     coordinateF_t startPointF = {OFFSET_X, OFFSET_Y, OFFSET_Z};
//     coordinate_t endPoint;
//     coordinateF_t endPointF;
    
//     // circularly get measurement and send to edge-computing uav
//     while (1) 
//     {
//         vTaskDelay(M2T(1000));
//         // set start point
//         startPointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
//         startPointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
//         startPointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
//         startPoint.x = startPointF.x;
//         startPoint.y = startPointF.y;
//         startPoint.z = startPointF.z;
        
//         // set measurement
//         get_measurement(&measurement);
//         if (startPointF.z < TOP) {
//             measurement.data[4] = TOP - startPointF.z;
//         } else {
//             measurement.data[4] = 0;
//         }
//         if (startPointF.z > BOTTOM) {
//             measurement.data[5] = startPointF.z - BOTTOM;
//         } else {
//             measurement.data[5] = 0;
//         }
        
//         // set end point
//         for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
//         {
//             if (cal_Point(&measurement, &startPointF, dir, &endPointF))
//             {
//                 endPoint.x = endPointF.x;
//                 endPoint.y = endPointF.y;
//                 endPoint.z = endPointF.z;

//                 // add (startPoint, endPoint) to mappingRequestPayload
//                 appendMappingRequestPayload(&startPoint, &endPoint, MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC);
//             }
//         }
//     }
// }