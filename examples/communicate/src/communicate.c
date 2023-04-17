//
// Created by chaihu on 3/29/23.
// Communicate test in CF side
/*
 *  The process of Crazyflie Communication Example
 *  Experiment1:
 *  Crazyflie   -----(CPX)----->    AIdeck
 *  Crazyflie   <-----(CPX)-----    AIdeck
 *
 * */
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

#define DEBUG_MODULE "P2P"
#include "debug.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "cpx.h"
#define SIZE 3
#define COORDS_LENGTH 10
#define MSG_LENGTH 10

// static CPXPacket_t cpxRx;

void MtrP2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("Callback called!");
    uint8_t other_id = p->data[0];
    static coordinate_t msg[COORDS_LENGTH];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*COORDS_LENGTH);
    uint8_t rssi = p->rssi;
    //TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d), Sent to Ad\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
}

void AdP2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    uint8_t other_id = p->data[0];
    static coordinate_t msg[COORDS_LENGTH];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*COORDS_LENGTH);
    uint8_t rssi = p->rssi;
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d) Sending to AD...\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
    //Send msg to GAP8
    CPXPacket_t cpxPacket;
    cpxInitRoute(CPX_T_STM32,CPX_T_GAP8,CPX_F_APP,&cpxPacket.route);
    cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
    memcpy(cpxPacket.data, msg, cpxPacket.dataLength);
    bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
    DEBUG_PRINT("Send %s\n",flag==false?"timeout":"success");
}

void MtrP2PListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(MtrP2PCallbackHandler);
}

void AdP2PListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(AdP2PCallbackHandler);
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}

bool SendCoords(coordinate_t* coords){

    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port=0x00;

    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    packet.data[0]=my_id;

    memcpy(&packet.data[1], coords, sizeof(coordinate_t)*COORDS_LENGTH);

    // Set the size, which is the amount of bytes the payload with ID and the string
    packet.size=sizeof(coordinate_t)*COORDS_LENGTH+1;
    // Send the P2P packet
    DEBUG_PRINT("P2P Msg Sent by:%d, First Coord is: (%d,%d,%d)\n",my_id,coords[0].x,coords[0].y,coords[0].z);
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool SendReq(coordinate_t* coords) {
    //TODO Req send and differ packets
    return false;
}

void sendMsgToGAP8(coordinate_t *coords)
{
    CPXPacket_t cpxPacket;
    cpxPacket.route.source=1;
    cpxPacket.route.destination=4;
    cpxPacket.route.function=5;
    cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
    memcpy(&cpxPacket,coords,cpxPacket.dataLength);
    bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
    DEBUG_PRINT("Send %s\n",flag==false?"failed":"success");
    vTaskDelay(M2T(10000));
    return;
}

// uint16_t* ReceiveMsgFromGAP8(CPXPacket_t *CPXPacket){
//     /*
//     1. 监听各种途径发过来的CPX包(步骤1不需要，放在主函数里)
//     2. 把CPX包按照包的结构拆开，分别存储
//     3. 打印到Console上
//     4. 封装成P2P包，发送给其他crazyflie
//     */
//    //2. 拆开CPX包
//    /*
//    cpx packet sturcture

//    */
//     //暂存
//     uint8_t From = CPXPacket->route.source;
//     uint8_t Dest = CPXPacket->route.destination;

    
//     uint16_t ReceiveLength = CPXPacket->dataLength;
//     uint16_t MergeMsg[ReceiveLength/2+1];

//     // 暂存payload
//     static uint8_t buffer[ReceiveLength];
//     memcpy(buffer, &CPXPacket->data[0], ReceiveLength);
    
//     //两个uint8_t合并成uint16_t
//     int len = sizeof(buffer) / sizeof(buffer[0]);
//     int i, j;
//     if(len%2==0){
//         for (i = 0, j = 0; i < len; i+=2,j++){
//             MergeMsg[j] = ((uint16_t)buffer[i] << 8) | buffer[i + 1];
//         }
//     }else{
//         for (i = 0, j = 0; i < len - 1;i+=2,j++){
//             MergeMsg[j] = ((uint16_t)buffer[i] << 8) | buffer[i + 1];
//         }
//         MergeMsg[j] = ((uint16_t)buffer[len - 1]);
//     }

//     //打印到Console
//     DEBUG_PRINT("Analyse Msg From AIdeck successfully");
//     return MergeMsg;
// }
// void RecFmGAP8(){
//     int buffer[SIZE-1];
//     cpxInternalRouterReceiveOthers(&cpxRx);
//     if(cpxRx.route.function == CPX_F_WIFI_CTRL){
//         if(cpxRx.data[0] == 0x31){
//             for (int i = 0; i < SIZE-1;i++){
//                 buffer[i] = cpxRx.data[i + 1];
//             }
//             DEBUG_PRINT("%d,%d,%d\n", buffer[0], buffer[1]);
//         }
//         else
//         {
//             DEBUG_PRINT("WRONG DATA!");
//         }
//     }else{
//         DEBUG_PRINT("OTHER DATA!");
//     }
// }

