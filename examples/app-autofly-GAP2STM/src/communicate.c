#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#include "radiolink.h"
#include "cpx.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse common data
    uint8_t sourceId = p->data[0];
    uint8_t reqType = p->data[1];
    uint16_t seq = p->data[2];
    uint8_t rssi = p->rssi;

    if (reqType == MAPPING_REQ) {
        uint8_t mappingRequestPayloadLength = p->data[3];
        coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
        memcpy(mappingRequestPayload, &p->data[4], sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]Receive P2P mapping request from: %d, RSSI: -%d dBm, seq: %d, payloadLength: %d\n", sourceId, rssi, seq, mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]First coordinate pair: (%d, %d, %d), (%d, %d, %d)\n",
            mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
            mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);
        
        //Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        cpxPacket.dataLength=1+sizeof(sourceId) + sizeof(reqType) + sizeof(seq) + sizeof(coordinate_pair_t)*mappingRequestPayloadLength;
        cpxPacket.data[0]=sourceId;
        cpxPacket.data[1]=reqType;
        cpxPacket.data[2]=seq;
        cpxPacket.data[3]=mappingRequestPayloadLength;
        memcpy(&cpxPacket.data[4], mappingRequestPayload, cpxPacket.dataLength);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[STM32-Edge]CPX Forward mapping request %s, from: %d, seq: %d\n\n", flag == false ? "timeout" : "success", sourceId, seq);
    } else {
        DEBUG_PRINT("[STM32-Edge]Receive P2P other request from: %d, RSSI: -%d dBm, seq: %d, reqType: %d\n", sourceId, rssi, seq, reqType);
    }
}

void CPXForwardInit() {
    cpxInternalRouterInit();
    cpxExternalRouterInit();
    DEBUG_PRINT("[STM32-Edge]CPX Forward Init...\n");
}

void P2PListeningInit() {
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[STM32-Edge]P2P Listening Init...\n");
}

bool SendReq(coordinate_t* coords,ReqType mode,uint16_t seq){

    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port=0x00;

    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    packet.data[0]=my_id;
    packet.data[1]=mode;
    //取出seq的高8位，放入packet.data[2]    
    packet.data[2]=(uint8_t)(seq>>8);
    packet.data[3] = (uint8_t)(seq);
    memcpy(&packet.data[4], coords, sizeof(coordinate_t) * mode);

    // Set the size, which is the amount of bytes the payload with ID and the string
    //packet.size=sizeof(coordinate_t)*mode+sizeof(uint8_t)*2+sizeof(uint16_t);
    packet.size=sizeof(coordinate_t)*mode+sizeof(uint8_t)*4;
    // Send the P2P packet
    DEBUG_PRINT("[STM32-Lidar]ReqType:%d Sent by:%d Seq:%d, First Coord is: (%d,%d,%d)\n",mode,my_id,seq,coords[0].x,coords[0].y,coords[0].z);
    return radiolinkSendP2PPacketBroadcast(&packet);
}

void ProcessAndTransfer(){
    coordinate_t coord[1] = {0};
    uint8_t buffer[100]={0};
    CPXPacket_t *rxPacket = (CPXPacket_t *)malloc(sizeof(CPXPacket_t));
    cpxGetRxPacket(rxPacket);
    //uint16_t a;
    //cpxGetRxPacket(&a);
    //DEBUG_PRINT("a is %d\n",a);
    //cpxGetRxPacket(&rxPacket);
    //这里是不是应该是desId
    uint8_t sourceId = rxPacket->data[0];
    DEBUG_PRINT("TEST: sourceId is %d\n",rxPacket->route.source);
    uint8_t reqType = rxPacket->data[1];
    DEBUG_PRINT("TEST: reqType is %d\n", reqType);
    uint16_t seq = rxPacket->data[2];
    uint8_t datalength = rxPacket->data[3];
    DEBUG_PRINT("TEST: datalength is %d\n", datalength);
    DEBUG_PRINT("TEST: data[5] is %d\n", rxPacket->data[5]);
    
    memcpy(buffer, &rxPacket->data[4], sizeof(uint8_t)*datalength);
    uint16_t x=buffer[0]<<8|buffer[1];
    uint16_t y=buffer[2]<<8|buffer[3];
    uint16_t z=buffer[4]<<8|buffer[5];
    coord[0].x=x;
    coord[0].y=y;
    coord[0].z=z;
    DEBUG_PRINT("TEST:COORD_X[0]=%d\n", coord[0].x);
    if (reqType == EXPLORE_RESP)
    {
        DEBUG_PRINT("[STM32-Edge]Receive CPX explore response from GAP8, seq: %d, payloadLength: %d\n", seq, datalength);
        bool flag = SendReq(coord, reqType, seq);
        //bool flag = false;
        DEBUG_PRINT("[STM32-Edge]P2P Forward explore response %s, from: %d, seq: %d\n\n", flag == false ? "timeout" : "success", sourceId, seq);
    }
    else if(reqType == 108){
        DEBUG_PRINT("[STM32-Edge]Receive CPX mapping response from GAP8, seq: %d, payloadLength: %d\n", seq, datalength);
        bool flag = SendReq(coord, 1, seq);
        //bool flag = false;
        DEBUG_PRINT("[STM32-Edge]P2P Forward mapping response %s, from: %d, seq: %d\n\n", flag == false ? "timeout" : "success", sourceId, seq);
    }
    else{

        DEBUG_PRINT("ERROR: Unknown CPX packet type\n");
    }
    free(rxPacket);
}

