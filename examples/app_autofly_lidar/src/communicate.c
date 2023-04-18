#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"
#include "debug.h"

#include"radiolink.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("[STM32-LiDAR]Callback called!");
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t respType = p->data[1];
    uint16_t respSeq = p->data[2];

    static coordinate_t responsePayload[RESPONSE_PAYLOAD_LENGTH];
    memcpy(responsePayload, &p->data[3], sizeof(coordinate_t) * RESPONSE_PAYLOAD_LENGTH);
    
    // TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[STM32-LiDAR]Receive P2P response from: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", sourceId, rssi, respType, respSeq);
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool sendMappingRequest(coordinate_pair_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)MAPPING_REQ;
    packet.data[2] = mappingRequestSeq >> 8;
    packet.data[3] = mappingRequestSeq & 0xff;
    packet.data[4] = mappingRequestPayloadLength;
    memcpy(&packet.data[5], mappingRequestPayloadPtr, sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
    // 1b for sourceId, 2b for mappingRequestSeq, 1b for mappingRequestPayloadLength, 12b for each coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof((uint8_t)MAPPING_REQ) + sizeof(mappingRequestSeq) + sizeof(mappingRequestPayloadLength) + sizeof(coordinate_pair_t)*mappingRequestPayloadLength;
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)EXPLORE_REQ;
    packet.data[2] = exploreRequestSeq;
    memcpy(&packet.data[3], exploreRequestPayloadPtr, sizeof(explore_req_payload_t));
    // 1b for sourceId, 2b for exploreRequestSeq, 6b for coordinate_t
    packet.size = sizeof(sourceId) + sizeof(exploreRequestSeq) + sizeof(explore_req_payload_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendPathRequest(coordinate_pair_t* pathRequestPayloadPtr, uint16_t pathRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)PATH_REQ;
    packet.data[2] = pathRequestSeq;
    memcpy(&packet.data[3], pathRequestPayloadPtr, sizeof(coordinate_pair_t));
    // 1b for sourceId, 2b for pathRequestSeq, 12b for coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof(pathRequestSeq) + sizeof(coordinate_pair_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("[STM32-Lidar] Callback called!");
    uint8_t other_id = p->data[0];
    uint8_t reqType = p->data[1];
    static coordinate_t msg[5];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*reqType);
    uint8_t rssi = p->rssi;
    //TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d), Sent to Ad\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
}

void ListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
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
    packet.data[2]=seq;
    memcpy(&packet.data[3], coords, sizeof(coordinate_t)*mode);

    // Set the size, which is the amount of bytes the payload with ID and the string
    packet.size=sizeof(coordinate_t)*mode+sizeof(uint8_t)*2+sizeof(uint16_t);
    // Send the P2P packet
    DEBUG_PRINT("ReqType:%d Sent by:%d Seq:%d, First Coord is: (%d,%d,%d)\n",mode,my_id,seq,coords[0].x,coords[0].y,coords[0].z);
    return radiolinkSendP2PPacketBroadcast(&packet);
}

