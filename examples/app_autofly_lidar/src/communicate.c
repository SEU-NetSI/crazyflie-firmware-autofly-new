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
    DEBUG_PRINT("[LiDAR-STM32]Callback called!");
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t respType = p->data[1];
    uint16_t respSeq = p->data[2];

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

bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
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
    memcpy(&packet.data[5], mappingRequestPayloadPtr, sizeof(mapping_req_payload_t)*mappingRequestPayloadLength);
    // 1b for sourceId, 2b for mappingRequestSeq, 1b for mappingRequestPayloadLength, 12b for each coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof((uint8_t)MAPPING_REQ) + sizeof(mappingRequestSeq) 
        + sizeof(mappingRequestPayloadLength) + sizeof(mapping_req_payload_t)*mappingRequestPayloadLength;
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
