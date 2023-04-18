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
    packet.data[2] = exploreRequestSeq >> 8;
    packet.data[3] = exploreRequestSeq & 0xff;
    memcpy(&packet.data[4], exploreRequestPayloadPtr, sizeof(explore_req_payload_t));
    // 1b for sourceId, 2b for exploreRequestSeq, 6b for coordinate_t
    packet.size = sizeof(sourceId) + sizeof(exploreRequestSeq) + sizeof(explore_req_payload_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
