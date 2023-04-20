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

uint8_t getSourceId()
{
    return UAV_LIDAR_ID;
}

bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;
    // Assemble the packet
    mapping_req_packet_t mappingReqPacket;
    mappingReqPacket.sourceId = sourceId;
    mappingReqPacket.destinationId = destinationId;
    mappingReqPacket.packetType = MAPPING_REQ;
    mappingReqPacket.seq = mappingRequestSeq;
    mappingReqPacket.mappingRequestPayloadLength = mappingRequestPayloadLength;
    memcpy(&mappingReqPacket.mappingRequestPayload, mappingRequestPayloadPtr, sizeof(mapping_req_payload_t)*mappingRequestPayloadLength);

    // DEBUG_PRINT("sizeof(mapping_req_packet_t):%d\n",sizeof(mapping_req_packet_t)); # 36
    memcpy(&packet.data, &mappingReqPacket, sizeof(mapping_req_packet_t));
    packet.size = sizeof(mapping_req_packet_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;
    // Assemble the packet
    explore_req_packet_t exploreReqPacket;
    exploreReqPacket.sourceId = sourceId;
    exploreReqPacket.destinationId = destinationId;
    exploreReqPacket.packetType = EXPLORE_REQ;
    exploreReqPacket.seq = exploreRequestSeq;
    memcpy(&exploreReqPacket.exploreRequestPayload, exploreRequestPayloadPtr, sizeof(explore_req_payload_t));

    memcpy(&packet.data, &exploreReqPacket, sizeof(explore_req_packet_t));
    packet.size = sizeof(explore_req_packet_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
