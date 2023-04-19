#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "radiolink.h"
#include "configblock.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"

#define DEBUG_PRINT_ENABLED 1


void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t destinationId = p->data[1];
    uint8_t packetType = p->data[2];
    if (destinationId != UAV_COMPUTING_ID)
    {
        return;
    }

    if (packetType == MAPPING_REQ) {
        mapping_req_packet_t mappingRequestPacket;
        memcpy(&mappingRequestPacket, p->data, sizeof(mapping_req_packet_t));
        DEBUG_PRINT("[Edge-STM32]P2P: Receive mapping request from: %d, RSSI: -%d dBm, seq: %d, payloadLength: %d\n", 
            mappingRequestPacket.sourceId, rssi, mappingRequestPacket.seq, mappingRequestPacket.mappingRequestPayloadLength);

        // Print debugging info
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[Edge-STM32]P2P: mapping request payload: \n");
            for (int i = 0; i < mappingRequestPacket.mappingRequestPayloadLength; i++)
            {
                DEBUG_PRINT("[Edge-STM32]P2P: coordinatePair %d: (%d, %d, %d), (%d, %d, %d), mergedNums: %d\n", 
                    i, 
                    mappingRequestPacket.mappingRequestPayload[i].startPoint.x, 
                    mappingRequestPacket.mappingRequestPayload[i].startPoint.y, 
                    mappingRequestPacket.mappingRequestPayload[i].startPoint.z,
                    mappingRequestPacket.mappingRequestPayload[i].endPoint.x, 
                    mappingRequestPacket.mappingRequestPayload[i].endPoint.y, 
                    mappingRequestPacket.mappingRequestPayload[i].endPoint.z,
                    mappingRequestPacket.mappingRequestPayload[i].mergedNums);
                vTaskDelay(50);
            }
            DEBUG_PRINT("\n");
        }
        
        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        memcpy(&cpxPacket.data, &mappingRequestPacket, sizeof(mapping_req_packet_t));
        cpxPacket.dataLength = sizeof(mapping_req_packet_t);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[Edge-STM32]CPX: Forward mapping request %s\n\n", flag == false ? "timeout" : "success");
    } else if (packetType == EXPLORE_REQ) {
        explore_req_packet_t exploreRequestPacket;
        memcpy(&exploreRequestPacket, p->data, sizeof(explore_req_packet_t));
        DEBUG_PRINT("[Edge-STM32]P2P: Receive explore request from: %d, RSSI: -%d dBm, seq: %d\n", 
            exploreRequestPacket.sourceId, rssi, exploreRequestPacket.seq);
        
        // Print debugging info
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[Edge-STM32]P2P: Explore request payload: \n");
            DEBUG_PRINT("[Edge-STM32]P2P: startPoint: (%d, %d, %d)\n", 
                exploreRequestPacket.exploreRequestPayload.startPoint.x, 
                exploreRequestPacket.exploreRequestPayload.startPoint.y, 
                exploreRequestPacket.exploreRequestPayload.startPoint.z);
            DEBUG_PRINT("[Edge-STM32]P2P: data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[0], 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[1], 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[2], 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[3], 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[4], 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.data[5]);
            DEBUG_PRINT("[Edge-STM32]P2P: roll: %.2f, pitch: %.2f, yaw: %.2f\n", 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.roll, 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.pitch, 
                (double)exploreRequestPacket.exploreRequestPayload.measurement.yaw);
            vTaskDelay(50);
        }

        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        memcpy(&cpxPacket.data, &exploreRequestPacket, sizeof(explore_req_packet_t));
        cpxPacket.dataLength = sizeof(explore_req_packet_t);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[Edge-STM32]CPX: Forward explore request %s\n\n", flag == false ? "timeout" : "success");
    } else {
        DEBUG_PRINT("[Edge-STM32]P2P: Receive unknown packet from: %d, RSSI: -%d dBm, destinationId: %d, packetType: %d\n\n", 
            sourceId, rssi, destinationId, packetType);
    }
}

bool sendExploreResponse(explore_resp_packet_t* exploreResponsePacket)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Assemble the packet
    memcpy(packet.data, exploreResponsePacket, sizeof(explore_resp_packet_t));
    packet.size = sizeof(explore_resp_packet_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

void CPXForwardInit() {
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[Edge-STM32]CPX Forward Init...\n");
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}