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
