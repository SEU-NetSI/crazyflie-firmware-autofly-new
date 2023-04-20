//#include "communicate.h"
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
#include "crtp.h"
#include "stdlib.h"

//static CPXPacket_t cpxRx;

void appMain()
{
    //systemWaitStart();
    CPXForwardInit();
    cpxInit();
    cpxInternalRouterInit();
    cpxExternalRouterInit();

    while(1) {
        CPXPacket_t *cpxRx = (CPXPacket_t *)malloc(sizeof(CPXPacket_t));
        cpxGetRxPacket(cpxRx);
        uint8_t respType = cpxRx->data[2];

        if (respType == EXPLORE_RESP) {
            explore_resp_packet_t exploreResponsePacket;
            memcpy(&exploreResponsePacket, cpxRx->data, sizeof(explore_resp_packet_t));
            DEBUG_PRINT("[Edge-STM32]CPX: Receive explore response packet, destinationId: %d, seq: %d\n", 
                exploreResponsePacket.destinationId, exploreResponsePacket.seq);
            bool flag = sendExploreResponse(&exploreResponsePacket);
            DEBUG_PRINT("[Edge-STM32]P2P: Forward explore response %s\n\n", flag == false ? "timeout" : "success");
        } else {
            DEBUG_PRINT("[Edge-STM32]CPX: Receive unknown packet, type: %d)\n\n", respType);
        }
        free(cpxRx);
        vTaskDelay(M2T(100));
    }
}