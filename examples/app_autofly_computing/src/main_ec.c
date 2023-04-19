#include <string.h>
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "system.h"
#include "app.h"

#include "cpx_internal_router.h"

#include "communicate.h"

static CPXPacket_t cpxRx;

void appMain()
{
    systemWaitStart();
    CPXForwardInit();

    while(1) {
        cpxInternalRouterReceiveOthers(&cpxRx);
        uint8_t respType = cpxRx.data[2];

        if (respType == EXPLORE_RESP) {
            explore_resp_packet_t exploreResponsePacket;
            memcpy(&exploreResponsePacket, cpxRx.data, sizeof(explore_resp_packet_t));
            DEBUG_PRINT("[Edge-STM32]CPX: Receive explore response packet, destinationId: %d, seq: %d\n", 
                exploreResponsePacket.destinationId, exploreResponsePacket.seq);
            bool flag = sendExploreResponse(&exploreResponsePacket);
            DEBUG_PRINT("[Edge-STM32]P2P: Forward explore response %s\n\n", flag == false ? "timeout" : "success");
        } else {
            DEBUG_PRINT("[Edge-STM32]CPX: Receive unknown packet, type: %d)\n\n", respType);
        }

        vTaskDelay(M2T(100));
    }
}