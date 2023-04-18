//
// Created by mfxjx on 2023/3/29.
//

#include "communicate.h"
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

void appMain(){

    //vTaskDelay(M2T(20000));
    //create test coords
    P2PListeningInit();
    cpxInit();
    cpxInternalRouterInit();
    cpxExternalRouterInit();
    while(1)
    {
        ProcessAndTransfer();
        vTaskDelay(M2T(100));  
    }
}