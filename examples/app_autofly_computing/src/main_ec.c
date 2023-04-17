#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"

#include "communicate.h"
#define COORDS_LENGTH 5

void appMain()
{
    // Listening for forward packets
    CPXForwardInit();
    P2PListeningInit();
    while(1) {
        vTaskDelay(M2T(2000));
    }
}