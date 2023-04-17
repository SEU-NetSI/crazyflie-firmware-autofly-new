//
// Created by chaihu on 3/29/23.
//
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "cpx.h"
#define COORDS_LENGTH 10
#ifndef CRAZYFLIE_FIRMWARE_COMMUNICATE_H
#define CRAZYFLIE_FIRMWARE_COMMUNICATE_H
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct{
    uint8_t x;
    uint8_t y;
    uint8_t z;
} __attribute__((packed)) MessagePacket;

void MtrP2PListeningInit();
void AdP2PListeningInit();
bool SendCoords(coordinate_t* coords);
bool SendReq(coordinate_t* coords);
void sendMsgToGAP8(coordinate_t *coords);
void ReceiveMsgFromGAP8(CPXPacket_t *CPXPacket);
void RecFmGAP8();
#endif // CRAZYFLIE_FIRMWARE_COMMUNICATE_H
