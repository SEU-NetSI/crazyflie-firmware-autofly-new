#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"


#include "debug.h"

#include"radiolink.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("Callback called!");
    uint8_t other_id = p->data[0];
    uint8_t reqType = p->data[1];
    static coordinate_t msg[5];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*reqType);
    uint8_t rssi = p->rssi;
    //TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d), Sent to Ad\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
}

void ListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool SendReq(coordinate_t* coords,ReqType mode,uint16_t seq){

    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port=0x00;

    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    packet.data[0]=my_id;
    packet.data[1]=mode;
    packet.data[2]=seq;
    memcpy(&packet.data[3], coords, sizeof(coordinate_t)*mode);

    // Set the size, which is the amount of bytes the payload with ID and the string
    packet.size=sizeof(coordinate_t)*mode+sizeof(uint8_t)*2+sizeof(uint16_t);
    // Send the P2P packet
    DEBUG_PRINT("ReqType:%d Sent by:%d Seq:%d, First Coord is: (%d,%d,%d)\n",mode,my_id,seq,coords[0].x,coords[0].y,coords[0].z);
    return radiolinkSendP2PPacketBroadcast(&packet);
}

