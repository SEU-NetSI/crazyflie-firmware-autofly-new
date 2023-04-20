#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"




#include "radiolink.h"
#include "cpx.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5

#define MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT 4
#define MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC 4
#define MAPPING_REQUEST_PAYLOAD_LENGTH_MOVING 1
#define UAV_COMPUTING_ID 0x00

typedef struct
{
    float data[6];
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct
{
    coordinate_t startPoint;
    coordinate_t endPoint;
    uint8_t mergedNums;
} mapping_req_payload_t;

typedef struct
{
    coordinate_t startPoint;
    example_measure_t measurement;
} explore_req_payload_t;

typedef struct
{
    coordinate_t endPoint;
} explore_resp_payload_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    uint8_t mappingRequestPayloadLength;
    mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
} mapping_req_packet_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_resp_payload_t exploreResponsePayload;
} explore_resp_packet_t;

void CPXForwardInit();
void P2PListeningInit();
bool sendExploreResponse(explore_resp_packet_t* exploreResponsePacket);
#endif