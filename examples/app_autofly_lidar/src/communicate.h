#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "measure_tool.h"
#define DEBUG_MODULE "P2P"

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5
#define METRICS_REQ 9

#define MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT 3
#define MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC 2
#define MAPPING_REQUEST_PAYLOAD_LENGTH_MOVING 1
#define UAV_COMPUTING_ID 0x7E

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

typedef struct {
    uint16_t mappingRequestCount;
    uint16_t exploreRequestCount;
    uint16_t exploreResponseCount;
} metrics_req_payload_t;

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

typedef struct{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    metrics_req_payload_t metricsRequestPayload;
} metrics_req_packet_t;


uint8_t getSourceId();
bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq);
bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq);
bool sendMetricsRequest(metrics_req_payload_t* metricsRequestPayloadPtr, uint16_t metricsRequestSeq);
#endif
