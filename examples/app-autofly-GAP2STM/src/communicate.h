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
#define COORDS_LENGTH 6
typedef enum {
    ExploreReq = 1,
    PathReq = 2,
    MappingReq = 5
} ReqType;
#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5
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
} coordinate_pair_t;
bool SendReq(coordinate_t *coords, ReqType mode, uint16_t seq);
void ProcessAndTransfer();
void P2PListeningInit();
//bool SendReq(coordinate_t* coords,ReqType mode);
#endif