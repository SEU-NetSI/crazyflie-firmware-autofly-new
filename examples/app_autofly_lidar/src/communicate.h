#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
typedef enum {
    ExploreReq = 1,
    PathReq = 2,
    MappingReq = 5
} ReqType;
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

void ListeningInit();
bool SendReq(coordinate_t* coords,ReqType mode,uint16_t seq);
#endif