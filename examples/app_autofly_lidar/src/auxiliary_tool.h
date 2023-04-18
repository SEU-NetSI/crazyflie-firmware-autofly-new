#include "range.h"

typedef struct
{
    float x;
    float y;
    float z;
} coordinateF_t;

typedef struct
{
    float data[6];
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

void get_measurement(example_measure_t *measurement);
bool cal_Point(example_measure_t *measurement, coordinateF_t *start_point, rangeDirection_t dir, coordinateF_t *res);
//rotate
coordinateF_t rot(float roll, float pitch, float yaw, coordinateF_t* origin, coordinateF_t* point);
void determine_threshold(coordinateF_t *point);
void dot(float A[][3], float B[][1]);