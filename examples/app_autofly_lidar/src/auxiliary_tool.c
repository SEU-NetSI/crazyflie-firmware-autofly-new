#include <stdbool.h>
#include "math.h"
#include "log.h"
#include "auxiliary_tool.h"
#include "config_autofly.h"

void get_measurement(example_measure_t *measurement)
{
    // distance unit: cm
    measurement->data[0] = logGetFloat(logGetVarId("range", "front")) / 10;
    measurement->data[1] = logGetFloat(logGetVarId("range", "back")) / 10;
    // measurement->up = logGetFloat(logGetVarId("range","up")) / 10;
    measurement->data[2] = logGetFloat(logGetVarId("range", "left")) / 10;
    measurement->data[3] = logGetFloat(logGetVarId("range", "right")) / 10;

    measurement->pitch = logGetFloat(logGetVarId("stabilizer", "pitch"));
    measurement->roll = logGetFloat(logGetVarId("stabilizer", "roll"));
    measurement->yaw = logGetFloat(logGetVarId("stabilizer", "yaw"));
}

bool cal_Point(example_measure_t *measurement, coordinateF_t *start_point, rangeDirection_t dir, coordinateF_t *res)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    switch (dir)
    {
    case rangeFront:
        if (measurement->data[0] < SENSOR_TH && measurement->data[0] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x + measurement->data[0], start_point->y, start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeBack:
        if (measurement->data[1] < SENSOR_TH && measurement->data[1] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x - measurement->data[1], start_point->y, start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeLeft:
        if (measurement->data[2] < SENSOR_TH && measurement->data[2] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y + measurement->data[2], start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeRight:
        if (measurement->data[3] < SENSOR_TH && measurement->data[3] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y - measurement->data[3], start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeUp:
        if (measurement->data[4] < SENSOR_TH && measurement->data[4] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y, start_point->z + measurement->data[4]};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeDown:
        if (measurement->data[5] < SENSOR_TH && measurement->data[5] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y, start_point->z - measurement->data[5]};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    default:
        break;
    }
    return FALSE;
}

coordinateF_t rot(float roll, float pitch, float yaw, coordinateF_t *origin, coordinateF_t *point)
{
    float cosr = cos((double)roll * M_PI / 180);
    float cosp = cos((double)pitch * M_PI / 180);
    float cosy = cos((double)yaw * M_PI / 180);

    float sinr = sin((double)roll * M_PI / 180);
    float sinp = sin((double)pitch * M_PI / 180);
    float siny = sin((double)yaw * M_PI / 180);

    float roty[3][3];
    float rotp[3][3];
    float rotr[3][3];

    roty[0][0] = cosy;
    roty[0][1] = -siny;
    roty[0][2] = 0;
    roty[1][0] = siny;
    roty[1][1] = cosy;
    roty[1][2] = 0;
    roty[2][0] = 0;
    roty[2][1] = 0;
    roty[2][2] = 1;

    rotp[0][0] = cosp;
    rotp[0][1] = 0;
    rotp[0][2] = sinp;
    rotp[1][0] = 0;
    rotp[1][1] = 1;
    rotp[1][2] = 0;
    rotp[2][0] = -sinp;
    rotp[2][1] = 0;
    rotp[2][2] = cosp;

    rotr[0][0] = 1;
    rotr[0][1] = 0;
    rotr[0][2] = 0;
    rotr[1][0] = 0;
    rotr[1][1] = cosr;
    rotr[1][2] = -sinr;
    rotr[2][0] = 0;
    rotr[2][1] = sinr;
    rotr[2][2] = cosr;
    float tmp[3][1];
    tmp[0][0] = point->x - origin->x;
    tmp[1][0] = point->y - origin->y;
    tmp[2][0] = point->z - origin->z;

    dot(roty, tmp);
    dot(rotp, tmp);
    dot(rotr, tmp);
    coordinateF_t tmp2 = {tmp[0][0] + origin->x, tmp[1][0] + origin->y, tmp[2][0] + origin->z};

    determine_threshold(&tmp2);
    return tmp2;
}

void determine_threshold(coordinateF_t *point)
{
    point->x = fmax(fmin(point->x, WIDTH_X), 0);
    point->y = fmax(fmin(point->y, WIDTH_Y), 0);
    point->z = fmax(fmin(point->z, WIDTH_Z), 0);
}

void dot(float A[][3], float B[][1])
{
    float C[3][1];
    for (int i = 0; i < 3; i++)
    {
        C[i][0] = 0;
        for (int k = 0; k < 3; k++)
        {
            C[i][0] += A[i][k] * B[k][0];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        B[i][0] = C[i][0];
    }
}
