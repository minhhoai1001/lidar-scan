#ifndef _LIDAR_SCAN_H_
#define _LIDAR_SCAN_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <ctime>

typedef enum robot_type { unknow, B1, R5, R5_bin } ROBOT_TYPE;
typedef enum grip_type { SSG, SDG, DDG } GRIP_TYPE;
typedef enum roi_position { opposite, left, left_corner, right, right_corner } ROI_POSITION;

#define PI 3.14159265

#define DDG_L 0.505
#define DDG_W 0.705
#define SDG_L 0.48
#define SDG_W 0.705
#define SSG_L 0.48
#define SSG_W 0.68

#define R5_W 0.65
#define R5_L 0.505
#define R5BIN_L 0.989
#define B1_W 0.701
#define B1_L 0.642

struct Point
{
    float x, y, z;
};

struct st_calib_sensor
{
    float x_offset;
    float y_offset;
};

struct st_ROI_cell
{
    int x1;
    int y1;
    int x2;
    int y2;
};

struct st_ROI_m
{
    float x1;
    float y1;
    float x2;
    float y2;
};

struct st_service_req
{
    int grip;
    int robot_x;
    int robot_y;
    st_ROI_cell roi;
};

struct st_service_res
{
    ROBOT_TYPE type;
    int angle;
    int x;
    int x_sub;
    int y;
    int y_sub;
    int conf;
};

struct st_grip_info
{
    GRIP_TYPE type;
    float cell_length;
    float cell_width;
};

struct st_cluster_cfg
{
    unsigned int min_size;
    float epsilon;
    float distance;
};

#endif