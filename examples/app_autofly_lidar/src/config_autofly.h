// Autofly_Main config
#define OFFSET_X 50
#define OFFSET_Y 50
#define OFFSET_Z 0
#define TOP 60 //obstacle top
#define BOTTOM 30 //obstacle bottom
#define DELAY_MOVE 600
#define DELAY_TAKEOFF 10000
#define DELAY_START 3000
#define DELAY_MAPPING 300
#define DELAY_PRINT 50
#define TIMEOUT_EXPLORE_RESP 250

//octomap config
#define TREE_CENTER_X 128
#define TREE_CENTER_Y 128
#define TREE_CENTER_Z 128
#define WIDTH_X TREE_CENTER_X * 2
#define WIDTH_Y TREE_CENTER_Y * 2
#define WIDTH_Z TREE_CENTER_Z * 2
#define TREE_RESOLUTION 4

#define LOG_ODDS_OCCUPIED 6
#define LOG_ODDS_FREE 0
#define LOG_ODDS_DIFF_STEP 1

//auxiliary_tool config
#define SENSOR_TH 300

// custom config for app-lidar
#define TRUE 1
#define FALSE 0