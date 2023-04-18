
// Autofly_Main config
#define OFFSET_X 50
#define OFFSET_Y 50
#define OFFSET_Z 0
#define TOP 60 //obstacle top
#define BOTTOM 30 //obstacle bottom
#define MOVE_DELAY 600

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
#define MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT 4
#define MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC 4
#define MAPPING_REQUEST_PAYLOAD_LENGTH_MOVING 1
#define TRUE 1
#define FALSE 0