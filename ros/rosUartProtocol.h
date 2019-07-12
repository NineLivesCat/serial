#ifndef _UART_PROTOCOL_H_
#define _UART_PROTOCOL_H_

/*
 *   NOTE: This header file is shared among
 *   robomaster computer(s) & MCU(s)
 *   =======SHOULD CHECK THE VERSION NUMBER BEFORE USE======
 *   =======VERSION: 2019.07.12=============================
 */
#define UART_PROTOCOL_VERSION     0x05
#define UART_START_BYTE           0xAA
#define UART_CHECKSUM_OFFSET      0xA5

#define UART_SYNC_H2G_ID          0x01
#define UART_SYNC_G2H_ID          0x02
#define UART_HOST_HEARTBEAT_ID    0x03
#define UART_GIMBAL_INFO_ID       0x05
#define UART_GIMBAL_CMD_ID        0x06
#define UART_TARGET_ID            0x07
#define UART_ROS_PARAM_ID         0x0A
#define UART_ROS_RESPONSE_ID      0x0B
#define UART_INVALID_ID           0xFF

#define RESPONSE_OK             0xA5A5

#define MIN_BULLET_SPEED           10U

typedef enum
{
    CV_MODE_ARMOR   = 0,
    CV_MODE_RUNE    = 1,
    CV_MODE_DUMMY   = 255
} cv_mode_t;

typedef enum{
    SHOOTING_MODE_IDLE       = 0,
    SHOOTING_MODE_FIRE       = 1,
    SHOOTING_MODE_CRAZY_FIRE = 2, //Very Very DEADLY!!!!!!
    SHOOTING_MODE_FORBID     = 3
};

typedef enum
{
    ROBOT_TEAM_UNDEFINED = 0,
    ROBOT_TEAM_RED       = 1,
    ROBOT_TEAM_BLUE      = 2
};

typedef enum
{
    RUNE_SMALL = 0,
    RUNE_BIG   = 1,
};

typedef struct
{
    uint8_t start;
    uint8_t type;
} __attribute__((packed)) uart_header_t;

typedef uint8_t  uart_crc_t;
typedef uint8_t  uart_heartbeat_t;

typedef struct
{
    uint32_t dt;
    int16_t  error;
    uint8_t  status  : 1;
    uint8_t  version : 7;
} __attribute__((packed)) uart_sync_t;

typedef struct
{
    uint8_t  bullet_speed_0  : 5;  //17mm speed
    uint8_t  bullet_speed_1  : 3;  //42mm speed
    float    yaw;
    float    pitch;
    float    roll;
    int8_t   rc_x;
    int8_t   rc_y;
    uint8_t  rc_enable_cv   : 1;
    uint8_t  rc_reset_cv    : 1;
    uint8_t  rc_cv_mode     : 2; //0-armor, 1-rune, 2-siege
    uint8_t  rc_reserve     : 4;
    float    imu_w[3];
    uint8_t  color          : 2; //undefined:0, red:1, blue:2
    uint8_t  rune_mode      : 1; //remain time in minutes
    uint8_t  reserve        : 5;
} __attribute__((packed)) uart_gimbal_info_t;

typedef struct
{
    uint8_t  reserve[27];
    uint16_t content;      //Send -1 to respond to parameter packet
} __attribute__((packed)) uart_param_response_t;

typedef struct
{
    float VS_kp_p;
    float VS_kd_p;
    float VS_kp_y;
    float VS_kd_y;
    float VS_sd;
} __attribute__((packed)) uart_ros_param_t;

typedef struct
{
    float    qw;
    float    qx;
    float    qy;
    float    qz;
    uint8_t  shootMode_0 : 3;
    uint8_t  shootMode_1 : 3;
    uint8_t  valid       : 1;
    uint8_t  tracking    : 1;

    uint8_t  scaleDown   : 1;
    uint8_t  target_num  : 7;

    uint16_t distance_cm;
    uint8_t  reserve[9];
} __attribute__((packed)) uart_gimbal_cmd_t;

typedef struct
{
    float    z_pos_0;
    float    y_pos_0;
    float    z_pos_1;
    float    y_pos_1;
    float    z_vel;
    float    y_vel;

    uint8_t  shootMode_0 : 3;
    uint8_t  shootMode_1 : 3;
    uint8_t  valid       : 1;
    uint8_t  tracking    : 1;

    uint8_t  scaleDown   : 1;
    uint8_t  target_num  : 7;

    uint16_t distance_cm;
    uint8_t  reserve;
} __attribute__((packed)) uart_target_t;

#endif
