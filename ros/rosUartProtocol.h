#ifndef _UART_PROTOCOL_H_
#define _UART_PROTOCOL_H_

/*
 *   NOTE: This header file is shared among
 *   robomaster computer(s) & MCU(s)
 *   =======SHOULD CHECK THE VERSION NUMBER BEFORE USE======
 *   =======VERSION: 2019.06.30=============================
 */
#define UART_PROTOCOL_VERSION     0x02
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

#define GIMBAL_INFO_ANG_PSC      10000
#define GIMBAL_CMD_ANGVEL_PSC     1000

#define TARGET_POS_PSC           40000
#define TARGET_VEL_PSC            4000
#define RESPONSE_OK             0xA5A5

typedef enum
{
    CTRL_MODE_IDLE  = 0,
    CTRL_MODE_AIM_0 = 1,
} gimbal_ctrl_mode_t;

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
    uint8_t  bullet_speed_0;
    uint8_t  bullet_speed_1;
    uint8_t  cv_mode       ;    //0-armor, 1-rune, 2-siege
    int16_t  yaw;                  //real range: -pi ~ pi
    int16_t  pitch;
    int16_t  roll;
    int8_t   rc_x;
    int8_t   rc_y;
    uint8_t  rc_enable_cv  : 1;
    uint8_t  rc_reset_cv   : 1;
    uint8_t  rc_reserve    : 6;
    int16_t  imu_w[3];
} __attribute__((packed)) uart_gimbal_info_t;

typedef struct
{
    uint8_t  reserve[16];
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
    int16_t  yaw_velCmd;
    int16_t  pitch_velCmd;
    uint8_t  shootMode_0 : 3;
    uint8_t  shootMode_1 : 3;
    uint8_t  valid       : 1;
    uint8_t  scaleDown   : 1;
    uint8_t  reserve;
} __attribute__((packed)) uart_gimbal_cmd_t;

typedef struct
{
    int16_t  z_pos_0;
    int16_t  y_pos_0;
    int16_t  z_pos_1;
    int16_t  y_pos_1;
    int16_t  z_vel;
    int16_t  y_vel;
    uint8_t  shootMode_0 : 3;
    uint8_t  shootMode_1 : 3;
    uint8_t  valid       : 1;
    uint8_t  scaleDown   : 1;
} __attribute__((packed)) uart_target_t;

#endif
