#pragma once

#include <ros/ros.h>
#include "rosUartProtocol.h"
#include "serial/serial.h"

#include "rm_vehicle_msgs/gimbalInfo.h"
#include "rm_vehicle_msgs/gimbalCmd.h"
#include "rm_vehicle_msgs/RC.h"

class CommBase
{
public:
    CommBase(ros::NodeHandle& nh)
    {
        sync_time = ros::Time::now();
        comm_status = COMM_UNINIT;
        sync_error = 0;

        uint8_t chn_err_cnt = 0;
        uint8_t timeout_cnt = 0;
        uint8_t crc_err_cnt = 0;
        uint8_t protocol_err_cnt = 0;
    }

    enum comm_status_t
    {
        COMM_UNINIT  = 0,
        COMM_OFF     = 1,
        COMM_SYNC_0  = 2,
        COMM_SYNC_1  = 3,
        COMM_IDLE    = 4,
        COMM_ON      = 5,
        COMM_ERROR   = -1
    };

    bool check_timeout(void)
    {
        return ros::Time::now() - heartbeat_time > ros::Duration(HEARTBEAT_TIMEOUT_S);
    }

    bool inSyncMode(void)
    {
        return comm_status < COMM_IDLE;
    }

    bool inIdleMode(void)
    {
        return comm_status == COMM_IDLE;
    }

    void toggleSyncMode(void)
    {
        sync_time = ros::Time::now();
        comm_status = COMM_SYNC_0;
    }

    void toggleRXMode(void)
    {
        comm_status = COMM_IDLE;
    }

protected:
    bool verify_crc(const uint8_t* rxbuf, const uint8_t length)
    {
        uint8_t check_result = 0;
        uint8_t i = 0;
        for(;i < length - 1; check_result+= rxbuf[i++]);

        check_result += UART_CHECKSUM_OFFSET;

        return rxbuf[i] == check_result;
    }

    void append_crc(uint8_t* txbuf, const uint8_t length)
    {
        uint8_t result = 0;
        uint8_t i = 0;
        for(;i < length - 1; result += txbuf[i++]);

        result += UART_CHECKSUM_OFFSET;

        txbuf[i] = result;
    }

    ros::Time     sync_time;
    ros::Time     heartbeat_time;

    int16_t       sync_error; //sync error in microsecond
    const double  SYNC_TIMEOUT_S      = 1;
    const double  HEARTBEAT_TIMEOUT_S = 1;
    const double  SYNC_ERROR_TH_MS    = 1.0;

    static uint8_t chn_err_cnt;
    static uint8_t timeout_cnt;
    static uint8_t crc_err_cnt;
    static uint8_t protocol_err_cnt;

    comm_status_t comm_status;
};

class UartComm : public CommBase
{
public:
    UartComm(ros::NodeHandle& nh, serial::Serial* serial_port):
        CommBase(nh), serial_port(serial_port),
        sync_attempt(0), last_write(ros::Time::now())
    {
        ROS_INFO("Starting UART host");
        comm_status = COMM_UNINIT;
    }

    void sendHeartbeat(uint8_t txbuf[])
    {
        if((ros::Time::now() - last_write).toSec() < 1e-4)
            return;

        uart_header_t header;
        header.start = UART_START_BYTE;
        header.type  = UART_HOST_HEARTBEAT_ID;

        uint8_t len = sizeof(uart_header_t)+
            sizeof(uart_heartbeat_t)+
            sizeof(uart_crc_t);

        uint8_t* txptr = txbuf;
        memcpy(txptr, &header, sizeof(uart_header_t));

        this->append_crc(txbuf, len);
        this->serial_port->write(txbuf, len);

        last_write = ros::Time::now();
    }

    /*
     * @brief:  convert ros::Time to uart synchonization packet data
     * @return: length of tx bufffer
     */
    uint8_t packSyncSeq(uint8_t txbuf[], const bool sync_ok);

    uint8_t processSyncSeq(uint8_t rxbuf[]);

    /*
     * @brief:  convert geometry_msgs::TwistStamped to
     *          to uart gimbal cmd packet data
     * @return: length of tx bufffer
     */
    uint8_t packGimbalCmd(uint8_t txbuf[],
        const rm_vehicle_msgs::gimbalCmd &msg);
    void    processGimbalInfo(uint8_t rxbuf[], const bool valid);

    void gimbalCmdCallback(const rm_vehicle_msgs::gimbalCmd::ConstPtr& msg);

    ros::Publisher gimbalInfo_pub;
    ros::Publisher RC_pub;

private:
    uint32_t sync_attempt;
    serial::Serial* serial_port;

    ros::Time last_write;

    ros::Time restore_timeStamp32(uint16_t rx_time)
    {
        uint32_t timestamp_ms =
            (ros::Time::now() - sync_time).toNSec()/1e6;

        uint16_t timestamp_low16  = (uint16_t)(timestamp_ms);
        uint16_t timestamp_high16 = (uint16_t)(timestamp_ms >> 16);

        uint32_t rx_time_32 = 0;

        if(rx_time - timestamp_low16 > 32767 && timestamp_high16)
            rx_time_32 = ((timestamp_high16-1) << 16) | rx_time;
        else if(rx_time - timestamp_low16 < -32767)
            rx_time_32 = ((timestamp_high16+1) << 16) | rx_time;
        else
            rx_time_32 = (timestamp_high16 << 16) | rx_time;

        ros::Duration rx_duration((double)rx_time_32 / 1000);

        return sync_time + rx_duration;
    }
};
