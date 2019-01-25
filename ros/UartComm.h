#pragma once

#include <ros/ros.h>
#include "UartProtocol.h"
#include "serial/serial.h"

#include "can_host_msgs/gimbalInfo.h"
#include "geometry_msgs/TwistStamped.h"

class CommBase
{
public:
    CommBase(ros::NodeHandle& nh)
    {
        start_time = ros::Time::now();
        comm_status = COMM_UNINIT;
        sync_error = 0;
    }

    enum comm_status_t
    {
        COMM_UNINIT = 0,
        COMM_OFF    = 1,
        COMM_SYNC_0 = 2,
        COMM_SYNC_1 = 3,
        COMM_ON     = 4,
        COMM_ERROR  = -1
    };

    bool check_timeout(void)
    {
        return ros::Time::now() - heartbeat_time > ros::Duration(HEARTBEAT_TIMEOUT_S);
    }

protected:
    ros::Time     start_time;
    ros::Time     heartbeat_time;

    int16_t       sync_error; //sync error in microsecond
    const double  SYNC_TIMEOUT_S      = 1;
    const double  HEARTBEAT_TIMEOUT_S = 1;
    const double  SYNC_ERROR_TH_MS    = 0.5;

    comm_status_t comm_status;
};

class UartComm : public CommBase
{
public:
    UartComm(ros::NodeHandle& nh, serial::Serial* serial_port):
        CommBase(nh), serial_port(serial_port), sync_attempt(0)
    {
        ROS_INFO("Starting UART host");
        comm_status = COMM_UNINIT;
    }

    /*
     * @brief:  convert ros::Time to uart synchonization packet data
     * @return: length of tx bufffer
     */
    uint8_t packSyncSeq(uint8_t txbuf[], const bool);

    uint8_t processSyncSeq(uint8_t rxbuf[]);

    /*
     * @brief:  convert geometry_msgs::TwistStamped to
     *          to uart gimbal cmd packet data
     * @return: length of tx bufffer
     */
    uint8_t packGimbalCmd(uint8_t txbuf[],
        const geometry_msgs::TwistStamped &msg);
    void    processGimbalInfo(uint8_t rxbuf[], ros::Publisher &pub,
        const bool valid);

    void gimbalCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

private:
    uint32_t sync_attempt;
    serial::Serial* serial_port;

    ros::Time restore_timeStamp32(uint16_t rx_time)
    {
        uint32_t timestamp_ms =
            (ros::Time::now() - start_time).toNSec()/1e6;

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

        return start_time + rx_duration;
    }
};
