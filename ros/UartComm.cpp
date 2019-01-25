#include "UartComm.h"

void UartComm::gimbalCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if(comm_status == COMM_ON)
    {
        uint8_t txbuf[30];
        uint8_t length = packGimbalCmd(txbuf, *msg);
        this->serial_port->write(txbuf, length);
    }
}

uint8_t UartComm::packSyncSeq(uint8_t txbuf[], const bool sync_ok = false)
{
    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_SYNC_H2G_ID;

    uint8_t len = sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t);
    header.len = len;

    uart_sync_t sync;
    sync.dt = (ros::Time::now() - start_time).toNSec()/1e6;
    sync.error = this->sync_error;
    sync.status = sync_ok ? 1 : 0;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &sync,   sizeof(uart_sync_t  ));

    return len;
}

uint8_t UartComm::processSyncSeq(uint8_t rxbuf[])
{
    if(rxbuf[0] != UART_START_BYTE)
    {
        printf("%c\n", rxbuf[0]);
        ROS_ERROR("Incorrect data frame received");
        //TODO Do some other things to prevent the following packets go bad
        return -2;
    }

    uart_sync_t* sync = (uart_sync_t*)(rxbuf + sizeof(uart_header_t));
    this->sync_error = (ros::Time::now() - start_time).toNSec()/1e4 -
                        sync->dt*100;

    if(++sync_attempt > 1000)
    {
        ROS_FATAL("Invalid target sync data");
        return -1;
    }
    else if(sync_error < SYNC_ERROR_TH_MS*1e2 &&
            sync_error > -SYNC_ERROR_TH_MS*1e2)
    {
        comm_status = COMM_ON;
        std::cout<<"Timestamp sync attempt(s):"<<sync_attempt<<
            "\tsync_error:"<<double(sync_error)/1e2<<"ms \n";
        sync_attempt = 0;
        return 0;
    }

    return 1;
}

uint8_t UartComm::packGimbalCmd(uint8_t txbuf[], const geometry_msgs::TwistStamped &msg)
{
    uint8_t len = sizeof(uart_header_t)+sizeof(gimbal_cmd_t)+sizeof(uart_crc_t);

    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_GIMBAL_CMD_ID;

    gimbal_cmd_t cmd;
    cmd.yaw_velCmd   = msg.twist.angular.z * GIMBAL_CMD_ANGVEL_PSC;
    cmd.pitch_velCmd = msg.twist.angular.y * GIMBAL_CMD_ANGVEL_PSC;
    cmd.ctrl_mode    = msg.twist.angular.x;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &cmd,   sizeof(gimbal_cmd_t));

    return len;
}

void UartComm::processGimbalInfo(uint8_t rxbuf[], ros::Publisher &pub,
    const bool valid = true)
{
    heartbeat_time = ros::Time::now();
    can_host_msgs::gimbalInfo msg;
    msg.valid = valid;

    if(valid)
    {
        gimbal_info_t gimbal;
        memcpy(&gimbal, &rxbuf[sizeof(uart_header_t)], sizeof(gimbal_info_t));

        msg.header.stamp = restore_timeStamp32(gimbal.timeStamp_16);

        msg.imu_yaw      = (float)(gimbal.yaw)/GIMBAL_INFO_ANG_PSC;
        msg.imu_pitch    = (float)(gimbal.pitch)/GIMBAL_INFO_ANG_PSC;
        msg.gimbal_pitch = (float)(gimbal.gimbal_pitch_angle)/GIMBAL_INFO_ANG_PSC;
        msg.imu_w.x      = (float)(gimbal.ang_vel[0])/GIMBAL_INFO_ANGVEL_PSC;
        msg.imu_w.y      = (float)(gimbal.ang_vel[1])/GIMBAL_INFO_ANGVEL_PSC;
        msg.imu_w.z      = (float)(gimbal.ang_vel[2])/GIMBAL_INFO_ANGVEL_PSC;
    }
    else
        msg.header.stamp = ros::Time::now();

    pub.publish(msg);
}
