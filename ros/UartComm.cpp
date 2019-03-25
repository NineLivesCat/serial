#include "UartComm.h"

void UartComm::gimbalCmdCallback(const rm_vehicle_msgs::gimbalCmd::ConstPtr& msg)
{
    if((ros::Time::now() - last_write).toSec() < 1e-4)
        return;

    if(comm_status == COMM_ON)
    {
        std::cout<<"Delay:"<<(ros::Time::now() - msg->header.stamp).toSec()<<std::endl;

        uint8_t txbuf[30];
        uint8_t length = packGimbalCmd(txbuf, *msg);
        this->serial_port->write(txbuf, length);
    }
    else if(comm_status == COMM_IDLE)
    {
        this->serial_port->flush();
        comm_status = COMM_ON; //Received first msg from other nodes
    }
}

uint8_t UartComm::packSyncSeq(uint8_t txbuf[], const bool sync_ok = false)
{
    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_SYNC_H2G_ID;

    uint8_t len = sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t);

    uart_sync_t sync;
    sync.dt = (ros::Time::now() - sync_time).toNSec()/1e6;
    sync.error = this->sync_error;
    sync.status = sync_ok ? 1 : 0;
    sync.version = UART_PROTOCOL_VERSION;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &sync,   sizeof(uart_sync_t  ));

    this->append_crc(txbuf, len);

    return len;
}

uint8_t UartComm::processSyncSeq(uint8_t rxbuf[])
{
    comm_status = COMM_SYNC_1;
    if(rxbuf[0] != UART_START_BYTE)
    {
        printf("%c\n", rxbuf[0]);
        ROS_ERROR("Incorrect data frame received");
        //TODO Do some other things to prevent the following packets go bad
        return -2;
    }

    uart_sync_t* sync = (uart_sync_t*)(rxbuf + sizeof(uart_header_t));
    this->sync_error = (ros::Time::now() - sync_time).toNSec()/1e4 -
                        sync->dt*100;

    if(++sync_attempt > 1000)
    {
        ROS_FATAL("Invalid target sync data");
        return -1;
    }
    else if(sync_error < SYNC_ERROR_TH_MS*1e2 &&
            sync_error > -SYNC_ERROR_TH_MS*1e2)
    {
        std::cout<<"Timestamp sync attempt(s):"<<sync_attempt<<
            "\tsync_error:"<<double(sync_error)/1e2<<"ms \n";
        sync_attempt = 0;
        return 0;
    }

    return 1;
}

uint8_t UartComm::packGimbalCmd(uint8_t txbuf[], const rm_vehicle_msgs::gimbalCmd &msg)
{
    uint8_t len = sizeof(uart_header_t)+sizeof(uart_gimbal_cmd_t)+sizeof(uart_crc_t);

    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_GIMBAL_CMD_ID;

    uart_gimbal_cmd_t cmd;
    cmd.yaw_velCmd   = msg.yaw_cmd * GIMBAL_CMD_ANGVEL_PSC;
    cmd.pitch_velCmd = msg.pitch_cmd * GIMBAL_CMD_ANGVEL_PSC;
    cmd.valid        = msg.valid;
    cmd.shootMode    = msg.shootMode;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &cmd,   sizeof(uart_gimbal_cmd_t));
    this->append_crc(txbuf, len);

    return len;
}

void UartComm::processGimbalInfo(uint8_t rxbuf[], const bool valid = true)
{
    heartbeat_time = ros::Time::now();
    rm_vehicle_msgs::gimbalInfo gimbalMsg;
    rm_vehicle_msgs::RC         RCMsg;

    gimbalMsg.valid = valid;
    RCMsg    .valid = valid;

    if(valid)
    {
        uart_gimbal_info_t gimbal;
        memcpy(&gimbal, &rxbuf[sizeof(uart_header_t)], sizeof(uart_gimbal_info_t));

        ros::Time stamp = restore_timeStamp32(gimbal.timeStamp_16);
        gimbalMsg.header.stamp = stamp;
        RCMsg.    header.stamp = stamp;

        gimbalMsg.imu_yaw      = (float)(gimbal.yaw)/GIMBAL_INFO_ANG_PSC;
        gimbalMsg.imu_pitch    = (float)(gimbal.pitch)/GIMBAL_INFO_ANG_PSC;
        gimbalMsg.gimbal_pitch = (float)(gimbal.gimbal_pitch_angle)/GIMBAL_INFO_ANG_PSC;
        gimbalMsg.imu_w.x      = (float)(gimbal.ang_vel[0])/GIMBAL_INFO_ANGVEL_PSC;
        gimbalMsg.imu_w.y      = (float)(gimbal.ang_vel[1])/GIMBAL_INFO_ANGVEL_PSC;
        gimbalMsg.imu_w.z      = (float)(gimbal.ang_vel[2])/GIMBAL_INFO_ANGVEL_PSC;
        gimbalMsg.bullet_speed = gimbal.bullet_speed;

        RCMsg  .control_enable = gimbal.cv_enable_cmd;
    }
    else
    {
        gimbalMsg.header.stamp = ros::Time::now();
        RCMsg    .header.stamp = ros::Time::now();
    }

    gimbalInfo_pub.publish(gimbalMsg);
    RC_pub.publish(RCMsg);
}
