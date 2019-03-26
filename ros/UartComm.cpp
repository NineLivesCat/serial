#include "UartComm.h"

void UartComm::gimbalCmdCallback(const rm_vehicle_msgs::gimbalCmd::ConstPtr& msg)
{
    ROS_INFO("Fuck");
    if(comm_status == COMM_ON)
    {
        if((ros::Time::now() - last_write).toSec() < 1e-4)
            return;

        std::cout<<"Delay:"<<(ros::Time::now() - msg->header.stamp).toSec()<<std::endl;

        uint8_t txbuf[30];
        uint8_t length = packGimbalCmd(txbuf, *msg);
        this->serial_port->write(txbuf, length);
        last_write = ros::Time::now();
    }
    else if(comm_status == COMM_IDLE)
    {
        comm_status = COMM_ON; //Received first msg from other nodes
    }
}

uint8_t UartComm::packSyncSeq(uint8_t txbuf[], const bool sync_ok = false)
{
    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_SYNC_H2G_ID;

    static const uint8_t len = sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t);

    uart_sync_t sync;
    sync.dt      = (ros::Time::now() - sync_time).toNSec()/1e6;

    int32_t sync_error_32 = this->sync_error;
    if(sync_error_32 > 32767)
        sync_error_32 = 32767;
    else if(sync_error_32 < -32768)
        sync_error_32 = -32768;

    sync.error   = sync_error_32;
    sync.status  = sync_ok ? 1 : 0;
    sync.version = UART_PROTOCOL_VERSION;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &sync,   sizeof(uart_sync_t  ));

    this->append_crc(txbuf, len);

    return len;
}

void UartComm::SendSyncSeq(uint8_t txbuf[], const bool sync_ok = false)
{
    if((ros::Time::now() - last_write).toSec() < 1e-4)
        return;

    static const uint8_t len = sizeof(uart_header_t)+sizeof(uart_sync_t)+sizeof(uart_crc_t);

    packSyncSeq(txbuf, sync_ok);
    this->serial_port->write(txbuf, len);

    last_write = ros::Time::now();
}

uint8_t UartComm::processSyncSeq(uint8_t rxbuf[])
{
    static const uint8_t len = sizeof(uart_header_t) +
            sizeof(uart_sync_t) +
            sizeof(uart_crc_t);

    if(rxbuf[0] != UART_START_BYTE || !verify_crc(rxbuf, len))
    {
        if(comm_status > COMM_SYNC_0)
            ROS_WARN("Incorrect data frame received");

        toggleSyncMode();
        return -2;
    }
    else if(comm_status < COMM_SYNC_1)
    {
        serial::Timeout stable_timeout = serial::Timeout::simpleTimeout(50);
        this->serial_port->setTimeout(stable_timeout);

        ROS_INFO("Synchonizing with device");
        comm_status = COMM_SYNC_1;
    }

    uart_sync_t* sync = (uart_sync_t*)(rxbuf + sizeof(uart_header_t));
    this->sync_error  = (ros::Time::now() - sync_time).toNSec()/1e4 -
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
    static const uint8_t len = sizeof(uart_header_t)+sizeof(uart_gimbal_cmd_t)+sizeof(uart_crc_t);

    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_GIMBAL_CMD_ID;

    uart_gimbal_cmd_t cmd;

    uint32_t timestamp = (msg.header.stamp - sync_time).toNSec()/1e6;
    cmd.timeStamp_16 = (uint16_t)(timestamp & 0x0000FFFF);
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
    static const uint8_t len = sizeof(uart_header_t)+
            sizeof(uart_gimbal_info_t)+
            sizeof(uart_crc_t);

    heartbeat_time = ros::Time::now();
    rm_vehicle_msgs::gimbalInfo gimbalMsg;
    rm_vehicle_msgs::RC         RCMsg;

    if(valid)
    {
        if(rxbuf[0] != UART_START_BYTE || !verify_crc(rxbuf, len))
        {
            ROS_WARN("Incorrect data frame received");

            frame_err_cnt++;
            if(frame_err_cnt > 5)
                toggleSyncMode();
            return;
        }

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

        gimbalMsg.valid = gimbal.bullet_speed; //bullet_speed = 0 means gimbal not initialized
        RCMsg    .valid = gimbal.bullet_speed;

    }
    else
    {
        gimbalMsg.valid = valid;
        RCMsg    .valid = valid;

        gimbalMsg.header.stamp = ros::Time::now();
        RCMsg    .header.stamp = ros::Time::now();
    }

    gimbalInfo_pub.publish(gimbalMsg);
    RC_pub.publish(RCMsg);
}

void UartComm::heartbeatTxProcess(void)
{
    uint8_t txbuf[10];
    while(ros::ok())
    {
        if(comm_status == COMM_IDLE)
        {
            sendHeartbeat(txbuf);
            ros::Duration(0.05).sleep();
        }
    }
}

void UartComm::gimbalInfoRxProcess(void)
{
    static const uint8_t length = sizeof(uart_header_t)+
            sizeof(uart_gimbal_info_t)+sizeof(uart_crc_t);

    uint8_t rxbuf[length];
    while(ros::ok())
    {
        if(comm_status >= COMM_IDLE)
        {
            uint8_t rx_size = this->serial_port->read(rxbuf, length);
            if(rx_size == length)
                processGimbalInfo(rxbuf, true);

            if(check_timeout()) //Switch to sync mode
            {
                ROS_WARN("Connection lost with device, trying re-connection...");
                this->serial_port->flush();
                toggleSyncMode();
            }
        }
    }
}
