#include "UartComm.h"

void UartComm::gimbalCmdCallback(const rm_vehicle_msgs::gimbalCmd::ConstPtr& msg)
{
    static const uint8_t len = sizeof(uart_header_t)+
        sizeof(uart_gimbal_cmd_t)+sizeof(uart_crc_t);

    static double tx_wait_time = len * 10. * 1.1 / this->baudrate;

    if(comm_status == COMM_ON)
    {
        if((ros::Time::now() - last_write).toSec() < tx_wait_time)
            return;

        uint8_t txbuf[30];
        packGimbalCmd(txbuf, *msg);
        this->serial_port->write(txbuf, len);
        last_write = ros::Time::now();
    }
    else if(comm_status == COMM_IDLE)
    {
        comm_status = COMM_ON; //Received first msg from other nodes
    }
}

void UartComm::visualServoCallback(const rm_cv_msgs::VisualServo::ConstPtr& msg)
{
    static const uint8_t len = sizeof(uart_header_t)+
        sizeof(uart_target_t)+sizeof(uart_crc_t);

    static double tx_wait_time = len * 10. * 1.1 / this->baudrate;

    if(comm_status == COMM_ON)
    {
        if((ros::Time::now() - last_write).toSec() < tx_wait_time)
            return;

        uint8_t txbuf[30];
        packTargetInfo(txbuf, *msg);
        this->serial_port->write(txbuf, len);
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
        this->serial_port->setTimeout(this->stable_timeout);

        ROS_INFO("Synchonizing with device");
        comm_status = COMM_SYNC_1;
    }

    uart_sync_t* sync = (uart_sync_t*)(rxbuf + sizeof(uart_header_t));
    if(sync->version != UART_PROTOCOL_VERSION)
    {
        ROS_FATAL("Incorrect version of uart ROS protocol!");
        return -3;
    }

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

uint8_t UartComm::packTargetInfo(uint8_t txbuf[], const rm_cv_msgs::VisualServo &msg)
{
    static const uint8_t len = sizeof(uart_header_t)+
        sizeof(uart_target_t)+sizeof(uart_crc_t);

    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type = UART_TARGET_ID;

    uart_target_t cmd;

    uint32_t timestamp = (msg.header.stamp - sync_time).toNSec()/1e6;
    cmd.timeStamp_16 = (uint16_t)(timestamp & 0x0000FFFF);

    //For gen1 visual servo, send target pos only
    cmd.z_pos        = (float)(msg.z )*TARGET_POS_PSC;
    cmd.y_pos        = (float)(msg.y )*TARGET_POS_PSC;
    cmd.z_vel        = (float)(msg.dz)*TARGET_VEL_PSC;
    cmd.y_vel        = (float)(msg.dy)*TARGET_VEL_PSC;
    cmd.valid        = msg.valid;
    cmd.shootMode    = msg.shootMode;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &cmd,   sizeof(uart_target_t));
    this->append_crc(txbuf, len);

    return len;
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
        uart_header_t* header = (uart_header_t*)&rxbuf[0];
        if(rxbuf[0] != UART_START_BYTE || !verify_crc(rxbuf, len) || header->type != UART_GIMBAL_INFO_ID)
        {
            ROS_WARN("Incorrect data frame received 1");

            frame_err_cnt++;
            if(frame_err_cnt > 5)
            {
                ROS_WARN("Broken frames exceed threshold, trying re-connection...");
                toggleSyncMode();
            }
            return;
        }

        uart_gimbal_info_t gimbal;
        memcpy(&gimbal, &rxbuf[sizeof(uart_header_t)], sizeof(uart_gimbal_info_t));

        if(this->use_hard_timestamp)
        {
            ros::Time stamp = restore_timeStamp32(gimbal.timeStamp_16);
            gimbalMsg.header.stamp = stamp;
            RCMsg.    header.stamp = stamp;
        }
        else
        {
            gimbalMsg.header.stamp = ros::Time::now() - dt;
            RCMsg.    header.stamp = ros::Time::now() - dt;
        }

        gimbalMsg.yaw   = (float)(gimbal.yaw  )/GIMBAL_INFO_ANG_PSC;
        gimbalMsg.pitch = (float)(gimbal.pitch)/GIMBAL_INFO_ANG_PSC;
        gimbalMsg.roll  = (float)(gimbal.roll )/GIMBAL_INFO_ANG_PSC;

        gimbalMsg.bullet_speed = gimbal.bullet_speed;
        RCMsg  .control_enable = gimbal.cv_enable_cmd;

        gimbalMsg.valid = valid; //bullet_speed = 0 means gimbal not initialized
        RCMsg    .valid = valid;

        //Process user input command
        cmd.reset = gimbal.cv_reset_cmd;

        static bool rune_mode = false;
        if(!rune_mode && gimbal.cv_mode == 1)
        {
            rune_mode = true;
            cmd.rune_mode = true;
        }
        else if(rune_mode && gimbal.cv_mode == 0)
        {
            rune_mode = false;
            cmd.armor_mode = true;
        }
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

void UartComm::processParamResponse(uint8_t rxbuf[])
{
    static const uint8_t len = sizeof(uart_header_t)+
            sizeof(uart_param_response_t)+
            sizeof(uart_crc_t);

    heartbeat_time = ros::Time::now();

    uart_header_t*         header   = (uart_header_t*)&rxbuf[0];
    uart_param_response_t* response = (uart_param_response_t*)&rxbuf[sizeof(uart_header_t)];

    if(
        rxbuf[0] == UART_START_BYTE &&
        verify_crc(rxbuf, len) &&
        header->type == UART_ROS_RESPONSE_ID &&
        response->content == RESPONSE_OK
      )
    {
        ROS_INFO("Param setup complete");
        comm_status = COMM_IDLE;
    }
    else
    {
        ROS_WARN("Incorrect data frame received 1");

        frame_err_cnt++;
        if(frame_err_cnt > 5)
        {
            ROS_WARN("Broken frames exceed threshold, trying re-connection...");
            toggleSyncMode();
        }
        return;
    }
}

uint8_t UartComm::sendParameters(uint8_t txbuf[])
{
    if((ros::Time::now() - last_write).toSec() < 1e-4)
        return 1;

    static const uint8_t len = sizeof(uart_header_t)+
            sizeof(uart_ros_param_t)+
            sizeof(uart_crc_t);

    uart_header_t header;
    header.start = UART_START_BYTE;
    header.type  = UART_ROS_PARAM_ID;

    uart_ros_param_t param;

    float vs_kp;
    if (!ros::param::get("/serial_node/control/VS_kp", vs_kp))
    {
        ROS_FATAL("Param VS_KP not found!");
        return -1;
    }

    float vs_kd;
    if (!ros::param::get("/serial_node/control/VS_kd", vs_kd))
    {
        ROS_FATAL("Param VS_KD not found!");
        return -1;
    }

    float vs_ff;
    if (!ros::param::get("/serial_node/control/VS_ff", vs_ff))
        vs_ff = 0.0;

    param.VS_kp = vs_kp;
    param.VS_kd = vs_kd;
    param.VS_ff = vs_ff;

    uint8_t* txptr = txbuf;
    memcpy(txptr, &header, sizeof(uart_header_t));
    txptr += sizeof(uart_header_t);
    memcpy(txptr, &param,   sizeof(uart_ros_param_t));
    this->append_crc(txbuf, len);

    this->serial_port->write(txbuf, len);
    last_write = ros::Time::now();

    return 0;
}

void UartComm::heartbeatTxProcess(void)
{
    static uint8_t txbuf[40];
    while(ros::ok())
    {
        if(comm_status > COMM_IDLE)
        {
            frame_err_cnt = 0; //Flush error counter
            ros::Duration(1).sleep();
        }
        else if(comm_status == COMM_IDLE)
        {
            sendHeartbeat(txbuf);
            ros::Duration(0.05).sleep();
        }
        else if(comm_status == COMM_SEND_PARAM)
        {
            if(sendParameters(txbuf) == -1)
            {
                ROS_FATAL("Required parameter not found!");
            }
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
        if(comm_status >= COMM_SEND_PARAM)
        {
            uint8_t rx_size = this->serial_port->read(rxbuf, length);
            if(rx_size == length)
            {
                if(comm_status > COMM_SEND_PARAM)
                    processGimbalInfo(rxbuf, true);
                else
                    processParamResponse(rxbuf);
            }

            if(check_timeout()) //Switch to sync mode
            {
                ROS_WARN("Connection lost with device, trying re-connection...");
                this->serial_port->flushInput();
                toggleSyncMode();
                ros::Duration(0.05).sleep();
            }
        }
    }
}
