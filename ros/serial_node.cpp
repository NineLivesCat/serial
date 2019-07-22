#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "UartComm.h"
#include "rm_vehicle_msgs/cvEnable.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

bool use_sync;
static bool armor_ok = false;
static bool rune_ok  = false;

void MasterCtrlProcess(ros::NodeHandle& nh, UartComm* comm)
{
    ros::ServiceClient armor_ctrl = nh.serviceClient<rm_vehicle_msgs::cvEnable>("/Armor_Node_Enable");
    ros::ServiceClient rune_ctrl  = nh.serviceClient<rm_vehicle_msgs::cvEnable>("/Rune_Node_Enable");

    bool use_armor, use_rune;
    nh.param<bool>  ("/serial_node/master/use_armor", use_armor, true);
    nh.param<bool>  ("/serial_node/master/use_rune" , use_rune , true);
    //wait for cv nodes to start
    while(ros::ok())
    {
        ros::param::get("/armor_detection_node/OK", armor_ok);
        ros::param::get("/rune_detection_node/OK" , rune_ok);
        if((!use_armor || armor_ok) && (!use_rune || rune_ok))
            break;

        ros::Duration(0.1).sleep();
    }

    rm_vehicle_msgs::cvEnable armor_en, rune_en;

    ros::Duration(1).sleep();
    armor_en.request.enable = false;
    armor_ctrl.call(armor_en);
    ros::Duration(1).sleep();

    int robot_type;
    nh.param<int>("/armor_detection_node/robot_type", robot_type, 0);
    comm->cmd.robot_hero = robot_type == 1;

    //enable no-gimbal debug mode
    bool Narmor_debug, Nrune_debug;
    nh.param<bool>("/armor_detection_node/debug_off", Narmor_debug, true);
    nh.param<bool>("/rune_detection_node/debug_off" , Nrune_debug,  true);

    //enable camera-only debug mode
    if(comm->inSyncMode())
    {
        if(use_armor && !Narmor_debug)
        {
            armor_en.request.use_judge_color = false;
            armor_en.request.enable = true;
            armor_ctrl.call(armor_en);
        }
        else if(use_rune && !Nrune_debug)
        {
            rune_en.request.use_judge_color = false;
            rune_en.request.enable  = true;
            rune_en.request.rune_mode = RUNE_UNDEFINED;
            rune_ctrl.call(rune_en);
        }
    }

    while(ros::ok())
    {
        rune_en.request.rune_mode = comm->cmd.rune_type;
        if(comm->cmd.robot_color == ROBOT_TEAM_UNDEFINED)
        {
            armor_en.request.use_judge_color = false;
            rune_en. request.use_judge_color = false;
        }
        else
        {
            armor_en.request.use_judge_color = true;
            rune_en. request.use_judge_color = true;
            armor_en.request.target_blue = comm->cmd.robot_color == ROBOT_TEAM_RED;
            rune_en. request.target_blue = comm->cmd.robot_color == ROBOT_TEAM_BLUE;
        }

        if(comm->cmd.switch_flag)
        {
            if(use_armor && comm->cmd.cv_mode == CV_MODE_ARMOR)
            {
                armor_en.request.enable = true;
                rune_en.request.enable  = false;

                rune_ctrl.call(rune_en);
                armor_ctrl.call(armor_en);
            }
            if(use_rune && comm->cmd.cv_mode == CV_MODE_RUNE)
            {
                armor_en.request.enable = false;
                rune_en.request.enable  = true;

                armor_ctrl.call(armor_en);
                rune_ctrl.call(rune_en);
            }

            comm->cmd.switch_flag = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh("~");

    std::string port;
    int      baud, max_len;
    double   timeout_ms;

    nh.param<std::string>("/serial_node/port", port, "/dev/ttyUSB0");
    nh.param<bool>  ("/serial_node/use_sync", use_sync,    true);
    nh.param<int>   ("/serial_node/baudrate", baud,      230400);
    nh.param<int>   ("/serial_node/max_len",  max_len,       30);
    nh.param<double>("/serial_node/timeout",  timeout_ms,   0.3);

    serial::Serial serial_host(port, baud, serial::Timeout::simpleTimeout(4));
    if(serial_host.isOpen())
    {
        serial_host.flush();
        ROS_INFO("Connected to port %s", port.c_str());
    }

    UartComm comm(nh, &serial_host, baud);
    comm.gimbalInfo_pub = nh.advertise<rm_vehicle_msgs::gimbalInfo>("/rm_vehicle/gimbal_info", 3);
    comm.RC_pub = nh.advertise<rm_vehicle_msgs::RC>("/rm_vehicle/RC", 3);

    ros::Subscriber visualServo_sub = nh.subscribe("/VI_position_cmd",10,
        &UartComm::visualServoCallback, &comm);
    ros::Subscriber gimbalCmd_pub = nh.subscribe("/Hunter_Killer_cmd",10,
        &UartComm::gimbalCmdCallback, &comm);

    uint8_t rx_buffer[max_len],
            tx_buffer[max_len];
    size_t  rx_size;

    ros::Duration(0.5).sleep();
    serial_host.flush();

    //boost::thread serialReadThd(boost::bind(&UartComm::gimbalInfoRxProcess, &comm));
    //serialReadThd.detach();

    boost::thread heartbeatTxThd(boost::bind(&UartComm::heartbeatTxProcess, &comm));
    boost::thread masterCtrlThd(boost::bind(&MasterCtrlProcess, nh, &comm));

    int rx_cnt = 0;
    while (ros::ok())
    {
        if(comm.inSyncMode())//synchonization
        {
            static const uint8_t length = sizeof(uart_header_t)+
                sizeof(uart_sync_t)+sizeof(uart_crc_t);

            comm.processGimbalInfo(rx_buffer, false);
            comm.SendSyncSeq(tx_buffer, false);
            rx_size = serial_host.read(rx_buffer, length);
            if(rx_size == length)
            {
                if(!comm.processSyncSeq(rx_buffer, use_sync))
                {
                    comm.SendSyncSeq(tx_buffer, true);
                    comm.toggleRXMode();
                }
            }
            else
                comm.toggleSyncMode();

            static uint32_t noData_cnt;
            if(!rx_size && !(++noData_cnt % 50))
                ROS_WARN("No data received from embedded device");
        }
        else
        {
            static const uint8_t length = sizeof(uart_header_t)+
                    sizeof(uart_gimbal_info_t)+sizeof(uart_crc_t);

            if(comm.getStatus() >= UartComm::COMM_SEND_PARAM)
            {
                rx_size = serial_host.read(rx_buffer, length);

                if(rx_size == length)
                {
                    if(comm.getStatus() > UartComm::COMM_SEND_PARAM)
                        comm.processGimbalInfo(rx_buffer, true);
                    else
                        comm.processParamResponse(rx_buffer);
                }
                else
                    ROS_WARN("Incorrect frame received 0: %d", rx_size - length);

                if(comm.check_timeout()) //Switch to sync mode
                {
                    ROS_WARN("Connection lost with device, trying re-connection...");
                    serial_host.flushInput();
                    comm.toggleSyncMode();
                    ros::Duration(0.05).sleep();
                }
            }
        }

        ros::spinOnce();
    }

    serial_host.flush();
    return 0;
}
