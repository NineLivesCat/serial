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

void MasterCtrlProcess(ros::NodeHandle& nh, UartComm* comm)
{
    ros::ServiceClient armor_ctrl = nh.serviceClient<rm_vehicle_msgs::cvEnable>("/Armor_Node_Enable");
    ros::ServiceClient rune_ctrl  = nh.serviceClient<rm_vehicle_msgs::cvEnable>("/Rune_Node_Enable");

    while(ros::ok())
    {
        if(comm->cmd.rune_mode) //Switch to rune mode
        {
            rm_vehicle_msgs::cvEnable armor_en, rune_en;
            armor_en.request.enable = false;
            rune_en.request.enable  = true;

            armor_ctrl.call(armor_en);
            rune_ctrl.call(rune_en);
            comm->cmd.rune_mode  = false;
        }

        if(comm->cmd.armor_mode) //Switch to armor mode
        {
            rm_vehicle_msgs::cvEnable armor_en, rune_en;
            armor_en.request.enable = true;
            rune_en.request.enable  = false;

            rune_ctrl.call(rune_en);
            armor_ctrl.call(armor_en);
            comm->cmd.armor_mode = false;
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
    nh.param<int>   ("/serial_node/baudrate", baud,      230400);
    nh.param<int>   ("/serial_node/max_len",  max_len,       30);
    nh.param<double>("/serial_node/timeout",  timeout_ms,   0.3);

    serial::Timeout timeout(1, 1, 0, 0, 0),
                    timeout_sync = serial::Timeout::simpleTimeout(4);

    serial::Serial serial_host(port, baud, timeout_sync);
    if(serial_host.isOpen())
    {
        serial_host.flush();
        ROS_INFO("Connected to port %s", port);
    }

    UartComm comm(nh, &serial_host, baud);
    comm.gimbalInfo_pub = nh.advertise<rm_vehicle_msgs::gimbalInfo>("/rm_vehicle/gimbal_info", 10);
    comm.RC_pub = nh.advertise<rm_vehicle_msgs::RC>("/rm_vehicle/RC", 10);

    //ros::Subscriber gimbalCmd_sub = nh.subscribe("/RM_gimbal/control",10,
    //    &UartComm::gimbalCmdCallback, &comm);

    ros::Subscriber visualServo_sub = nh.subscribe("/VI_position_cmd",10,
        &UartComm::visualServoCallback, &comm);

    uint8_t rx_buffer[max_len],
            tx_buffer[max_len];
    size_t  rx_size;

    ros::Duration(0.5).sleep();
    serial_host.flush();

    boost::thread serialReadThd(boost::bind(&UartComm::gimbalInfoRxProcess, &comm));
    serialReadThd.detach();

    boost::thread heartbeatTxThd(boost::bind(&UartComm::heartbeatTxProcess, &comm));
    heartbeatTxThd.detach();

    boost::thread masterCtrlThd(boost::bind(&MasterCtrlProcess, nh, &comm));
    masterCtrlThd.detach();

    while (ros::ok())
    {
        if(comm.inSyncMode())//synchonization
        {
            uint8_t length = sizeof(uart_header_t)+
                sizeof(uart_sync_t)+sizeof(uart_crc_t);

            comm.SendSyncSeq(tx_buffer, false);
            rx_size = serial_host.read(rx_buffer, length);
            if(rx_size == length)
            {
                if(!comm.processSyncSeq(rx_buffer))
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

            comm.processGimbalInfo(rx_buffer, false);
        }

        ros::spinOnce();
    }

    serial_host.flush();
    return 0;
}
