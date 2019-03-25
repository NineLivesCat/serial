#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "UartComm.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh("~");

    std::string port;
    int      baud, max_len;
    double   timeout_ms;

    nh.param<std::string>("/serial/port", port, "/dev/ttyUSB0");
    nh.param<int>   ("/serial/baudrate", baud,      230400);
    nh.param<int>   ("/serial/max_len",  max_len,       30);
    nh.param<double>("/serial/timeout",  timeout_ms,   0.3);

    serial::Timeout timeout(1, 1, 0, 0, 0),
                    timeout_sync = serial::Timeout::simpleTimeout(50);

    serial::Serial serial_host(port, baud, timeout_sync);
    if(serial_host.isOpen())
    {
        serial_host.flush();
        ROS_INFO("Connected to port %s", port);
    }

    UartComm comm(nh, &serial_host);
    comm.gimbalInfo_pub = nh.advertise<rm_vehicle_msgs::gimbalInfo>("/rm_vehicle/gimbal_info", 10);
    comm.RC_pub = nh.advertise<rm_vehicle_msgs::RC>("/rm_vehicle/RC", 10);

    ros::Subscriber gimbalCmd_sub = nh.subscribe("/RM_gimbal/control",10,
        &UartComm::gimbalCmdCallback, &comm);

    uint8_t rx_buffer[max_len],
            tx_buffer[max_len];
    size_t  rx_size;

    serial_host.flush();
    ros::Duration(0.5).sleep();

    while (ros::ok())
    {
        if(comm.inSyncMode())//synchonization
        {
            uint8_t length = comm.packSyncSeq(tx_buffer, false);
            serial_host.write(tx_buffer, length);
            rx_size = serial_host.read(rx_buffer, length);
            if(rx_size == length)
            {
                if(!comm.processSyncSeq(rx_buffer))
                {
                    ros::Duration(0.01).sleep();
                    comm.packSyncSeq(tx_buffer, true);
                    serial_host.write(tx_buffer, length);
                    serial_host.flush();
                    comm.toggleRXMode();
                }
            }
            else
                ROS_WARN("Timestamp sync unsuccessful, check UART connection");

            comm.processGimbalInfo(rx_buffer, false);
            ros::Duration(0.005).sleep();
        }
        else
        {
            uint8_t length = sizeof(uart_header_t)+
                    sizeof(uart_gimbal_info_t)+sizeof(uart_crc_t);
            rx_size = serial_host.read(rx_buffer, length);
            if(rx_size == length)
                comm.processGimbalInfo(rx_buffer, true);

            if(comm.inIdleMode())
                comm.sendHeartbeat(tx_buffer);

            if(comm.check_timeout()) //Switch to sync mode
            {
                ROS_WARN("Connection lost with device, trying re-connection...");
                serial_host.flush();
                comm.toggleSyncMode();
            }
        }
        ros::spinOnce();
    }

    return 0;
}
