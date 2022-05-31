/**
  * @file adma_connect.cpp
  * @brief This file contains required definitions and functions for
  * receviing information from ADMA sensor with data format Version: 3.3.2.0
  * @authors Lakshman Balasubramanian
  * @date 06/08/2020
  * */


#include "ros/ros.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <adma_connect/Adma.h>
#include <adma_connect/adma_parse.h>
#include "std_msgs/String.h"

/** \namespace BOOST UDP link*/
using boost::asio::ip::udp;

/** \brief Length of the stream */
size_t len = 0;
/** \brief Check the timings */
bool performance_check = 0;
/** \brief speed of accessing the data */
#define loopSpeed 100
/// \file
/// \brief  Main function
/// \param  argc An integer argument count of the command line arguments
/// \param  argv An argument vector of the command line arguments
/// \param  loop_rate Set the frequency of publising in Hz
/// \return an integer 0 upon exit success
int main(int argc, char **argv)
{
  /* Initialize node */
  ros::init(argc, argv, "adma_connect_pkg");
  ros::NodeHandle nh("~");
  /* Port number to which ADMA broadcasts */
  /** \get port number list from launch file */
  std::string port_num_ADMA;
  std::string ip_adress_ADMA;
  if(!nh.getParam("port_num_ADMA", port_num_ADMA))
  {
     ROS_INFO("Missing Portnumber (see ADMA_pub_Ethernet.launch file!)");
  }
  if(!nh.getParam("ip_adress_ADMA", ip_adress_ADMA))
  {
     ROS_INFO("Missing IP Adress (see ADMA_pub_Ethernet.launch file!)");
  }

  /** \brief Port Number to which ADMA broadcasts */
  const unsigned short port = static_cast<unsigned short>(std::strtoul(port_num_ADMA.c_str(), NULL, 0));
  /** \brief IP address to which ADMA broadcasts */
  const boost::asio::ip::address address = boost::asio::ip::address::from_string(ip_adress_ADMA);

  /* Initiliaze publisher */
  ros::Publisher  publisher_  = nh.advertise<adma_connect::Adma>("adma_data",1);
  ros::Publisher  raw_publisher_  = nh.advertise<std_msgs::String>("raw_adma_data",1);

  /* Initilaize loop rate */
  ros::Rate loop_rate(loopSpeed);
  /* Create an IO Service wit the OS given the IP and the port */
  boost::asio::io_service io_service;
  /* Establish UDP connection*/
  udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(address, port);
  std::cout << "Local bind " << local_endpoint << std::endl;
  unsigned int seq = 0;
  /* Endless loop until ROS is ok*/
  while (ros::ok())
  {
    /* Socket handling */
    udp::socket socket(io_service);
    socket.open(udp::v4());
    socket.bind(local_endpoint);
    /* The length of the stream from ADMA is 856 bytes */
    boost::array<char, 856> recv_buf;
    udp::endpoint sender_endpoint;
    len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);    
    /* Prepare for parsing */
    std::string local_data(recv_buf.begin(), recv_buf.end());
    /* Load the messages on the publisers */
    std_msgs::String msg_raw_adma;
    msg_raw_adma.data = local_data;
    adma_connect::Adma message;
    getParsedData(local_data,message);
    
    /* publish the ADMA message */
    // fill timestamp and increment seq counter
    message.header.stamp = ros::Time::now();
    message.header.seq = seq++;
    msg_raw_adma.header.stamp = ros::Time::now();
    msg_raw_adma.header.seq = seq++;
    /* Get current time */
    double grab_time = ros::Time::now().toSec();

    if (performance_check)
    {
    char INS_Time_msec[] = {local_data[584],local_data[585],local_data[586],local_data[587]};
    memcpy(&message.INSTimemsec , &INS_Time_msec, sizeof(message.INSTimemsec));
    float weektime = message.INSTimeWeek;
    ROS_INFO("%f ", ((grab_time*1000)-(message.INSTimemsec+1592697600000)));
    }
    message.TimeMsec = ros::Time::now().toSec()*1000;
    message.TimeNsec = ros::Time::now().toNSec();
    
    /* publish ADMA message */
    publisher_.publish(message);
    raw_publisher_.publish(msg_raw_adma);
    /* Loop rate maintain*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
