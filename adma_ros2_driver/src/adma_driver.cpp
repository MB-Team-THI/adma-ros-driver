#include <rclcpp/rclcpp.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros_driver/adma_parse.h"

using namespace std::chrono_literals;

/** \namespace BOOST UDP link*/
using boost::asio::ip::udp;
/** \brief IP address to which ADMA broadcasts */
const boost::asio::ip::address address = boost::asio::ip::address::from_string("0.0.0.0");

/** \brief Length of the stream */
size_t len = 0;
/** \brief Check the timings */
bool performance_check = true;
/// \brief  Main function
/// \param  argc An integer argument count of the command line arguments
/// \param  argv An argument vector of the command line arguments
/// \param  loop_rate Set the frequency of publising in Hz
/// \return an integer 0 upon exit success
int main(int argc, char **argv)
{
  /* Initialize node */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("adma_ros_driver");

  /* Port number to which ADMA broadcasts */
  /** \get port number list from launch file */
  const unsigned short port = 3333; //TODO: Parameterize
  /* Initiliaze publisher */
  auto adma_data_publisher = node->create_publisher<adma_msgs::msg::AdmaData>("adma/data", 1);
  auto navsat_fix_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 1);
  auto heading_publisher = node->create_publisher<std_msgs::msg::Float64>("gps/heading", 1);
  auto velocity_publisher = node->create_publisher<std_msgs::msg::Float64>("gps/velocity", 1);

  /* Create an IO Service wit the OS given the IP and the port */
  boost::asio::io_service io_service;
  /* Establish UDP connection*/
  udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(address, port);
  std::cout << "Local bind " << local_endpoint << std::endl;

  /* Socket handling */
  udp::socket socket(io_service);
  socket.open(udp::v4());
  socket.bind(local_endpoint);
  udp::endpoint sender_endpoint;

  /* Endless loop while ROS is ok*/
  while (rclcpp::ok())
  {
    /* The length of the stream from ADMA is 856 bytes */
    boost::array<char, 856> recv_buf;
    len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
    /* Prepare for parsing */
    std::string local_data(recv_buf.begin(), recv_buf.end());
    /* Load the messages on the publishers */
    adma_msgs::msg::AdmaData message;
    sensor_msgs::msg::NavSatFix message_fix;
    message_fix.header.stamp = node->now();
    message_fix.header.frame_id = "adma";
    std_msgs::msg::Float64 message_heading;
    std_msgs::msg::Float64 message_velocity;
    message.timemsec = node->get_clock()->now().seconds() * 1000;
    message.timensec = node->get_clock()->now().nanoseconds();
    getparseddata(local_data, message, message_fix, message_heading, message_velocity);
    /* publish the ADMA message */

    adma_data_publisher->publish(message);
    navsat_fix_publisher->publish(message_fix);
    heading_publisher->publish(message_heading);
    velocity_publisher->publish(message_velocity);
    double grab_time = node->get_clock()->now().seconds();

    if (performance_check)
    {
      char ins_time_msec[] = {local_data[584], local_data[585], local_data[586], local_data[587]};
      memcpy(&message.instimemsec, &ins_time_msec, sizeof(message.instimemsec));
      float weektime = message.instimeweek;
      RCLCPP_INFO(node->get_logger(), "%f ", ((grab_time * 1000) - (message.instimemsec + 1592697600000)));
    }
    
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
