#include <rclcpp/rclcpp.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "adma_msgs/msg/adma_data.hpp"
#include "adma_ros_driver/adma_parse.h"

/** \namespace BOOST UDP link*/
using boost::asio::ip::udp;

/* Create an IO Service with the OS */
boost::asio::io_service io_service;

class ADMA_driver : public rclcpp::Node
{
public:
  ADMA_driver() : Node("adma_ros_driver"), socket(io_service)
  {
    /* Declare Parameters */
    this->declare_parameter<std::string>("destination_ip", std::string("192.168.88.100"));
    this->declare_parameter<int>("destination_port", 1040);
    this->declare_parameter<bool>("performance_check", false);

    this->declare_parameter<std::string>("adma_data_topic", std::string("adma/data"));
    this->declare_parameter<std::string>("adma_navsatfix_topic", std::string("adma/fix"));
    this->declare_parameter<std::string>("adma_imu_topic", std::string("adma/imu"));
    this->declare_parameter<std::string>("adma_heading_topic", std::string("adma/heading"));
    this->declare_parameter<std::string>("adma_velocity_topic", std::string("adma/velocity"));

    this->declare_parameter<std::string>("gnss_frame", std::string("gnss_link"));
    this->declare_parameter<std::string>("imu_frame", std::string("imu_link"));

    /* IP address to which ADMA broadcasts */
    std::string destination_ip = this->get_parameter("destination_ip").as_string();
    int destination_port = this->get_parameter("destination_port").as_int();
    const boost::asio::ip::address destination_ip_address = boost::asio::ip::address::from_string(destination_ip);

    /* Establish UDP connection*/
    udp::endpoint local_endpoint = udp::endpoint(destination_ip_address, destination_port);
    std::cout << "Local bind " << local_endpoint << std::endl;

    /* Socket handling */
    socket.open(udp::v4());
    socket.bind(local_endpoint);

    /* Initialize publishers */
    std::string adma_data_topic = this->get_parameter("adma_data_topic").as_string();
    std::string adma_navsatfix_topic = this->get_parameter("adma_navsatfix_topic").as_string();
    std::string adma_imu_topic = this->get_parameter("adma_imu_topic").as_string();
    std::string adma_heading_topic = this->get_parameter("adma_heading_topic").as_string();
    std::string adma_velocity_topic = this->get_parameter("adma_velocity_topic").as_string();
    adma_data_publisher = this->create_publisher<adma_msgs::msg::AdmaData>(adma_data_topic, 1);
    navsat_fix_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(adma_navsatfix_topic, 1);
    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(adma_imu_topic, 1);
    heading_publisher = this->create_publisher<std_msgs::msg::Float64>(adma_heading_topic, 1);
    velocity_publisher = this->create_publisher<std_msgs::msg::Float64>(adma_velocity_topic, 1);
  }

  void parse_and_publish()
  {
    /* The length of the stream from ADMA is 856 bytes */
    boost::array<char, 856> recv_buf;
    socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
    /* Prepare for parsing */
    std::string local_data(recv_buf.begin(), recv_buf.end());
    
    /* Load the messages on the publishers */
    adma_msgs::msg::AdmaData message_admadata;
    message_admadata.timemsec = this->get_clock()->now().seconds() * 1000;
    message_admadata.timensec = this->get_clock()->now().nanoseconds();
    sensor_msgs::msg::NavSatFix message_navsatfix;
    message_navsatfix.header.stamp = this->get_clock()->now();
    message_navsatfix.header.frame_id = this->get_parameter("gnss_frame").as_string();
    sensor_msgs::msg::Imu message_imu;
    message_imu.header.stamp = this->get_clock()->now();
    message_imu.header.frame_id = this->get_parameter("imu_frame").as_string();

    std_msgs::msg::Float64 message_heading;
    std_msgs::msg::Float64 message_velocity;
    getparseddata(local_data, message_admadata, message_navsatfix, message_imu, message_heading, message_velocity);

    /* publish the ADMA message */
    adma_data_publisher->publish(message_admadata);
    navsat_fix_publisher->publish(message_navsatfix);
    imu_publisher->publish(message_imu);
    heading_publisher->publish(message_heading);
    velocity_publisher->publish(message_velocity);

    if (this->get_parameter("performance_check").as_bool())
    {
      double grab_time = this->get_clock()->now().seconds();
      char ins_time_msec[] = { local_data[584], local_data[585], local_data[586], local_data[587] };
      memcpy(&message_admadata.instimemsec, &ins_time_msec, sizeof(message_admadata.instimemsec));
      float weektime = message_admadata.instimeweek;
      RCLCPP_INFO(this->get_logger(), "%f ", ((grab_time * 1000) - (message_admadata.instimemsec + 1592697600000)));
    }
  }

private:
  udp::socket socket;
  udp::endpoint sender_endpoint;

  rclcpp::Publisher<adma_msgs::msg::AdmaData>::SharedPtr adma_data_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher;
};

/// \brief  Main function
/// \param  argc An integer argument count of the command line arguments
/// \param  argv An argument vector of the command line arguments
/// \return an integer 0 upon exit success
int main(int argc, char** argv)
{
  /* Initialize node */
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ADMA_driver>();

  /* Endless loop while ROS is ok*/
  while (rclcpp::ok())
  {
    node->parse_and_publish();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
