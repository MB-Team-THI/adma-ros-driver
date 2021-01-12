
**ADMA - GPS Receiver for ROS**

- Tested with ROS-Melodic and Ubuntu 18
- Uses Boost asio for UDP Socket handling

**How to buld the package?**

- Open a terminal in a folder of choice
- mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
- git clone https://github.com/lab176344/adma_ros_driver.git
- cd .. 
- catkin_make
- source devel/setup.bash

**Using the launch file**

 - Set the port number in the ADMA_pub.launch file in the port_no args
 - Use a port number greater than 1030 (to be set up in ADMA Web interface)
 - roslaunch ADMA_pub.launch

**Accessing the ADMA Messages**

 - ROS Messages for ADMA are published in a custom message called adma under the topic "adma_data"
 
**Disclaimer**
 
 - The drivers are written for the use of the working group, hence the data used by my working group is only verified, all the parsing is not verfied and the package is done in a prototype fashion. Feel free to correct it if there are any issues
