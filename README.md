# ADMA Format Version: 3.3.3 - GPS Receiver for ROS

- Tested with ROS-Melodic and Ubuntu 18
Ethernet:
- Uses Boost asio for UDP Socket handling
CAN:
- Uses python-can and cantools library for receiving and decoding CAN bus messages. Tested on NVIDA Jetson AGX Xavier

**How to build the package?**

- Open a terminal in a folder of choice
- enter: mkdir catkin_ws <- creates the directory "catkin_ws"
- enter: cd catkin_ws <- enters the directory "catkin_ws"
- enter: mkdir src <- creates the directory "src"
- enter: cd src <- enters the directory "src"
- enter: git clone https://github.com/lab176344/adma_ros_driver.git <- clones the source code to the local drive
- enter: cd ..  <- goes a folder upwards
- enter: catkin_make
- enter: source devel/setup.bash

**How to use the launch files**

 - Set the port number in the ADMA_pub.launch file in the port_num_ADMA args
 - Use a port number greater than 1030 (to be set up in ADMA Web interface)
 - Set the IP number in the ADMA_pub.launch file in the ip_adress_ADMA args
  
 Ethernet:
 - roslaunch ADMA_pub_Ethernet.launch
 
 CAN:
 - roslaunch ADMA_pub_CAN.launch
 - You might have to change the parameters in the launch file accordingly to your setup.

**Accessing the ADMA Messages**

 - ROS Messages for ADMA are published in a custom message called adma under the topic "adma_data"
 
**Credits**
 - The original project can be found here:
 https://github.com/lab176344/adma_ros_driver
 
 - This project exists, thanks to these people:
 
   - https://github.com/lab176344
 
   - https://github.com/BlackForestFormula
 
   - https://github.com/Betschler
 
 
