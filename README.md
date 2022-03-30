# ADMA Format Version: 3.3.3 - GPS Receiver for ROS

- Tested with ROS-Melodic and Ubuntu 18
Ethernet:
- Uses Boost asio for UDP Socket handling
CAN:
- Uses python-can and cantools library for receiving and decoding CAN bus messages. Tested on NVIDA Jetson AGX Xavier

In the following, it is described how to install the ROS driver from the ground up. 
Note: If you already have a ROS installation, jump to "How to build the ADMA ROS-Driver package?"

## How to install ROS?

- Go to http://wiki.ros.org/noetic/Installation/Ubuntu
- Follow steps 1.1, 1.2 and 1.3
- From step 1.4, on the Terminal enter: sudo apt update
- From step 1.4, at least a "Desktop installation". On the Terminal enter: sudo apt install ros-noetic-desktop

### How to update the source folder?

- Open the Terminal and enter: 
```
gedit ~/.bashrc
```
This opens the text file to alter the source folder.
- Once the text file is open, go to the bottom and add the two following lines:
```
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash"
```
- To check if ROS is installed properly, on the terminal enter: 
```
roscore
```
- If ROS is correclty installed, no error messages should appear. 

## Installation of the ADMA ROS-Driver.

### How to build the ADMA ROS-Driver package?

- Open a terminal in a folder of choice. This folder will be the installation folder.
- Enter:
```
mkdir catkin_ws
``` 
<- This creates the directory "catkin_ws"
- Enter: 
```
cd catkin_ws
```
<- enters the directory "catkin_ws"
- Enter: 
```
mkdir src
```
<- creates the directory "src"
- Enter: 
```
cd src
```
<- enters the directory "src"
- Enter: 
```
git clone https://github.com/lab176344/adma_ros_driver.git
```
<- clones the source code to the local drive
- Enter: 
```
cd ..
```
<- goes a folder upwards
- Enter: 
```
catkin_make
```
- Enter: 
```
source devel/setup.bash
```

### How to configure the AMDA ROS Driver?

The ADMA ROS-Driver has to be configured to receive data from the ADMA. This means that the IP and the port have to be updated in the code according to the current ADMA configuration.

- Go to the ADMA web interface, and note the Port and IP numbers: 
- 
![Screenshot from 2022-03-28 17-05-17 - Kopie](https://user-images.githubusercontent.com/60926891/160432015-a6e6248b-3799-4d77-b101-226315e801bd.png)

![Screenshot from 2022-03-28 17-07-14 - Kopie](https://user-images.githubusercontent.com/60926891/160432024-5577d8b5-18d3-4f0a-8d6e-aed389e82e14.png)

- Open the folder where the ADMA ROS-driver was installed.
- Inside the installation folder, enter the folder \catkin_ws\src\adma_ros_driver\launch\
- Inside the folder, open the file: ADMA_pub_Ethernet.launch
- Update port and IP numbers according to the configuration in the ADMA web interface:

![Screenshot from 2022-03-28 17-20-54 - Kopie](https://user-images.githubusercontent.com/60926891/160431840-ae510525-2a5f-41b6-92ef-0c723bd96711.png)

Note that the computer where the ADMA ROS-Driver is running should also be configured:
- The IP should be assigned
- The PC and its firewall (if present) should be able to receive data from the corresponding IP and Port. 

### How to use the launch files?
  
 #### When connected via Ethernet:
 - Enter the installation folder
 - Open a terminal and enter: 
 ```
 source devel/setup.bash
 ```
 - Enter: 
 ```
 roslaunch adma_connect ADMA_pub_Ethernet.launch
 ```
 
 ####  When connected via CAN:
 - Enter the installation folder
 - Open a terminal and enter: 
 ```
 roslaunch ADMA_pub_CAN.launch
 ```
 - You might have to change the parameters in the launch file accordingly to your setup.

### Accessing the ADMA Messages

 - The ROS Messages from the ADMA are published in a custom message called adma under the topic "adma_data". 
 There are two manners to test the data reception: a) by means of rostopic echo, and b) via rqt.
 
 #### a) By means of rostopic echo:
 - Enter the installation folder
 - Open a terminal and enter: 
 ```
 source devel/setup.bash
 ```
 - Enter: 
 ```
 rostopic echo /adma_connect/adma_data
 ```
 - If the information is being received, it should appear on the terminal
 
 #### b) Via rqt:
 - Enter the installation folder
 - Open a terminal and enter: 
 ```
 source devel/setup.bash
 ```
 - Enter: 
 ```
 rqt_topic
 ```
 - The list of topic will be displayed
 - Select the "/adma_connect/adma_data" topic.
 - If the information is being received, it should appear. 
 
# Credits
 - The original project can be found here:
 https://github.com/lab176344/adma_ros_driver
 
 - This project exists, thanks to these people:
 
   - https://github.com/lab176344
 
   - https://github.com/BlackForestFormula
 
   - https://github.com/Betschler
 
 
