#pragma once
#include "adma_msgs/msg/adma_data.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <bitset>




void getparseddata(const std::string& local_data, 
                   adma_msgs::msg::AdmaData& message, 
                   sensor_msgs::msg::NavSatFix& msg_fix, 
                   sensor_msgs::msg::Imu& msg_imu, 
                   std_msgs::msg::Float64& msg_heading, 
                   std_msgs::msg::Float64& msg_velocity);
void getadmastaticheader(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getadmadynamicheader(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getstatusgps(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix);
void getstatustrigger(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getevkstatus(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getstatuscount(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void geterrorandwarning(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getsensorbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::Imu& msg_imu);
void getratesbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getrateshorizontalxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbody(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhor(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationbodypoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getaccelerationhorpoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getexternalvelocityanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getexternalvecovitydigpulses(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getexternalvelocitycorrected(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getbarometerpressure(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getbarometerheight(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuos(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getmiscellaneuospoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void gettriggers(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getsystemdata(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsabs(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsvelframe(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_velocity);
void getgpsveleve(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsauxdata1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsauxdata2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsanglegpscog(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::Imu& msg_imu, std_msgs::msg::Float64& msg_heading);
void getgpsheight(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualanttimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualantangle(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgpsdualantangleete(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspositionheight(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix);
void getinspositionpoi(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspositionabs(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix);
void getinsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinspospoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelframexyz(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos1(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos2(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos3(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos4(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos5(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos6(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos7(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsvelhorxyzpos8(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getinsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix);
void getinseveandete(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::Imu& msg_imu);
void getanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getkalmanfilter(const std::string& local_data, adma_msgs::msg::AdmaData& message);
void getgnssreceiver(const std::string& local_data, adma_msgs::msg::AdmaData& message);
bool getbit(unsigned char byte, int position);
