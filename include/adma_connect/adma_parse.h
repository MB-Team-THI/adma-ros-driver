/**
  * @file adma_parse.h
  * @brief This file contains required functions for parsing the ADMA string
  * @authors Lakshman Balasubramanian
  * @date 06/08/2020
  * */

#pragma once
#include <adma_connect/Adma.h>
#include <bitset>

#ifndef ADMA_P
#define ADMA_P




void getParsedData(const std::string& local_data, adma_connect::Adma& message);
void getADMAStaticHeader(const std::string& local_data, adma_connect::Adma& message);
void getADMADynamicHeader(const std::string& local_data, adma_connect::Adma& message);
void getStatusGPS(const std::string& local_data, adma_connect::Adma& message);
void getStatusTrigger(const std::string& local_data, adma_connect::Adma& message);
void getEVKStatus(const std::string& local_data, adma_connect::Adma& message);
void getStatusCount(const std::string& local_data, adma_connect::Adma& message);
void getErrorandWarning(const std::string& local_data, adma_connect::Adma& message);
void getSensorBodyXYZ(const std::string& local_data, adma_connect::Adma& message);
void getRatesBodyXYZ(const std::string& local_data, adma_connect::Adma& message);
void getRatesHorizontalXYZ(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBody(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHor(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI1(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI2(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI3(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI4(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI5(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI6(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI7(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationBodyPOI8(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI1(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI2(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI3(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI4(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI5(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI6(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI7(const std::string& local_data, adma_connect::Adma& message);
void getAccelerationHorPOI8(const std::string& local_data, adma_connect::Adma& message);
void getExternalVelocityAnalog(const std::string& local_data, adma_connect::Adma& message);
void getExternalVecovityDigPulses(const std::string& local_data, adma_connect::Adma& message);
void getExternalVelocityCorrected(const std::string& local_data, adma_connect::Adma& message);
void getBarometerPressure(const std::string& local_data, adma_connect::Adma& message);
void getBarometerHeight(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuos(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI1(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI2(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI3(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI4(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI5(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI6(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI7(const std::string& local_data, adma_connect::Adma& message);
void getMiscellaneuosPOI8(const std::string& local_data, adma_connect::Adma& message);
void getTriggers(const std::string& local_data, adma_connect::Adma& message);
void getSystemData(const std::string& local_data, adma_connect::Adma& message);
void getGPSAbs(const std::string& local_data, adma_connect::Adma& message);
void getGPSPosRel(const std::string& local_data, adma_connect::Adma& message);
void getGPSEPE(const std::string& local_data, adma_connect::Adma& message);
void getGPSVelFrame(const std::string& local_data, adma_connect::Adma& message);
void getGPSVelEVE(const std::string& local_data, adma_connect::Adma& message);
void getGPSTimeUTC(const std::string& local_data, adma_connect::Adma& message);
void getGPSAuxData1(const std::string& local_data, adma_connect::Adma& message);
void getGPSAuxData2(const std::string& local_data, adma_connect::Adma& message);
void getINSAngleGPSCOG(const std::string& local_data, adma_connect::Adma& message);
void getGPSHeight(const std::string& local_data, adma_connect::Adma& message);
void getGPSDualAntTimeUTC(const std::string& local_data, adma_connect::Adma& message);
void getGPSDualAntAngle(const std::string& local_data, adma_connect::Adma& message);
void getGPSDualAntAngleETE(const std::string& local_data, adma_connect::Adma& message);
void getINSPositionHeight(const std::string& local_data, adma_connect::Adma& message);
void getINSPositionPOI(const std::string& local_data, adma_connect::Adma& message);
void getINSTimeUTC(const std::string& local_data, adma_connect::Adma& message);
void getINSPositionAbs(const std::string& local_data, adma_connect::Adma& message);
void getINSPosRel(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI1(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI2(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI3(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI4(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI5(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI6(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI7(const std::string& local_data, adma_connect::Adma& message);
void getINSPosPOI8(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZ(const std::string& local_data, adma_connect::Adma& message);
void getINSVelFrameXYZ(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS1(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS2(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS3(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS4(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS5(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS6(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS7(const std::string& local_data, adma_connect::Adma& message);
void getINSVelHorXYZPOS8(const std::string& local_data, adma_connect::Adma& message);
void getINSEPE(const std::string& local_data, adma_connect::Adma& message);
void getINSEVEandETE(const std::string& local_data, adma_connect::Adma& message);
void getAnalog(const std::string& local_data, adma_connect::Adma& message);
void getKalmanFilter(const std::string& local_data, adma_connect::Adma& message);
void getGNSSReceiver(const std::string& local_data, adma_connect::Adma& message);
bool getBit(unsigned char byte, int position);

#endif
