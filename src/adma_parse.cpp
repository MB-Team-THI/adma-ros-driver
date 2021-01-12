/**
  * @file adma_parse.cpp
  * @brief This file contains required functions for parsing the ADMA string
  * @authors Lakshman Balasubramanian
  * @date 06/08/2020
  * */

#include "adma_connect/adma_parse.h"


/// \file
/// \brief  getParsedData function - entry function
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getParsedData(const std::string& local_data, adma_connect::Adma& message)
{
    getADMAStaticHeader(local_data,message);
    getADMADynamicHeader(local_data,message);
    getStatusGPS(local_data,message);
    getStatusTrigger(local_data,message);
    getStatusCount(local_data,message);
    getEVKStatus(local_data,message);
    getErrorandWarning(local_data,message);
    getSensorBodyXYZ(local_data,message);
    getRatesBodyXYZ(local_data,message);
    getGPSAbs(local_data,message);
    getRatesHorizontalXYZ(local_data,message);
    getAccelerationBodyPOI1(local_data,message);
    getAccelerationBodyPOI2(local_data,message);
    getAccelerationBodyPOI3(local_data,message);
    getAccelerationBodyPOI4(local_data,message);
    getAccelerationBodyPOI5(local_data,message);
    getAccelerationBodyPOI6(local_data,message);
    getAccelerationBodyPOI7(local_data,message);
    getAccelerationBodyPOI8(local_data,message);
    getAccelerationHorPOI1(local_data,message);
    getAccelerationHorPOI2(local_data,message);
    getAccelerationHorPOI3(local_data,message);
    getAccelerationHorPOI4(local_data,message);
    getAccelerationHorPOI5(local_data,message);
    getAccelerationHorPOI6(local_data,message);
    getAccelerationHorPOI7(local_data,message);
    getAccelerationHorPOI8(local_data,message);
    getExternalVelocityAnalog(local_data,message);
    getExternalVecovityDigPulses(local_data,message);
    getExternalVelocityCorrected(local_data,message);
    getBarometerPressure(local_data,message);
    getBarometerHeight(local_data,message);
    getMiscellaneuos(local_data,message);
    getMiscellaneuosPOI1(local_data,message);
    getMiscellaneuosPOI2(local_data,message);
    getMiscellaneuosPOI3(local_data,message);
    getMiscellaneuosPOI4(local_data,message);
    getMiscellaneuosPOI5(local_data,message);
    getMiscellaneuosPOI6(local_data,message);
    getMiscellaneuosPOI7(local_data,message);
    getMiscellaneuosPOI8(local_data,message);
    getTriggers(local_data,message);
    getSystemData(local_data,message);
    getGPSPosRel(local_data,message);
    getGPSEPE(local_data,message);
    getGPSVelFrame(local_data,message);
    getGPSVelEVE(local_data,message);
    getGPSTimeUTC(local_data,message);
    getGPSAuxData1(local_data,message);
    getGPSAuxData2(local_data,message);
    getINSAngleGPSCOG(local_data,message);
    getGPSHeight(local_data,message);
    getGPSDualAntTimeUTC(local_data,message);
    getGPSDualAntAngle(local_data,message);
    getGPSDualAntAngleETE(local_data,message);
    getINSPositionHeight(local_data,message);
    getINSPositionPOI(local_data,message);
    getINSTimeUTC(local_data,message);
    getINSPositionAbs(local_data,message);
    getINSPosRel(local_data,message);
    getINSPosPOI1(local_data,message);
    getINSPosPOI2(local_data,message);
    getINSPosPOI3(local_data,message);
    getINSPosPOI4(local_data,message);
    getINSPosPOI5(local_data,message);
    getINSPosPOI6(local_data,message);
    getINSPosPOI7(local_data,message);
    getINSPosPOI8(local_data,message);
    getINSVelHorXYZ(local_data,message);
    getINSVelFrameXYZ(local_data,message);
    getINSVelHorXYZPOS1(local_data,message);
    getINSVelHorXYZPOS2(local_data,message);
    getINSVelHorXYZPOS3(local_data,message);
    getINSVelHorXYZPOS4(local_data,message);
    getINSVelHorXYZPOS5(local_data,message);
    getINSVelHorXYZPOS6(local_data,message);
    getINSVelHorXYZPOS7(local_data,message);
    getINSVelHorXYZPOS8(local_data,message);
    getINSEPE(local_data,message);
    getINSEVEandETE(local_data,message);
    getAnalog(local_data,message);
    getKalmanFilter(local_data,message);
    getGNSSReceiver(local_data,message);

}

/// \file
/// \brief  getADMAStaticHeader function - ADMA Static Header Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getADMAStaticHeader(const std::string& local_data, adma_connect::Adma& message)
{
    char GeneSysID[] = {local_data[0],local_data[1],local_data[2],local_data[3]};
    char FormatID[] = {local_data[8],local_data[9],local_data[10],local_data[11]};
    memcpy(&message.FormatID , &FormatID, sizeof(message.FormatID));
    char SerialNo[] = {local_data[32],local_data[33],local_data[34],local_data[35]};
    memcpy(&message.SerialNo , &SerialNo, sizeof(message.SerialNo));
    message.GeneSysID =  GeneSysID;
    std::stringstream ss_HV;
    ss_HV <<  int(local_data[4]) << int(local_data[5]) << int(local_data[6]) << int(local_data[15]);
    message.HeaderVersion = ss_HV.str();
    std::stringstream ss_FV;
    ss_FV <<  int(local_data[12]) << int(local_data[13]) << int(local_data[14]) << int(local_data[15]);
    message.FormatVersion = ss_FV.str();

}

/// \file
/// \brief  getADMAStaticHeader function - ADMA Static Header Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getADMADynamicHeader(const std::string& local_data, adma_connect::Adma& message)
{
    // ! ADMA Dynamic Header
    char ConfigID[] = {local_data[68],local_data[69],local_data[70],local_data[71]};
    memcpy(&message.ConfigID , &ConfigID, sizeof(message.ConfigID));
    char ConfigFormat[] = {local_data[72],local_data[73],local_data[74],local_data[75]};
    memcpy(&message.ConfigFormat , &ConfigFormat, sizeof(message.ConfigFormat));
    char ConfigVersion[] = {local_data[76],local_data[77],local_data[78],local_data[79]};
    memcpy(&message.ConfigVersion , &ConfigVersion, sizeof(message.ConfigVersion));
    char ConfigSize[] = {local_data[80],local_data[81],local_data[82],local_data[83]};
    memcpy(&message.ConfigSize , &ConfigSize, sizeof(message.ConfigSize));
    char ByteOffset[] = {local_data[84],local_data[85],local_data[86],local_data[87]};
    memcpy(&message.ByteOffset , &ByteOffset, sizeof(message.ByteOffset));
    char SliceSize[] = {local_data[88],local_data[89],local_data[90],local_data[91]};
    memcpy(&message.SliceSize , &SliceSize, sizeof(message.SliceSize));
    char SliceData[] = {local_data[92],local_data[93],local_data[94],local_data[95]};

}

/// \file
/// \brief  getStatusGPS function - ADMA Status Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getStatusGPS(const std::string& local_data, adma_connect::Adma& message)
{
    unsigned char statusGPS;
    char Status_GPS[] = {local_data[96]};
    memcpy(&statusGPS, &Status_GPS, sizeof(statusGPS));
    bool status_External_Vel = getBit(statusGPS,7);
    bool status_skidding = getBit(statusGPS,5);
    bool standstill_c = getBit(statusGPS,4);
    bool rtk_precise = getBit(statusGPS,3);
    bool rtk_coarse = getBit(statusGPS,2);
    bool gps_mode = getBit(statusGPS,1);
    bool gps_out = getBit(statusGPS,0);

    /* Status GPS Mode */
    if(gps_out)
    {
        message.StatusGPSMode = 1;
    }
    else if (gps_mode) 
    {
        message.StatusGPSMode = 2;
    }
    else if (rtk_coarse) 
    {
        message.StatusGPSMode = 3;
    }
    else if (rtk_precise) 
    {
        message.StatusGPSMode = 4;
    }
    /* Status Stand Still */
    message.StatusStandStill = standstill_c;
    /* Status Skidding */
    message.StatusSkidding = status_skidding;
    /* Status External Velocity Slip */
    message.StatusExternalVelOut = status_External_Vel;
}


/// \file
/// \brief  getStatusTrigger function - ADMA GPS Trigger Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getStatusTrigger(const std::string& local_data, adma_connect::Adma& message)
{
    unsigned char statusTriggerGps;
    char StatusTrigger[] = {local_data[96]};
    memcpy(&statusTriggerGps, &StatusTrigger, sizeof(statusTriggerGps));
    bool status_synclock = getBit(statusTriggerGps,7);
    bool status_dead_reckoning = getBit(statusTriggerGps,6);
    bool status_ahrs_ins = getBit(statusTriggerGps,5);
    bool status_alignment = getBit(statusTriggerGps,4);
    bool status_signal_in1 = getBit(statusTriggerGps,3);
    bool status_signal_in2 = getBit(statusTriggerGps,2);
    bool status_signal_in3 = getBit(statusTriggerGps,1);
    bool status_trig_gps = getBit(statusTriggerGps,0);
    /* Status StatusTrigGps */
    message.StatusTrigGps = status_trig_gps;
    /* Status StatusSignalIN3 */
    message.StatusSignalIN3 = status_signal_in3;
    /* Status StatusSignalIn2 */
    message.StatusSignalIn2 = status_signal_in2;
    /* Status StatusSignalIn1 */
    message.StatusSignalIn1 = status_signal_in1;
    /* Status StatusAlignment */
    message.StatusAlignment = status_alignment;
    /* Status StatusAHRSINS */
    message.StatusAHRSINS = status_ahrs_ins;
    /* Status StatusDeadReckoning */
    message.StatusDeadReckoning = status_dead_reckoning;
    /* Status StatusSyncLock */
    message.StatusSyncLock = status_synclock;
}

/// \file
/// \brief  getStatusTrigger function - ADMA GPS Trigger Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getEVKStatus(const std::string& local_data, adma_connect::Adma& message)
{
    unsigned char statusEVK;
    char StatusEVK[] = {local_data[96]};
    memcpy(&statusEVK, &StatusEVK, sizeof(statusEVK));
    bool status_pos_b2 = getBit(statusEVK,7);
    bool status_pos_b1 = getBit(statusEVK,6);
    bool status_tilt_b2 = getBit(statusEVK,5);
    bool status_tilt_b1 = getBit(statusEVK,4);
    bool status_configuration_changed = getBit(statusEVK,3);
    bool status_heading_executed = getBit(statusEVK,2);
    bool status_evk_estimates = getBit(statusEVK,1);
    bool status_evk_activ = getBit(statusEVK,0);
    /* Status StatusTrigGps */
    message.StatusEVKActiv = status_evk_activ;
    /* Status status_evk_estimates */
    message.StatusEVKEstimates = status_evk_estimates;
    /* Status status_heading_executed */
    message.StatusHeadingExecuted = status_heading_executed;
    /* Status status_configuration_changed */
    message.StatusConfigurationChanged = status_configuration_changed;
    /* Status Pos */
    if(status_tilt_b1==0 && status_tilt_b2==0)
    {
        message.StatusTilt = 0;
    }
    else if(status_tilt_b1==0 && status_tilt_b2==1)
    {
        message.StatusTilt = 1;
    }
    else if(status_tilt_b1==1 && status_tilt_b2==0)
    {
        message.StatusTilt = 2;
    }
        /* Status Tilt */
    if(status_pos_b1==0 && status_pos_b2==0)
    {
        message.StatusPos = 0;
    }
    else if(status_pos_b1==0 && status_pos_b2==1)
    {
        message.StatusPos = 1;
    }
    else if(status_pos_b1==1 && status_pos_b2==0)
    {
        message.StatusPos = 2;
    }
}

/// \file
/// \brief  getStatusCount function - ADMA Status Count
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getStatusCount(const std::string& local_data, adma_connect::Adma& message)
{
    //! Status Count
    char Status_Count[] = {local_data[99]};
    message.StatusCount = int(local_data[99]);
    unsigned char statusKF;
    char Status_KF[] = {local_data[100]};
    memcpy(&statusKF, &Status_KF, sizeof(statusKF));
    bool status_speed_b2 = getBit(statusKF,5);
    bool status_speed_b1 = getBit(statusKF,4);
    bool status_KF_steady_state = getBit(statusKF,3);
    bool status_KF_Long_Stimulated = getBit(statusKF,2);
    bool status_KF_Lat_Stimulated = getBit(statusKF,1);
    bool status_KalmanFilter_settled = getBit(statusKF,0);
    message.StatusKalmanFilterSetteled = status_KalmanFilter_settled;
    message.StatusKFLatStimulated = status_KF_Lat_Stimulated;
    message.StatusKFLongStimulated = status_KF_Long_Stimulated;
    message.StatusKFSteadyState = status_KF_steady_state;
    if(status_speed_b1==0 && status_speed_b2==0)
    {
        message.StatusSpeed = 0;
    }
    else if(status_speed_b1==0 && status_speed_b2==1)
    {
        message.StatusSpeed = 1;
    }
    else if(status_speed_b1==1 && status_speed_b2==0)
    {
        message.StatusSpeed = 2;
    }
}

/// \file
/// \brief  getErrorandWarning function - ADMA Error and Warning
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getErrorandWarning(const std::string& local_data, adma_connect::Adma& message)
{
    std::bitset<8> bitdataError1 = local_data[104];
    std::bitset<8> bitdataError2 = local_data[105];
    std::bitset<8> bitdataWarn3 = local_data[106];
    std::bitset<8> ErrorHw = local_data[107];
    std::bitset<4> erHW1;
    std::bitset<4> erMisc1;
    std::bitset<4> erMisc2;
    std::bitset<4> erMisc3;
    std::bitset<4> warnGPS;
    std::bitset<4> warnMisc1;
    std::bitset<1> erHWSticky;

    for(size_t i=0;i<4;i++)
    {
        erHW1[i]    = bitdataError1[i];
        erMisc1[i]  = bitdataError1[i+4];
        erMisc2[i]  = bitdataError2[i];
        erMisc3[i]  = bitdataError2[i+4];
        warnGPS[i]  = bitdataWarn3[i];
        warnMisc1[i]  = bitdataWarn3[i+4];
    }
    erHWSticky[0] = ErrorHw[1];
    message.ErrorHardware = erHW1.to_string();
    message.Error_Misc1 = erMisc1.to_string();
    message.Error_Misc2 = erMisc2.to_string();
    message.Error_Misc3 = erMisc3.to_string();
    message.WarnGPS = warnGPS.to_string();
    message.WarnMisc1 = warnMisc1.to_string();
    message.ErrorHWSticky = erHWSticky.to_string();
}
/// \file
/// \brief  getSensorBodyXYZ function - ADMA Sensor Body X Y Z Acc and Rate Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getSensorBodyXYZ(const std::string& local_data, adma_connect::Adma& message)
{
    //! Sensor Body X
    char Acc_Body_HR_X[] = {local_data[112],local_data[113],local_data[114],local_data[115]};
    memcpy(&message.AccBodyHRX , &Acc_Body_HR_X, sizeof(message.AccBodyHRX));
    message.fAccBodyHRX = message.AccBodyHRX * 0.0001;
    
    char Rate_Body_HR_X[] = {local_data[116],local_data[117],local_data[118],local_data[119]};
    memcpy(&message.RateBodyHRX , &Rate_Body_HR_X, sizeof(message.RateBodyHRX));
    message.fRateBodyHRX = message.RateBodyHRX * 0.0001;

    //! Sensor Body Y
    char Acc_Body_HR_Y[] = {local_data[120],local_data[121],local_data[122],local_data[123]};
    memcpy(&message.AccBodyHRY , &Acc_Body_HR_Y, sizeof(message.AccBodyHRY));
    message.fAccBodyHRY = message.AccBodyHRY * 0.0001;   
    
    char Rate_Body_HR_Y[] = {local_data[124],local_data[125],local_data[126],local_data[127]};
    memcpy(&message.RateBodyHRY , &Rate_Body_HR_Y, sizeof(message.RateBodyHRY));
    message.fRateBodyHRY = message.RateBodyHRY * 0.0001;

    //! Sensor Body Z
    char Acc_Body_HR_Z[] = {local_data[128],local_data[129],local_data[130],local_data[131]};
    memcpy(&message.AccBodyHRZ , &Acc_Body_HR_Z, sizeof(message.AccBodyHRZ));
    message.fAccBodyHRZ = message.AccBodyHRZ * 0.0001;

    char Rate_Body_HR_Z[] = {local_data[132],local_data[133],local_data[134],local_data[135]};
    memcpy(&message.RateBodyHRZ , &Rate_Body_HR_Z, sizeof(message.RateBodyHRZ));
    message.fRateBodyHRZ = message.RateBodyHRZ * 0.0001;

}

/// \file
/// \brief  getRatesBodyXYZ function - ADMA Rate Body Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getRatesBodyXYZ(const std::string& local_data, adma_connect::Adma& message)
{
    //! Rates body
    char Rate_Body_X[] = {local_data[136],local_data[137]};
    memcpy(&message.RateBodyX , &Rate_Body_X, sizeof(message.RateBodyX));
    message.fRateBodyX = message.RateBodyX * 0.01;

    char Rate_Body_Y[] = {local_data[138],local_data[139]};
    memcpy(&message.RateBodyY , &Rate_Body_Y, sizeof(message.RateBodyY));
    message.fRateBodyY = message.RateBodyY * 0.01;

    char Rate_Body_Z[] = {local_data[140],local_data[141]};
    memcpy(&message.RateBodyZ , &Rate_Body_Z, sizeof(message.RateBodyZ));
    message.fRateBodyZ = message.RateBodyZ * 0.01;
}

/// \file
/// \brief  getRatesHorizontalXYZ function - ADMA Rates Horizontal
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getRatesHorizontalXYZ(const std::string& local_data, adma_connect::Adma& message)
{
    //! Rates horizontal
    char Rate_Hor_X[] = {local_data[144],local_data[145]};
    memcpy(&message.RateHorX , &Rate_Hor_X, sizeof(message.RateHorX));
    message.fRateHorX = message.RateHorX * 0.01;

    char Rate_Hor_Y[] = {local_data[146],local_data[147]};
    memcpy(&message.RateHorY , &Rate_Hor_Y, sizeof(message.RateHorY));
    message.fRateHorY = message.RateHorY * 0.01;

    char Rate_Hor_Z[] = {local_data[148],local_data[149]};
    memcpy(&message.RateHorZ , &Rate_Hor_Z, sizeof(message.RateHorZ));
    message.fRateHorZ = message.RateHorZ * 0.01;
}

/// \file
/// \brief  getAccelerationBody function - ADMA Acc Body
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBody(const std::string& local_data, adma_connect::Adma& message)
{

    //! Acceleration body
    char Acceleration_Body_X[] = {local_data[152],local_data[153]};
    memcpy(&message.AccBodyX , &Acceleration_Body_X, sizeof(message.AccBodyX));
    message.fAccBodyX = message.AccBodyX * 0.0004;    

    char Acceleration_Body_Y[] = {local_data[154],local_data[155]};
    memcpy(&message.AccBodyY , &Acceleration_Body_Y, sizeof(message.AccBodyY));
    message.fAccBodyY = message.AccBodyY * 0.0004;

    char Acceleration_Body_Z[] = {local_data[156],local_data[157]};
    memcpy(&message.AccBodyZ , &Acceleration_Body_Z, sizeof(message.AccBodyZ));
    message.fAccBodyZ = message.AccBodyZ * 0.0004;
}

/// \file
/// \brief  getAccelerationHor function - ADMA Acc Horizontal
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHor(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration horizontal
    char Acceleration_Hor_X[] = {local_data[160],local_data[161]};
    memcpy(&message.AccHorX , &Acceleration_Hor_X, sizeof(message.AccHorX));
    message.fAccHorX = message.AccHorX * 0.0004;

    char Acceleration_Hor_Y[] = {local_data[162],local_data[163]};
    memcpy(&message.AccHorY , &Acceleration_Hor_Y, sizeof(message.AccHorY));
    message.fAccHorY = message.AccHorY * 0.0004;

    char Acceleration_Hor_Z[] = {local_data[164],local_data[165]};
    memcpy(&message.AccHorZ , &Acceleration_Hor_Z, sizeof(message.AccHorZ));
    message.fAccHorZ = message.AccHorZ * 0.0004;

}

/// \file
/// \brief  getAccelerationBody POI1 function - ADMA Acc Body POI1
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI1(const std::string& local_data, adma_connect::Adma& message)
{
    char Acceleration_Body_X_POI1[] = {local_data[168],local_data[169]};
    memcpy(&message.AccBodyX_1 , &Acceleration_Body_X_POI1, sizeof(message.AccBodyX_1));
    message.fAccBodyX_1 = message.AccBodyX_1 * 0.0004;

    char Acceleration_Body_Y_POI1[] = {local_data[170],local_data[171]};
    memcpy(&message.AccBodyY_1 , &Acceleration_Body_Y_POI1, sizeof(message.AccBodyY_1));
    message.fAccBodyY_1 = message.AccBodyY_1 * 0.0004;

    char Acceleration_Body_Z_POI1[] = {local_data[172],local_data[173]};
    memcpy(&message.AccBodyZ_1 , &Acceleration_Body_Z_POI1, sizeof(message.AccBodyZ_1));
    message.fAccBodyZ_1 = message.AccBodyZ_1 * 0.0004;
}

/// \file
/// \brief  getAccelerationBody POI2 function - ADMA Acc Body POI2
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI2(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration POI2 body
    char Acceleration_Body_X_POI2[] = {local_data[176],local_data[177]};
    memcpy(&message.AccBodyX_2 , &Acceleration_Body_X_POI2, sizeof(message.AccBodyX_2));
    message.fAccBodyX_2 = message.AccBodyX_2 * 0.0004;

    char Acceleration_Body_Y_POI2[] = {local_data[178],local_data[179]};
    memcpy(&message.AccBodyY_2 , &Acceleration_Body_Y_POI2, sizeof(message.AccBodyY_2));
    message.fAccBodyY_2 = message.AccBodyY_2 * 0.0004;

    char Acceleration_Body_Z_POI2[] = {local_data[180],local_data[181]};
    memcpy(&message.AccBodyZ_2 , &Acceleration_Body_Z_POI2, sizeof(message.AccBodyZ_2));
    message.fAccBodyZ_2 = message.AccBodyZ_2 * 0.0004;

}

/// \file
/// \brief  getAccelerationBody POI3 function - ADMA Acc Body POI3
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI3(const std::string& local_data, adma_connect::Adma& message)
{

    //! Acceleration POI3 body
    char Acceleration_Body_X_POI3[] = {local_data[184],local_data[185]};
    memcpy(&message.AccBodyX_3 , &Acceleration_Body_X_POI3, sizeof(message.AccBodyX_3));
    message.fAccBodyX_3 = message.AccBodyX_3 * 0.0004;

    char Acceleration_Body_Y_POI3[] = {local_data[186],local_data[187]};
    memcpy(&message.AccBodyY_3 , &Acceleration_Body_Y_POI3, sizeof(message.AccBodyY_3));
    message.fAccBodyY_3 = message.AccBodyY_3 * 0.0004;

    char Acceleration_Body_Z_POI3[] = {local_data[188],local_data[189]};
    memcpy(&message.AccBodyZ_3 , &Acceleration_Body_Z_POI3, sizeof(message.AccBodyZ_3));
    message.fAccBodyZ_3 = message.AccBodyZ_3 * 0.0004;

}


/// \file
/// \brief  getAccelerationBody POI4 function - ADMA Acc Body POI4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI4(const std::string& local_data, adma_connect::Adma& message)
{

    //! Acceleration POI4 body
    char Acceleration_Body_X_POI4[] = {local_data[192],local_data[193]};
    memcpy(&message.AccBodyX_4 , &Acceleration_Body_X_POI4, sizeof(message.AccBodyX_4));
    message.fAccBodyX_4 = message.AccBodyX_4 * 0.0004;

    char Acceleration_Body_Y_POI4[] = {local_data[194],local_data[195]};
    memcpy(&message.AccBodyY_4 , &Acceleration_Body_Y_POI4, sizeof(message.AccBodyY_4));
    message.fAccBodyY_4 = message.AccBodyY_4 * 0.0004;

    char Acceleration_Body_Z_POI4[] = {local_data[196],local_data[197]};
    memcpy(&message.AccBodyZ_4 , &Acceleration_Body_Z_POI4, sizeof(message.AccBodyZ_4));
    message.fAccBodyZ_4 = message.AccBodyZ_4 * 0.0004;
}


/// \file
/// \brief  getAccelerationBody POI5 function - ADMA Acc Body POI5
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI5(const std::string& local_data, adma_connect::Adma& message)
{

    //! Acceleration POI5 body
    char Acceleration_Body_X_POI5[] = {local_data[200],local_data[201]};
    memcpy(&message.AccBodyX_5 , &Acceleration_Body_X_POI5, sizeof(message.AccBodyX_5));
    message.fAccBodyX_5 = message.AccBodyX_5 * 0.0004;

    char Acceleration_Body_Y_POI5[] = {local_data[202],local_data[203]};
    memcpy(&message.AccBodyY_5 , &Acceleration_Body_Y_POI5, sizeof(message.AccBodyY_5));
    message.fAccBodyY_5 = message.AccBodyY_5 * 0.0004;

    char Acceleration_Body_Z_POI5[] = {local_data[204],local_data[205]};
    memcpy(&message.AccBodyZ_5 , &Acceleration_Body_Z_POI5, sizeof(message.AccBodyZ_5));
    message.fAccBodyZ_5 = message.AccBodyZ_5 * 0.0004;

}

/// \file
/// \brief  getAccelerationBody POI6 function - ADMA Acc Body POI6
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI6(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration POI6 body
    char Acceleration_Body_X_POI6[] = {local_data[208],local_data[209]};
    memcpy(&message.AccBodyX_6 , &Acceleration_Body_X_POI6, sizeof(message.AccBodyX_6));
    message.fAccBodyX_6 = message.AccBodyX_6 * 0.0004;

    char Acceleration_Body_Y_POI6[] = {local_data[210],local_data[211]};
    memcpy(&message.AccBodyY_6 , &Acceleration_Body_Y_POI6, sizeof(message.AccBodyY_6));
    message.fAccBodyY_6 = message.AccBodyY_6 * 0.0004;

    char Acceleration_Body_Z_POI6[] = {local_data[212],local_data[213]};
    memcpy(&message.AccBodyZ_6 , &Acceleration_Body_Z_POI6, sizeof(message.AccBodyZ_6));
    message.fAccBodyZ_6 = message.AccBodyZ_6 * 0.0004;
}

/// \file
/// \brief  getAccelerationBody POI7 function - ADMA Acc Body POI7
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI7(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration POI7 body
    char Acceleration_Body_X_POI7[] = {local_data[216],local_data[217]};
    memcpy(&message.AccBodyX_7 , &Acceleration_Body_X_POI7, sizeof(message.AccBodyX_7));
    message.fAccBodyX_7 = message.AccBodyX_7 * 0.0004;

    char Acceleration_Body_Y_POI7[] = {local_data[218],local_data[219]};
    memcpy(&message.AccBodyY_7 , &Acceleration_Body_Y_POI7, sizeof(message.AccBodyY_7));
    message.fAccBodyY_7 = message.AccBodyY_7 * 0.0004;

    char Acceleration_Body_Z_POI7[] = {local_data[220],local_data[221]};
    memcpy(&message.AccBodyZ_7 , &Acceleration_Body_Z_POI7, sizeof(message.AccBodyZ_7));
    message.fAccBodyZ_7 = message.AccBodyZ_7 * 0.0004;

}

/// \file
/// \brief  getAccelerationBody POI8 function - ADMA Acc Body POI8
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationBodyPOI8(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration POI8 body
    char Acceleration_Body_X_POI8[] = {local_data[224],local_data[225]};
    memcpy(&message.AccBodyX_8 , &Acceleration_Body_X_POI8, sizeof(message.AccBodyX_8));
    message.fAccBodyX_8 = message.AccBodyX_8 * 0.0004;

    char Acceleration_Body_Y_POI8[] = {local_data[226],local_data[227]};
    memcpy(&message.AccBodyY_8 , &Acceleration_Body_Y_POI8, sizeof(message.AccBodyY_8));
    message.fAccBodyY_8 = message.AccBodyY_8 * 0.0004;

    char Acceleration_Body_Z_POI8[] = {local_data[228],local_data[229]};
    memcpy(&message.AccBodyZ_8 , &Acceleration_Body_Z_POI8, sizeof(message.AccBodyZ_8));
    message.fAccBodyZ_8 = message.AccBodyZ_8 * 0.0004;
}

/// \file
/// \brief  getAccelerationHor POI1 function - ADMA Acc Hor POI1
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI1(const std::string& local_data, adma_connect::Adma& message)
{
      //! Acceleration Hor. POI1 body
    char Acceleration_Hor_X_POI1[] = {local_data[232],local_data[233]};
    memcpy(&message.AccHorX_1 , &Acceleration_Hor_X_POI1, sizeof(message.AccHorX_1));
    message.fAccHorX_1 = message.AccHorX_1 * 0.0004;

    char Acceleration_Hor_Y_POI1[] = {local_data[234],local_data[235]};
    memcpy(&message.AccHorY_1 , &Acceleration_Hor_Y_POI1, sizeof(message.AccHorY_1));
    message.fAccHorY_1 = message.AccHorY_1 * 0.0004;

    char Acceleration_Hor_Z_POI1[] = {local_data[236],local_data[237]};
    memcpy(&message.AccHorZ_1 , &Acceleration_Hor_Z_POI1, sizeof(message.AccHorZ_1));
    message.fAccHorZ_1 = message.AccHorZ_1 * 0.0004;
}


/// \file
/// \brief  getAccelerationHor POI2 function - ADMA Acc Hor POI2
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI2(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI2 body
    char Acceleration_Hor_X_POI2[] = {local_data[240],local_data[241]};
    memcpy(&message.AccHorX_2 , &Acceleration_Hor_X_POI2, sizeof(message.AccHorX_2));
    message.fAccHorX_2 = message.AccHorX_2 * 0.0004;

    char Acceleration_Hor_Y_POI2[] = {local_data[242],local_data[243]};
    memcpy(&message.AccHorY_2 , &Acceleration_Hor_Y_POI2, sizeof(message.AccHorY_2));
    message.fAccHorY_2 = message.AccHorY_2 * 0.0004;

    char Acceleration_Hor_Z_POI2[] = {local_data[244],local_data[245]};
    memcpy(&message.AccHorZ_2 , &Acceleration_Hor_Z_POI2, sizeof(message.AccHorZ_2));
    message.fAccHorZ_2 = message.AccHorZ_2 * 0.0004;
}


/// \file
/// \brief  getAccelerationHor POI3 function - ADMA Acc Hor POI3
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI3(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI3 body
    char Acceleration_Hor_X_POI3[] = {local_data[248],local_data[249]};
    memcpy(&message.AccHorX_3 , &Acceleration_Hor_X_POI3, sizeof(message.AccHorX_3));
    message.fAccHorX_3 = message.AccHorX_3 * 0.0004;

    char Acceleration_Hor_Y_POI3[] = {local_data[250],local_data[251]};
    memcpy(&message.AccHorY_3 , &Acceleration_Hor_Y_POI3, sizeof(message.AccHorY_3));
    message.fAccHorY_3 = message.AccHorY_3 * 0.0004;

    char Acceleration_Hor_Z_POI3[] = {local_data[252],local_data[253]};
    memcpy(&message.AccHorZ_3 , &Acceleration_Hor_Z_POI3, sizeof(message.AccHorZ_3));
    message.fAccHorZ_3 = message.AccHorZ_3 * 0.0004;
}


/// \file
/// \brief  getAccelerationHor POI4 function - ADMA Acc Hor POI4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI4(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI4 body
    char Acceleration_Hor_X_POI4[] = {local_data[256],local_data[257]};
    memcpy(&message.AccHorX_4 , &Acceleration_Hor_X_POI4, sizeof(message.AccHorX_4));
    message.fAccHorX_4 = message.AccHorX_4 * 0.0004;

    char Acceleration_Hor_Y_POI4[] = {local_data[258],local_data[259]};
    memcpy(&message.AccHorY_4 , &Acceleration_Hor_Y_POI4, sizeof(message.AccHorY_4));
    message.fAccHorY_4 = message.AccHorY_4 * 0.0004;

    char Acceleration_Hor_Z_POI4[] = {local_data[260],local_data[261]};
    memcpy(&message.AccHorZ_4 , &Acceleration_Hor_Z_POI4, sizeof(message.AccHorZ_4));
    message.fAccHorZ_4 = message.AccHorZ_4 * 0.0004;
}

/// \file
/// \brief  getAccelerationHorPOI5 function - ADMA Acc Hor POI5
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI5(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI5 body
    char Acceleration_Hor_X_POI5[] = {local_data[264],local_data[265]};
    memcpy(&message.AccHorX_5 , &Acceleration_Hor_X_POI5, sizeof(message.AccHorX_5));
    message.fAccHorX_5 = message.AccHorX_5 * 0.0004;

    char Acceleration_Hor_Y_POI5[] = {local_data[266],local_data[267]};
    memcpy(&message.AccHorY_5 , &Acceleration_Hor_Y_POI5, sizeof(message.AccHorY_5));
    message.fAccHorY_5 = message.AccHorY_5 * 0.0004;

    char Acceleration_Hor_Z_POI5[] = {local_data[268],local_data[269]};
    memcpy(&message.AccHorZ_5 , &Acceleration_Hor_Z_POI5, sizeof(message.AccHorZ_5));
    message.fAccHorZ_5 = message.AccHorZ_5 * 0.0004;
}

/// \file
/// \brief  getAccelerationHorPOI6 function - ADMA Acc Hor POI6
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI6(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI6 body
    char Acceleration_Hor_X_POI6[] = {local_data[272],local_data[273]};
    memcpy(&message.AccHorX_6 , &Acceleration_Hor_X_POI6, sizeof(message.AccHorX_6));
    message.fAccHorX_6 = message.AccHorX_6 * 0.0004;

    char Acceleration_Hor_Y_POI6[] = {local_data[274],local_data[275]};
    memcpy(&message.AccHorY_6 , &Acceleration_Hor_Y_POI6, sizeof(message.AccHorY_6));
    message.fAccHorY_6 = message.AccHorY_6 * 0.0004;

    char Acceleration_Hor_Z_POI6[] = {local_data[276],local_data[277]};
    memcpy(&message.AccHorZ_6 , &Acceleration_Hor_Z_POI6, sizeof(message.AccHorZ_6));
    message.fAccHorZ_6 = message.AccHorZ_6 * 0.0004;
}

/// \file
/// \brief  getAccelerationHorPOI7 function - ADMA Acc Hor POI7
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI7(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI7 body
    char Acceleration_Hor_X_POI7[] = {local_data[280],local_data[281]};
    memcpy(&message.AccHorX_7 , &Acceleration_Hor_X_POI7, sizeof(message.AccHorX_7));
    message.fAccHorX_7 = message.AccHorX_7 * 0.0004;

    char Acceleration_Hor_Y_POI7[] = {local_data[282],local_data[283]};
    memcpy(&message.AccHorY_7 , &Acceleration_Hor_Y_POI7, sizeof(message.AccHorY_7));
    message.fAccHorY_7 = message.AccHorY_7 * 0.0004;

    char Acceleration_Hor_Z_POI7[] = {local_data[284],local_data[285]};
    memcpy(&message.AccHorZ_7 , &Acceleration_Hor_Z_POI7, sizeof(message.AccHorZ_7));
    message.fAccHorZ_7 = message.AccHorZ_7 * 0.0004;

}

/// \file
/// \brief  getAccelerationHorPOI8 function - ADMA Acc Hor POI8
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAccelerationHorPOI8(const std::string& local_data, adma_connect::Adma& message)
{
    //! Acceleration Hor. POI8 body
    char Acceleration_Hor_X_POI8[] = {local_data[288],local_data[289]};
    memcpy(&message.AccHorX_8 , &Acceleration_Hor_X_POI8, sizeof(message.AccHorX_8));
    message.fAccHorX_8 = message.AccHorX_8 * 0.0004;

    char Acceleration_Hor_Y_POI8[] = {local_data[290],local_data[291]};
    memcpy(&message.AccHorY_8 , &Acceleration_Hor_Y_POI8, sizeof(message.AccHorY_8));
    message.fAccHorY_8 = message.AccHorY_8 * 0.0004;

    char Acceleration_Hor_Z_POI8[] {local_data[292],local_data[293]};
    memcpy(&message.AccHorZ_8 , &Acceleration_Hor_Z_POI8, sizeof(message.AccHorZ_8));
    message.fAccHorZ_8 = message.AccHorZ_8 * 0.0004;
}

/// \file
/// \brief  getExternalVelocityAnalog function - ADMA Ext Vel Analog
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getExternalVelocityAnalog(const std::string& local_data, adma_connect::Adma& message)
{
    //! External Velocity Analalog
    char Ext_Vel_An_X[] = {local_data[296],local_data[297]};
    memcpy(&message.ExtVelAnX , &Ext_Vel_An_X, sizeof(message.ExtVelAnX));
    message.fExtVelAnX = message.ExtVelAnX * 0.005;

    char Ext_Vel_An_Y[] = {local_data[298],local_data[299]};
    memcpy(&message.ExtVelAnY , &Ext_Vel_An_Y, sizeof(message.ExtVelAnY));
    message.fExtVelAnY = message.ExtVelAnY * 0.005;
}

/// \file
/// \brief  getExternalVecovityDigPulses function - ADMA Ext Vel Dig Pulses
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getExternalVecovityDigPulses(const std::string& local_data, adma_connect::Adma& message)
{
    //! External Velocity Digital Pulses
    char Ext_Vel_Dig_X[] = {local_data[304],local_data[305]};
    memcpy(&message.ExtVelDigX , &Ext_Vel_Dig_X, sizeof(message.ExtVelDigX));
    message.fExtVelDigX = message.ExtVelAnY * 0.005;

    char Ext_Vel_Dig_Y[] = {local_data[306],local_data[307]};
    memcpy(&message.ExtVelDigY , &Ext_Vel_Dig_Y, sizeof(message.ExtVelDigY));
    message.fExtVelDigY = message.ExtVelDigY * 0.005;

    char Ext_Vel_Dig_Pulses_X[] = {local_data[308],local_data[309]};
    memcpy(&message.ExtVelDigPulsesX , &Ext_Vel_Dig_Pulses_X, sizeof(message.ExtVelDigPulsesX));

    char Ext_Vel_Dig_Pulses_Y[] = {local_data[310],local_data[311]};
    memcpy(&message.ExtVelDigPulsesY , &Ext_Vel_Dig_Pulses_Y, sizeof(message.ExtVelDigPulsesY));
}

/// \file
/// \brief  getExternalVelocityCorrected function - ADMA Ext Vel Corrected
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getExternalVelocityCorrected(const std::string& local_data, adma_connect::Adma& message)
{
    //! External Velocity Corrected
    char Ext_Vel_X_Corrected[] = {local_data[312],local_data[313]};
    memcpy(&message.ExtVelXCorrected , &Ext_Vel_X_Corrected, sizeof(message.ExtVelXCorrected));
    message.fExtVelXCorrected = message.ExtVelXCorrected * 0.005;

    char Ext_Vel_Y_Corrected[] = {local_data[314],local_data[315]};
    memcpy(&message.ExtVelYCorrected , &Ext_Vel_Y_Corrected, sizeof(message.ExtVelYCorrected));
    message.fExtVelYCorrected = message.ExtVelYCorrected * 0.005;
}

/// \file
/// \brief  getBarometerPressure function - ADMA Barometer Pressure
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getBarometerPressure(const std::string& local_data, adma_connect::Adma& message)
{
    //! Barometer Pressure
    char Ext_Baro_Pressue[] = {local_data[320],local_data[321],local_data[322],local_data[323]};
    memcpy(&message.ExtBaroPressure , &Ext_Baro_Pressue, sizeof(message.ExtBaroPressure));
    message.fExtBaroPressure = message.ExtBaroPressure * 0.01;
}

/// \file
/// \brief  getBarometerHeight function - ADMA Barometer Height
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getBarometerHeight(const std::string& local_data, adma_connect::Adma& message)
{
    //! Barometer Height
    char Ext_Baro_Height[] = {local_data[328],local_data[329],local_data[330],local_data[331]};
    memcpy(&message.ExtBaroHeight , &Ext_Baro_Height, sizeof(message.ExtBaroHeight));
    message.fExtBaroHeight = message.ExtBaroHeight * 0.01;

    char Ext_Baro_Height_Corrected[] = {local_data[332],local_data[333],
                                        local_data[334],local_data[335]};
    memcpy(&message.ExtBaroHeightCorrected , &Ext_Baro_Height_Corrected, sizeof(message.ExtBaroHeightCorrected));
    message.fExtBaroHeightCorrected = message.ExtBaroHeightCorrected * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc.
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuos(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous
    char Inv_Path_Radius[] = {local_data[344],local_data[345]};
    memcpy(&message.InvPathRadius , &Inv_Path_Radius, sizeof(message.InvPathRadius));
    message.fInvPathRadius = message.InvPathRadius * 0.0001;

    char Side_Slip_Angle[] = {local_data[346],local_data[347]};
    memcpy(&message.SideSlipAngle , &Side_Slip_Angle, sizeof(message.SideSlipAngle));
    message.fSideSlipAngle = message.SideSlipAngle * 0.01;

    char Dist_Trav[] = {local_data[348],local_data[349],local_data[350],local_data[351]};
    memcpy(&message.DistTrav , &Dist_Trav, sizeof(message.DistTrav));
    message.fDistTrav = message.DistTrav * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI1
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI1(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 1
    char Inv_Path_Radius_POI1[] = {local_data[352],local_data[353]};
    memcpy(&message.InvPathRadius_1 , &Inv_Path_Radius_POI1, sizeof(message.InvPathRadius_1));
    message.fInvPathRadius_1 = message.InvPathRadius_1 * 0.0001;

    char Side_Slip_Angle_POI1[] = {local_data[354],local_data[355]};
    memcpy(&message.SideSlipAngle_1 , &Side_Slip_Angle_POI1, sizeof(message.SideSlipAngle_1));
    message.fSideSlipAngle_1 = message.SideSlipAngle_1 * 0.01;

    char Dist_Trav_POI1[] = {local_data[356],local_data[357],local_data[358],local_data[359]};
    memcpy(&message.DistTrav_1 , &Dist_Trav_POI1, sizeof(message.DistTrav_1));
    message.fDistTrav_1 = message.DistTrav_1 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI2
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI2(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 2
    char Inv_Path_Radius_POI2[] = {local_data[360],local_data[361]};
    memcpy(&message.InvPathRadius_2 , &Inv_Path_Radius_POI2, sizeof(message.InvPathRadius_2));
    message.fInvPathRadius_2 = message.InvPathRadius_2 * 0.0001;

    char Side_Slip_Angle_POI2[] = {local_data[362],local_data[363]};
    memcpy(&message.SideSlipAngle_2 , &Side_Slip_Angle_POI2, sizeof(message.SideSlipAngle_2));
    message.fSideSlipAngle_2 = message.SideSlipAngle_2 * 0.01;

    char Dist_Trav_POI2[] = {local_data[364],local_data[365],local_data[366],local_data[367]};
    memcpy(&message.DistTrav_2 , &Dist_Trav_POI2, sizeof(message.DistTrav_2));
    message.fDistTrav_2 = message.DistTrav_2 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI3
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI3(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 3
    char Inv_Path_Radius_POI3[] = {local_data[368],local_data[369]};
    memcpy(&message.InvPathRadius_3 , &Inv_Path_Radius_POI3, sizeof(message.InvPathRadius_3));
    message.fInvPathRadius_3 = message.InvPathRadius_3 * 0.0001;

    char Side_Slip_Angle_POI3[] = {local_data[370],local_data[371]};
    memcpy(&message.SideSlipAngle_3 , &Side_Slip_Angle_POI3, sizeof(message.SideSlipAngle_3));
    message.fSideSlipAngle_3 = message.SideSlipAngle_3 * 0.01;

    char Dist_Trav_POI3[] = {local_data[372],local_data[373],local_data[374],local_data[375]};
    memcpy(&message.DistTrav_3 , &Dist_Trav_POI3, sizeof(message.DistTrav_3));
    message.fDistTrav_3 = message.DistTrav_3 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI4(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 4
    char Inv_Path_Radius_POI4[] = {local_data[376],local_data[377]};
    memcpy(&message.InvPathRadius_4 , &Inv_Path_Radius_POI4, sizeof(message.DistTrav));
    message.fInvPathRadius_4 = message.DistTrav * 0.0001;

    char Side_Slip_Angle_POI4[] = {local_data[378],local_data[379]};
    memcpy(&message.SideSlipAngle_4 , &Side_Slip_Angle_POI4, sizeof(message.SideSlipAngle_4));
    message.fSideSlipAngle_4 = message.SideSlipAngle_4 * 0.01;

    char Dist_Trav_POI4[] = {local_data[380],local_data[381],local_data[382],local_data[383]};
    memcpy(&message.DistTrav_4 , &Dist_Trav_POI4, sizeof(message.DistTrav_4));
    message.fDistTrav_4 = message.DistTrav_4 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI5
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI5(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 5
    char Inv_Path_Radius_POI5[] = {local_data[384],local_data[385]};
    memcpy(&message.InvPathRadius_5 , &Inv_Path_Radius_POI5, sizeof(message.InvPathRadius_5));
    message.fInvPathRadius_5 = message.InvPathRadius_5 * 0.0001;

    char Side_Slip_Angle_POI5[] = {local_data[386],local_data[387]};
    memcpy(&message.SideSlipAngle_5 , &Side_Slip_Angle_POI5, sizeof(message.SideSlipAngle_5));
    message.fSideSlipAngle_5 = message.SideSlipAngle_5 * 0.01;

    char Dist_Trav_POI5[] = {local_data[388],local_data[389],local_data[390],local_data[391]};
    memcpy(&message.DistTrav_5 , &Dist_Trav_POI5, sizeof(message.DistTrav_5));
    message.fDistTrav_5 = message.DistTrav_5 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI6
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI6(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 6
    char Inv_Path_Radius_POI6[] = {local_data[392],local_data[393]};
    memcpy(&message.InvPathRadius_6 , &Inv_Path_Radius_POI6, sizeof(message.InvPathRadius_6));
    message.fInvPathRadius_6 = message.InvPathRadius_6 * 0.0001;

    char Side_Slip_Angle_POI6[] = {local_data[394],local_data[395]};
    memcpy(&message.SideSlipAngle_6 , &Side_Slip_Angle_POI6, sizeof(message.SideSlipAngle_6));
    message.fSideSlipAngle_6 = message.SideSlipAngle_6 * 0.01;

    char Dist_Trav_POI6[] = {local_data[396],local_data[397],local_data[398],local_data[399]};
    memcpy(&message.DistTrav_6 , &Dist_Trav_POI6, sizeof(message.DistTrav_6));
    message.fDistTrav_6 = message.DistTrav_6 * 0.01;
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI7
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI7(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 7
    char Inv_Path_Radius_POI7[] = {local_data[400],local_data[401]};
    memcpy(&message.InvPathRadius_7 , &Inv_Path_Radius_POI7, sizeof(message.InvPathRadius_7));
    message.fInvPathRadius_7 = message.InvPathRadius_7 * 0.0001;

    char Side_Slip_Angle_POI7[] = {local_data[402],local_data[403]};
    memcpy(&message.SideSlipAngle_7 , &Side_Slip_Angle_POI7, sizeof(message.SideSlipAngle_7));
    message.fSideSlipAngle_7 = message.SideSlipAngle_7 * 0.01;

    char Dist_Trav_POI7[] = {local_data[404],local_data[405],local_data[406],local_data[407]};
    memcpy(&message.DistTrav_7 , &Dist_Trav_POI7, sizeof(message.DistTrav_7));
    message.fDistTrav_7 = message.DistTrav_7 * 0.01;
    
}

/// \file
/// \brief  getMiscellaneuos function - ADMA Misc. POI8
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getMiscellaneuosPOI8(const std::string& local_data, adma_connect::Adma& message)
{
    //! Miscellaneous POI 8
    char Inv_Path_Radius_POI8[] = {local_data[408],local_data[409]};
    memcpy(&message.InvPathRadius_8 , &Inv_Path_Radius_POI8, sizeof(message.InvPathRadius_8));
    message.fInvPathRadius_8 = message.InvPathRadius_8 * 0.0001;

    char Side_Slip_Angle_POI8[] = {local_data[410],local_data[411]};
    memcpy(&message.SideSlipAngle_8 , &Side_Slip_Angle_POI8, sizeof(message.SideSlipAngle_8));
    message.fSideSlipAngle_8 = message.SideSlipAngle_8 * 0.01;

    char Dist_Trav_POI8[] = {local_data[412],local_data[413],local_data[414],local_data[415]};
    memcpy(&message.DistTrav_8 , &Dist_Trav_POI8, sizeof(message.DistTrav_8));
    message.fDistTrav_8 = message.DistTrav_8 * 0.01;
}

/// \file
/// \brief  getTriggers function - ADMA Triggers 1,2,3 and 4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getTriggers(const std::string& local_data, adma_connect::Adma& message)
{
    //! Triggers 1 and 2
    char Trigger_Raising_1[] = {local_data[416],local_data[417]};
    memcpy(&message.TrigRising1 , &Trigger_Raising_1, sizeof(message.TrigRising1));
    char Trigger_Falling_1[] = {local_data[418],local_data[419]};
    memcpy(&message.TrigFalling1 , &Trigger_Falling_1, sizeof(message.TrigFalling1));
    char Trigger_Raising_2[] = {local_data[420],local_data[421]};
    memcpy(&message.TrigRising2 , &Trigger_Raising_2, sizeof(message.TrigRising2));
    char Trigger_Falling_2[] = {local_data[422],local_data[423]};
    memcpy(&message.TrigFalling2 , &Trigger_Falling_2, sizeof(message.TrigFalling2));
    //! Triggers 3 and 4
    char Trigger_Raising_3[] = {local_data[424],local_data[425]};
    memcpy(&message.TrigRising3 , &Trigger_Raising_3, sizeof(message.TrigRising3));
    char Trigger_Falling_3[] = {local_data[426],local_data[427]};
    memcpy(&message.TrigFalling3 , &Trigger_Falling_3, sizeof(message.TrigFalling3));
    char Trigger_Raising_4[] = {local_data[428],local_data[429]};
    memcpy(&message.TrigRising4 , &Trigger_Raising_4, sizeof(message.TrigRising4));
    char Trigger_Falling_4[] = {local_data[430],local_data[431]};
    memcpy(&message.TrigFalling4 , &Trigger_Falling_4, sizeof(message.TrigFalling4));
}

/// \file
/// \brief  getSystemData function - ADMA System Data
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getSystemData(const std::string& local_data, adma_connect::Adma& message)
{
    //! System Data
    char System_Ta[] = {local_data[432],local_data[433]};
    memcpy(&message.SystemTa , &System_Ta, sizeof(message.SystemTa));
    char System_Temp[] = {local_data[434],local_data[435]};
    memcpy(&message.SystemTemp , &System_Temp, sizeof(message.SystemTemp));
    message.fSystemTemp = message.SystemTemp * 0.1;
    char System_TimeSinceInit[] = {local_data[436],local_data[437]};
    memcpy(&message.SystemTimeSinceInit , &System_TimeSinceInit, sizeof(message.SystemTimeSinceInit));
    char System_DSP_Load[] = {local_data[438],local_data[439]};
    memcpy(&message.SystemDSPLoad , &System_DSP_Load, sizeof(message.SystemDSPLoad));
    message.fSystemDSPLoad = message.SystemDSPLoad * 0.1;

}

/// \file
/// \brief  getGPSAbs function - ADMA Absolute GPS Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSAbs(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Position Absolute
    char GPS_Lat_Abs[] = {local_data[440],local_data[441],local_data[442],local_data[443]};
    memcpy(&message.GPSLatAbs , &GPS_Lat_Abs, sizeof(message.GPSLatAbs));
    message.fGPSLatAbs = message.GPSLatAbs * 0.0000001;
    char GPS_Lon_Abs[] = {local_data[444],local_data[445],local_data[446],local_data[447]};
    memcpy(&message.GPSLonAbs , &GPS_Lon_Abs, sizeof(message.GPSLonAbs));
    message.fGPSLonAbs = message.GPSLonAbs * 0.0000001;
}

/// \file
/// \brief  getGPSPosRel function - ADMA Relative GPS Information
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSPosRel(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Position Relative
    char GPS_Lat_Rel[] = {local_data[448],local_data[449],local_data[450],local_data[451]};
    memcpy(&message.GPSLatRel , &GPS_Lat_Rel, sizeof(message.GPSLatRel));
    message.fGPSLatRel = message.GPSLatRel * 0.01;

    char GPS_Lon_Rel[] = {local_data[452],local_data[453],local_data[454],local_data[455]};
    memcpy(&message.GPSLonRel , &GPS_Lon_Rel, sizeof(message.GPSLonRel));
    message.fGPSLonRel = message.GPSLonRel * 0.01;
}

/// \file
/// \brief  getGPSEPE function - ADMA Position Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSEPE(const std::string& local_data, adma_connect::Adma& message)
{
   //! GPS Position Error
    char GPS_Stddev_Lat[] = {local_data[456],local_data[457]};
    memcpy(&message.GPSStddevLat , &GPS_Stddev_Lat, sizeof(message.GPSStddevLat));
    message.fGPSStddevLat = message.GPSStddevLat * 0.001;
    char GPS_Stddev_Lon[] = {local_data[458],local_data[459]};
    memcpy(&message.GPSStddevLon , &GPS_Stddev_Lon, sizeof(message.GPSStddevLon));
    message.fGPSStddevLon = message.GPSStddevLon * 0.001;
    char GPS_Stddev_Height[] = {local_data[460],local_data[461]};
    memcpy(&message.GPSStddevHeight , &GPS_Stddev_Height, sizeof(message.GPSStddevHeight));
    message.fGPSStddevHeight = message.GPSStddevHeight * 0.001;
}

/// \file
/// \brief  getGPSAbs function - ADMA Velocity Frmae X Y Z
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSVelFrame(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Velocity Frame
    char GPS_Vel_Frame_X[] = {local_data[464],local_data[465]};
    memcpy(&message.GPSVelFrameX , &GPS_Vel_Frame_X, sizeof(message.GPSVelFrameX));
    message.fGPSVelFrameX = message.GPSVelFrameX * 0.005;
    char GPS_Vel_Frame_Y[] = {local_data[466],local_data[467]};
    memcpy(&message.GPSVelFrameY , &GPS_Vel_Frame_Y, sizeof(message.GPSVelFrameY));
    message.fGPSVelFrameY = message.GPSVelFrameY * 0.005;
    char GPS_Vel_Frame_Z[] = {local_data[468],local_data[469]};
    memcpy(&message.GPSVelFrameZ , &GPS_Vel_Frame_Z, sizeof(message.GPSVelFrameZ));
    message.fGPSVelFrameZ = message.GPSVelFrameZ * 0.005;
    char GPS_Vel_Latency[] = {local_data[470],local_data[471]};
    memcpy(&message.GPSVelLatency , &GPS_Vel_Latency, sizeof(message.GPSVelLatency));
    message.fGPSVelLatency = message.GPSVelLatency * 0.001;    
}

/// \file
/// \brief  getGPSAbs function - ADMA Expected Velocity Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSVelEVE(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Velocity Error
    char GPS_Stddev_Vel_X[] = {local_data[472],local_data[473]};
    memcpy(&message.GPSStddevVelX , &GPS_Stddev_Vel_X, sizeof(message.GPSStddevVelX));
    message.fGPSStddevVelX = message.GPSStddevVelX * 0.001;  
    char GPS_Stddev_Vel_Y[] = {local_data[474],local_data[475]};
    memcpy(&message.GPSStddevVelY , &GPS_Stddev_Vel_Y, sizeof(message.GPSStddevVelY));
    message.fGPSStddevVelY = message.GPSStddevVelY * 0.001;  
    char GPS_Stddev_Vel_Z[] = {local_data[476],local_data[477]};
    memcpy(&message.GPSStddevVelZ , &GPS_Stddev_Vel_Z, sizeof(message.GPSStddevVelZ));
    message.fGPSStddevVelZ = message.GPSStddevVelZ * 0.001;  
}

/// \file
/// \brief  getGPSAbs function - ADMA Expected Velocity Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSTimeUTC(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Time UTC
    char GPS_Time_msec[] = {local_data[480],local_data[481],local_data[482],local_data[483]};
    memcpy(&message.GPSTimemsec , &GPS_Time_msec, sizeof(message.GPSTimemsec));
    char GPS_Time_Weel[] = {local_data[484], local_data[485]};
    memcpy(&message.GPSTimeWeek , &GPS_Time_Weel, sizeof(message.GPSTimeWeek));
    char Trigger_GPS[] = {local_data[486],local_data[487]};
    memcpy(&message.GPSTrigger , &Trigger_GPS, sizeof(message.GPSTrigger));
}
/// \file
/// \brief  getGPSAbs function - ADMA Expected Velocity Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSAuxData1(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Aux Data 1
    char GPS_Diff_Age[] = {local_data[488],local_data[489]};
    memcpy(&message.GPSDiffAge , &GPS_Diff_Age, sizeof(message.GPSDiffAge));
    message.fGPSDiffAge = message.GPSDiffAge * 0.1;  
    char GPS_Stats_Used[] = {local_data[490]};
    memcpy(&message.GPSStatsUsed , &GPS_Diff_Age, sizeof(message.GPSStatsUsed));
    char GPS_Stats_Visible[] = {local_data[491]};
    memcpy(&message.GPSStatsVisible , &GPS_Diff_Age, sizeof(message.GPSStatsVisible));

}
/// \file
/// \brief  getGPSAbs function - ADMA Expected Velocity Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSAuxData2(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Aux Data 2
    char GPS_Log_Delay[] = {local_data[496]};
    memcpy(&message.GPSLogDelay , &GPS_Log_Delay, sizeof(message.GPSLogDelay));
    char GPS_Receiver_Load[] = {local_data[497]};
    memcpy(&message.GPSReceiverLoad , &GPS_Receiver_Load, sizeof(message.GPSReceiverLoad));
    message.fGPSReceiverLoad = message.GPSReceiverLoad * 0.5;  
    char GPS_BaseNr[] = {local_data[498]};
}
/// \file
/// \brief  getGPSAbs function - ADMA Expected Velocity Error
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSAngleGPSCOG(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Angle and GPS COG
    char INS_Roll[] = {local_data[504],local_data[505]};
    memcpy(&message.INSRoll , &INS_Roll, sizeof(message.INSRoll));
    message.fINSRoll = message.INSRoll * 0.01;  
    char INS_Pitch[] = {local_data[506],local_data[507]};
    memcpy(&message.INSPitch , &INS_Pitch, sizeof(message.INSPitch));
    message.fINSPitch = message.INSPitch * 0.01;  
    char INS_Yaw[] = {local_data[508],local_data[509]};
    memcpy(&message.INSYaw , &INS_Yaw, sizeof(message.INSYaw));
    message.fINSYaw = message.INSYaw * 0.01;  
    char GPS_COG[] = {local_data[510],local_data[511]};
    memcpy(&message.GPSCOG , &GPS_COG, sizeof(message.GPSCOG));
    message.fGPSCOG = message.GPSCOG * 0.01;  
}

/// \file
/// \brief  getGPSHeight function - ADMA GPS Height
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSHeight(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Height (MSL)
    char GPS_Height[] = {local_data[512],local_data[513],local_data[514],local_data[515]};
    memcpy(&message.GPSHeight , &GPS_Height, sizeof(message.GPSHeight));
    message.GPSHeight = message.GPSHeight * 0.01;  
    char Undulation[] = {local_data[516],local_data[517]};
    memcpy(&message.Undulation , &Undulation, sizeof(message.Undulation));
    message.fUndulation = message.Undulation * 0.01;  
}

/// \file
/// \brief  getGPSDualAntTimeUTC function - ADMA GPS Dual Ant Time UTC
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSDualAntTimeUTC(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS DualAnt Time UTC
    char GPS_DualAnt_Time_msec[] = {local_data[520],local_data[521],local_data[522],local_data[523]};
    memcpy(&message.GPSDualAntTimemsec , &GPS_DualAnt_Time_msec, sizeof(message.GPSDualAntTimemsec));
    char GPS_DualAnt_Time_Week[] = {local_data[524],local_data[525]};
    memcpy(&message.GPSDualAntTimeWeek , &GPS_DualAnt_Time_Week, sizeof(message.GPSDualAntTimeWeek));
}

/// \file
/// \brief  getGPSDualAntAngle function - ADMA GPS Dual Ant Angle
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSDualAntAngle(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS DualAnt Angle
    char GPS_DualAnt_Heading[] = {local_data[528],local_data[529]};
    memcpy(&message.GPSDualAntHeading , &GPS_DualAnt_Heading, sizeof(message.GPSDualAntHeading));
    message.fGPSDualAntHeading = message.GPSDualAntHeading * 0.01;  
    char GPS_DualAnt_Pitch[] = {local_data[530],local_data[531]};
    memcpy(&message.GPSDualAntPitch , &GPS_DualAnt_Pitch, sizeof(message.GPSDualAntPitch));
    message.fGPSDualAntPitch = message.GPSDualAntPitch * 0.01;  
}

/// \file
/// \brief  getGPSDualAntAngleETE function - ADMA GPS Dual Ant Angle ETE
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGPSDualAntAngleETE(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS DualAnt Angle ETE
    char GPS_DualAnt_Stddev_Heading[] =  {local_data[536]};
    memcpy(&message.GPSDualAntStdDevHeading , &GPS_DualAnt_Stddev_Heading, sizeof(message.GPSDualAntStdDevHeading));
    message.fGPSDualAntStdDevHeading = message.GPSDualAntStdDevHeading * 0.01;  
    char GPS_DualAnt_Stddev_Pitch[] = {local_data[537]};
    memcpy(&message.GPSDualAntStdDevPitch , &GPS_DualAnt_Stddev_Pitch, sizeof(message.GPSDualAntStdDevPitch));
    message.fGPSDualAntStdDevPitch = message.GPSDualAntStdDevPitch * 0.01;  
}


/// \file
/// \brief  getGPSDualAntAngleETE function - ADMA GPS Dual Ant Angle ETE
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPositionHeight(const std::string& local_data, adma_connect::Adma& message)
{
    char INS_Height[] = {local_data[544],local_data[545],local_data[546],local_data[547]};
    memcpy(&message.INSHeight , &INS_Height, sizeof(message.INSHeight));
    message.fINSHeight = message.INSHeight * 0.01;
}

/// \file
/// \brief  getGPSDualAntAngleETE function - ADMA GPS Dual Ant Angle ETE
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPositionPOI(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Position Height 1 and 2
    char INS_Height_POI1[] = {local_data[552],local_data[553],local_data[554],local_data[555]};
    memcpy(&message.INSHeight_1 , &INS_Height_POI1, sizeof(message.INSHeight_1));
    message.fINSHeight_1 = message.INSHeight_1 * 0.01;
    char INS_Height_POI2[] = {local_data[556],local_data[557],local_data[558],local_data[559]};
    memcpy(&message.INSHeight_2 , &INS_Height_POI2, sizeof(message.INSHeight_2));
    message.fINSHeight_2 = message.INSHeight_2 * 0.01;
    //! INS Position Height 3 and 4
    char INS_Height_POI3[] = {local_data[560],local_data[561],local_data[562],local_data[563]};
    memcpy(&message.INSHeight_3 , &INS_Height_POI3, sizeof(message.INSHeight_3));
    message.fINSHeight_3 = message.INSHeight_3 * 0.01;
    char INS_Height_POI4[] = {local_data[564],local_data[565],local_data[566],local_data[567]};
    memcpy(&message.INSHeight_4 , &INS_Height_POI4, sizeof(message.INSHeight_4));
    message.fINSHeight_4 = message.INSHeight_4 * 0.01;
    //! INS Position Height 5 and 6
    char INS_Height_POI5[] = {local_data[568],local_data[569],local_data[570],local_data[571]};
    memcpy(&message.INSHeight_5 , &INS_Height_POI5, sizeof(message.INSHeight_5));
    message.fINSHeight_5 = message.INSHeight_5 * 0.01;
    char INS_Height_POI6[] = {local_data[572],local_data[573],local_data[574],local_data[575]};
    memcpy(&message.INSHeight_6 , &INS_Height_POI6, sizeof(message.INSHeight_6));
    message.fINSHeight_6 = message.INSHeight_6 * 0.01;
    //! INS Position Height 7 and 8
    char INS_Height_POI7[] = {local_data[576],local_data[577],local_data[578],local_data[579]};
    memcpy(&message.INSHeight_7 , &INS_Height_POI7, sizeof(message.INSHeight_7));
    message.fINSHeight_7 = message.INSHeight_7 * 0.01;
    char INS_Height_POI8[] = {local_data[580],local_data[581],local_data[582],local_data[583]};
    memcpy(&message.INSHeight_8 , &INS_Height_POI8, sizeof(message.INSHeight_8));
    message.fINSHeight_8 = message.INSHeight_8 * 0.01;
}

/// \file
/// \brief  getINSTimeUTC function -ADMA INS Time UTC
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSTimeUTC(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Time UTC
    char INS_Time_msec[] = {local_data[584],local_data[585],local_data[586],local_data[587]};
    memcpy(&message.INSTimemsec , &INS_Time_msec, sizeof(message.INSTimemsec));
    char INS_Time_Week[] = {local_data[588],local_data[589]};
    memcpy(&message.INSTimeWeek , &INS_Time_Week, sizeof(message.INSTimeWeek));
    char Leap_Seconds[] = {local_data[590],local_data[591]};
    memcpy(&message.LeapSeconds , &Leap_Seconds, sizeof(message.LeapSeconds));
}

/// \file
/// \brief  getINSPositionAbs function -  ADMA INS Pos Abs
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPositionAbs(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Position Abs
    char INS_Lat_Abs[] = {local_data[592],local_data[593],local_data[594],local_data[595]};
    memcpy(&message.INSLatAbs , &INS_Lat_Abs, sizeof(message.INSLatAbs));
    message.fINSLatAbs = message.INSLatAbs * 0.0000001;
    char INS_Lon_Abs[] = {local_data[596],local_data[597],local_data[598],local_data[599]};
    memcpy(&message.INSLonAbs , &INS_Lon_Abs, sizeof(message.INSLonAbs));
    message.fINSLonAbs = message.INSLonAbs * 0.0000001;
}

/// \file
/// \brief  getINSPosRel function -  ADMA INS Pos Rel 
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosRel(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Position Rel 
    char INS_Lat_Rel[] = {local_data[600],local_data[601],local_data[602],local_data[603]};
    memcpy(&message.INSLatRel , &INS_Lat_Rel, sizeof(message.INSLatRel));
    message.fINSLatRel = message.INSLatRel * 0.01;
    char INS_Lon_Rel[] = {local_data[604],local_data[605],local_data[606],local_data[607]};
    memcpy(&message.INSLonRel , &INS_Lon_Rel, sizeof(message.INSLonRel));
    message.fINSLonRel = message.INSLonRel * 0.01;
}

/// \file
/// \brief  getINSPosPOI1 function  ADMA INS Pos Abs and Rel POI 1
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI1(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI1
    char INS_Lat_Abs_POI1[] = {local_data[608],local_data[609],local_data[610],local_data[611]};
    memcpy(&message.INSLatAbs_1 , &INS_Lat_Abs_POI1, sizeof(message.INSLatAbs_1));
    message.fINSLatAbs_1 = message.INSLatAbs_1 * 0.0000001;
    char INS_Lon_Abs_POI1[] = {local_data[612],local_data[613],local_data[614],local_data[615]};
    memcpy(&message.INSLonAbs_1 , &INS_Lon_Abs_POI1, sizeof(message.INSLonAbs_1));
    message.fINSLonAbs_1 = message.INSLonAbs_1 * 0.0000001;
    //! INS Position Rel POI1
    char INS_Lat_Rel_POI1[] = {local_data[616],local_data[617],local_data[618],local_data[619]};
    memcpy(&message.INSLatRel_1 , &INS_Lat_Rel_POI1, sizeof(message.INSLatRel_1));
    message.fINSLatRel_1 = message.INSLatRel_1 * 0.01;
    char INS_Lon_Rel_POI1[] = {local_data[620],local_data[621],local_data[622],local_data[623]};
    memcpy(&message.INSLonRel_1 , &INS_Lon_Rel_POI1, sizeof(message.INSLonRel_1));
    message.fINSLonRel_1 = message.INSLonRel_1 * 0.01;
}

/// \file
/// \brief  getINSPosPOI2 function  ADMA INS Pos Abs and Rel POI 2
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI2(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI2
    char INS_Lat_Abs_POI2[] = {local_data[624],local_data[625],local_data[626],local_data[627]};
    memcpy(&message.INSLatAbs_2 , &INS_Lat_Abs_POI2, sizeof(message.INSLatAbs_2));
    message.fINSLatAbs_2 = message.INSLatAbs_2 * 0.0000001;
    char INS_Lon_Abs_POI2[] = {local_data[628],local_data[629],local_data[630],local_data[631]};
    memcpy(&message.INSLonAbs_2 , &INS_Lon_Abs_POI2, sizeof(message.INSLonAbs_2));
    message.fINSLonAbs_2 = message.INSLonAbs_2 * 0.0000001;
    //! INS Position Rel POI2
    char INS_Lat_Rel_POI2[] = {local_data[632],local_data[633],local_data[634],local_data[635]};
    memcpy(&message.INSLatRel_2 , &INS_Lat_Rel_POI2, sizeof(message.INSLatRel_2));
    message.fINSLatRel_2 = message.INSLatRel_2 * 0.01;
    char INS_Lon_Rel_POI2[] = {local_data[636],local_data[637],local_data[638],local_data[639]};
    memcpy(&message.INSLonRel_2 , &INS_Lon_Rel_POI2, sizeof(message.INSLonRel_2));
    message.fINSLonRel_2 = message.INSLonRel_2 * 0.01;
}

/// \file
/// \brief  getINSPosPOI3 function  ADMA INS Pos Abs and Rel POI 3
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI3(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI3
    char INS_Lat_Abs_POI3[] = {local_data[640],local_data[641],local_data[642],local_data[643]};
    memcpy(&message.INSLatAbs_3 , &INS_Lat_Abs_POI3, sizeof(message.INSLatAbs_3));
    message.fINSLatAbs_3 = message.INSLatAbs_3 * 0.0000001;
    char INS_Lon_Abs_POI3[] = {local_data[644],local_data[645],local_data[646],local_data[647]};
    memcpy(&message.INSLonAbs_3 , &INS_Lon_Abs_POI3, sizeof(message.INSLonAbs_3));
    message.fINSLonAbs_3 = message.INSLonAbs_3 * 0.0000001;
    //! INS Position Rel POI3
    char INS_Lat_Rel_POI3[] = {local_data[648],local_data[649],local_data[650],local_data[651]};
    memcpy(&message.INSLatRel_3 , &INS_Lat_Rel_POI3, sizeof(message.INSLatRel_3));
    message.fINSLatRel_3 = message.INSLatRel_3 * 0.01;
    char INS_Lon_Rel_POI3[] = {local_data[652],local_data[653],local_data[654],local_data[655]};
    memcpy(&message.INSLonRel_3 , &INS_Lon_Rel_POI3, sizeof(message.INSLonRel_3));
    message.fINSLonRel_3 = message.INSLonRel_3 * 0.01;
}

/// \file
/// \brief  getINSPosPOI4 function  ADMA INS Pos Abs and Rel POI 4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI4(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI4
    char INS_Lat_Abs_POI4[] = {local_data[656],local_data[657],local_data[658],local_data[659]};
    memcpy(&message.INSLatAbs_4 , &INS_Lat_Abs_POI4, sizeof(message.INSLatAbs_4));
    message.fINSLatAbs_4 = message.INSLatAbs_4 * 0.0000001;
    char INS_Lon_Abs_POI4[] = {local_data[660],local_data[661],local_data[662],local_data[663]};
    memcpy(&message.INSLonAbs_4 , &INS_Lon_Abs_POI4, sizeof(message.INSLonAbs_4));
    message.fINSLonAbs_4 = message.INSLonAbs_4 * 0.0000001;
    //! INS Position Rel POI4
    char INS_Lat_Rel_POI4[] = {local_data[664],local_data[665],local_data[666],local_data[667]};
    memcpy(&message.INSLatRel_4 , &INS_Lat_Rel_POI4, sizeof(message.INSLatRel_4));
    message.fINSLatRel_4 = message.INSLatRel_4 * 0.01;
    char INS_Lon_Rel_POI4[] = {local_data[668],local_data[669],local_data[670],local_data[671]};
    memcpy(&message.INSLonRel_4 , &INS_Lon_Rel_POI4, sizeof(message.INSLonRel_4));
    message.fINSLonRel_4 = message.INSLonRel_4 * 0.01;
}

/// \file
/// \brief  getINSPosPOI5 function  ADMA INS Pos Abs and Rel POI 5
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI5(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI5
    char INS_Lat_Abs_POI5[] = {local_data[672],local_data[673],local_data[674],local_data[675]};
    memcpy(&message.INSLatAbs_5 , &INS_Lat_Abs_POI5, sizeof(message.INSLatAbs_5));
    message.fINSLatAbs_5 = message.INSLatAbs_5 * 0.0000001;
    char INS_Lon_Abs_POI5[] = {local_data[676],local_data[677],local_data[678],local_data[679]};
    memcpy(&message.INSLonAbs_5 , &INS_Lon_Abs_POI5, sizeof(message.INSLonAbs_5));
    message.fINSLonAbs_5 = message.INSLonAbs_5 * 0.0000001;
    //! INS Position Rel POI5
    char INS_Lat_Rel_POI5[] = {local_data[680],local_data[681],local_data[682],local_data[683]};
    memcpy(&message.INSLatRel_5 , &INS_Lat_Rel_POI5, sizeof(message.INSLatRel_5));
    message.fINSLatRel_5 = message.INSLatRel_5 * 0.01;
    char INS_Lon_Rel_POI5[] = {local_data[684],local_data[685],local_data[686],local_data[687]};
    memcpy(&message.INSLonRel_5 , &INS_Lon_Rel_POI5, sizeof(message.INSLonRel_5));
    message.fINSLonRel_5 = message.INSLonRel_5 * 0.01;
}

/// \file
/// \brief  getINSPosPOI6 function  ADMA INS Pos Abs and Rel POI 6
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI6(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI6
    char INS_Lat_Abs_POI6[] = {local_data[688],local_data[689],local_data[690],local_data[691]};
    memcpy(&message.INSLatAbs_6 , &INS_Lat_Abs_POI6, sizeof(message.INSLatAbs_6));
    message.fINSLatAbs_6 = message.INSLatAbs_6 * 0.0000001;
    char INS_Lon_Abs_POI6[] = {local_data[692],local_data[693],local_data[694],local_data[695]};
    memcpy(&message.INSLonAbs_6 , &INS_Lon_Abs_POI6, sizeof(message.INSLonAbs_6));
    message.fINSLonAbs_6 = message.INSLonAbs_6 * 0.0000001;
    //! INS Position Rel POI6
    char INS_Lat_Rel_POI6[] = {local_data[696],local_data[697],local_data[698],local_data[699]};
    memcpy(&message.INSLatRel_6 , &INS_Lat_Rel_POI6, sizeof(message.INSLatRel_6));
    message.fINSLatRel_6 = message.INSLatRel_6 * 0.01;
    char INS_Lon_Rel_POI6[] = {local_data[700],local_data[701],local_data[702],local_data[703]};
    memcpy(&message.INSLonRel_6 , &INS_Lon_Rel_POI6, sizeof(message.INSLonRel_6));
    message.fINSLonRel_6 = message.INSLonRel_6 * 0.01;
}

/// \file
/// \brief  getINSPosPOI7 function  ADMA INS Pos Abs and Rel POI 7
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI7(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI7
    char INS_Lat_Abs_POI7[] = {local_data[704],local_data[705],local_data[706],local_data[707]};
    memcpy(&message.INSLatAbs_7 , &INS_Lat_Abs_POI7, sizeof(message.INSLatAbs_7));
    message.fINSLatAbs_7 = message.INSLatAbs_7 * 0.0000001;
    char INS_Lon_Abs_POI7[] = {local_data[708],local_data[709],local_data[710],local_data[711]};
    memcpy(&message.INSLonAbs_7 , &INS_Lon_Abs_POI7, sizeof(message.INSLonAbs_7));
    message.fINSLonAbs_7 = message.INSLonAbs_7 * 0.0000001;
    //! INS Position Rel POI7
    char INS_Lat_Rel_POI7[] = {local_data[712],local_data[713],local_data[714],local_data[715]};
    memcpy(&message.INSLatRel_7 , &INS_Lat_Rel_POI7, sizeof(message.INSLatRel_7));
    message.fINSLatRel_7 = message.INSLatRel_7 * 0.01;
    char INS_Lon_Rel_POI7[] = {local_data[716],local_data[717],local_data[718],local_data[719]};
    memcpy(&message.INSLonRel_7 , &INS_Lon_Rel_POI7, sizeof(message.INSLonRel_7));
    message.fINSLonRel_7 = message.INSLonRel_7 * 0.01;
}

/// \file
/// \brief  getINSPosPOI8 function - ADMA INS Pos Abs and Rel POI 8
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSPosPOI8(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Postion Abs POI8
    char INS_Lat_Abs_POI8[] = {local_data[720],local_data[721],local_data[722],local_data[723]};
    memcpy(&message.INSLatAbs_8 , &INS_Lat_Abs_POI8, sizeof(message.INSLatAbs_8));
    message.fINSLatAbs_8 = message.INSLatAbs_8 * 0.0000001;
    char INS_Lon_Abs_POI8[] = {local_data[724],local_data[725],local_data[726],local_data[727]};
    memcpy(&message.INSLonAbs_8 , &INS_Lon_Abs_POI8, sizeof(message.INSLonAbs_8));
    message.fINSLonAbs_8 = message.INSLonAbs_8 * 0.0000001;
    //! INS Position Rel POI8
    char INS_Lat_Rel_POI8[] = {local_data[728],local_data[729],local_data[730],local_data[731]};
    memcpy(&message.INSLatRel_8 , &INS_Lat_Rel_POI8, sizeof(message.INSLatRel_8));
    message.fINSLatRel_8 = message.INSLatRel_8 * 0.01;
    char INS_Lon_Rel_POI8[] = {local_data[732],local_data[733],local_data[734],local_data[735]};
    memcpy(&message.INSLonRel_8 , &INS_Lon_Rel_POI8, sizeof(message.INSLonRel_8));
    message.fINSLonRel_8 = message.INSLonRel_8 * 0.01;
}

/// \file
/// \brief  getINSVelHorXYZ function - ADMA INS Vel Hor X Y Z
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZ(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal
    char INS_Vel_Hor_X[] = {local_data[736],local_data[737]};
    memcpy(&message.INSVelHorX , &INS_Vel_Hor_X, sizeof(message.INSVelHorX));
    message.fINSVelHorX = message.INSVelHorX * 0.005;
    char INS_Vel_Hor_Y[] = {local_data[738],local_data[739]};
    memcpy(&message.INSVelHorY , &INS_Vel_Hor_Y, sizeof(message.INSVelHorY));
    message.fINSVelHorY = message.INSVelHorY * 0.005;
    char INS_Vel_Hor_Z[] = {local_data[740],local_data[741]};
    memcpy(&message.INSVelHorZ , &INS_Vel_Hor_Z, sizeof(message.INSVelHorZ));
    message.fINSVelHorZ = message.INSVelHorZ * 0.005;  
} 

/// \file
/// \brief  getINSVelFrameXYZ function - ADMA INS Vel Frame X Y Z
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelFrameXYZ(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Frame
    char INS_Vel_Frame_X[] = {local_data[744],local_data[745]};
    memcpy(&message.INSVelFrameX , &INS_Vel_Frame_X, sizeof(message.INSVelFrameX));
    message.fINSVelFrameX = message.INSVelFrameX * 0.005;
    char INS_Vel_Frame_Y[] = {local_data[746],local_data[747]};
    memcpy(&message.INSVelFrameX , &INS_Vel_Frame_Y, sizeof(message.INSVelFrameX));
    message.fINSVelFrameX = message.INSVelFrameX * 0.005;
    char INS_Vel_Frame_Z[] = {local_data[748],local_data[749]};
    memcpy(&message.INSVelFrameX , &INS_Vel_Frame_Z, sizeof(message.INSVelFrameX));
    message.fINSVelFrameX = message.INSVelFrameX * 0.005;
}

/// \file
/// \brief  getINSVelHorXYZPOS1 function - ADMA INS Vel Hor X Y Z POS1
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS1(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI1
    char INS_Vel_Hor_X_POI1[] = {local_data[752],local_data[753]};
    memcpy(&message.INSVelHorX_1 , &INS_Vel_Hor_X_POI1, sizeof(message.INSVelHorX_1));
    message.fINSVelHorX_1 = message.INSVelHorX_1 * 0.005;
    char INS_Vel_Hor_Y_POI1[] = {local_data[754],local_data[755]};
    memcpy(&message.INSVelHorY_1 , &INS_Vel_Hor_Y_POI1, sizeof(message.INSVelHorY_1));
    message.fINSVelHorY_1 = message.INSVelHorY_1 * 0.005;
    char INS_Vel_Hor_Z_POI1[] = {local_data[756],local_data[757]};
    memcpy(&message.INSVelHorZ_1 , &INS_Vel_Hor_Z_POI1, sizeof(message.INSVelHorZ_1));
    message.fINSVelHorZ_1 = message.INSVelHorZ_1 * 0.005;
}

/// \file
/// \brief  getINSVelHorXYZPOS2 function - ADMA INS Vel Hor X Y Z POS2
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS2(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI2
    char INS_Vel_Hor_X_POI2[] = {local_data[760],local_data[761]};
    memcpy(&message.INSVelHorX_2 , &INS_Vel_Hor_X_POI2, sizeof(message.INSVelHorX_2));
    message.fINSVelHorX_2 = message.INSVelHorX_2 * 0.005;
    char INS_Vel_Hor_Y_POI2[] = {local_data[762],local_data[763]};
    memcpy(&message.INSVelHorY_2 , &INS_Vel_Hor_Y_POI2, sizeof(message.INSVelHorY_2));
    message.fINSVelHorY_2 = message.INSVelHorY_2 * 0.005;
    char INS_Vel_Hor_Z_POI2[] = {local_data[764],local_data[765]};
    memcpy(&message.INSVelHorZ_2 , &INS_Vel_Hor_Z_POI2, sizeof(message.INSVelHorZ_2));
    message.fINSVelHorZ_2 = message.INSVelHorZ_2 * 0.005;
}

/// \file
/// \brief  getINSVelHorXYZPOS3 function - ADMA INS Vel Hor X Y Z POS3
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS3(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI3
    char INS_Vel_Hor_X_POI3[] = {local_data[768],local_data[769]};
    memcpy(&message.INSVelHorX_3 , &INS_Vel_Hor_X_POI3, sizeof(message.INSVelHorX_3));
    message.fINSVelHorX_3 = message.INSVelHorX_3 * 0.005;
    char INS_Vel_Hor_Y_POI3[] = {local_data[770],local_data[771]};
    memcpy(&message.INSVelHorY_3 , &INS_Vel_Hor_Y_POI3, sizeof(message.INSVelHorY_3));
    message.fINSVelHorY_3 = message.INSVelHorY_3 * 0.005;
    char INS_Vel_Hor_Z_POI3[] = {local_data[772],local_data[773]};
    memcpy(&message.INSVelHorZ_3 , &INS_Vel_Hor_Z_POI3, sizeof(message.INSVelHorZ_3));
    message.fINSVelHorZ_3 = message.INSVelHorZ_3 * 0.005;
}

/// \file
/// \brief  getINSVelHorXYZPOS4 function - ADMA INS Vel Hor X Y Z POS4
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS4(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI4
    char INS_Vel_Hor_X_POI4[] = {local_data[776],local_data[777]};
    memcpy(&message.INSVelHorX_4 , &INS_Vel_Hor_X_POI4, sizeof(message.INSVelHorX_4));
    message.fINSVelHorX_4 = message.INSVelHorX_4 * 0.005;
    char INS_Vel_Hor_Y_POI4[] = {local_data[778],local_data[779]};
    memcpy(&message.INSVelHorY_4 , &INS_Vel_Hor_Y_POI4, sizeof(message.INSVelHorY_4));
    message.fINSVelHorY_4 = message.INSVelHorY_4 * 0.005;
    char INS_Vel_Hor_Z_POI4[] = {local_data[780],local_data[781]};
    memcpy(&message.INSVelHorZ_4 , &INS_Vel_Hor_Z_POI4, sizeof(message.INSVelHorZ_4));
    message.fINSVelHorZ_4 = message.INSVelHorZ_4 * 0.005;
}


/// \file
/// \brief  getINSVelHorXYZPOS5 function - ADMA INS Vel Hor X Y Z POS5
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS5(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI5
    char INS_Vel_Hor_X_POI5[] = {local_data[784],local_data[785]};
    memcpy(&message.INSVelHorX_5 , &INS_Vel_Hor_X_POI5, sizeof(message.INSVelHorX_5));
    message.fINSVelHorX_5 = message.INSVelHorX_5 * 0.005;
    char INS_Vel_Hor_Y_POI5[] = {local_data[786],local_data[787]};
    memcpy(&message.INSVelHorY_5 , &INS_Vel_Hor_Y_POI5, sizeof(message.INSVelHorY_5));
    message.fINSVelHorY_5 = message.INSVelHorY_5 * 0.005;
    char INS_Vel_Hor_Z_POI5[] = {local_data[788],local_data[789]};
    memcpy(&message.INSVelHorZ_5 , &INS_Vel_Hor_Z_POI5, sizeof(message.INSVelHorZ_5));
    message.fINSVelHorZ_5 = message.INSVelHorZ_5 * 0.005;
}

/// \file
/// \brief  getINSVelHorXYZPOS6 function - ADMA INS Vel Hor X Y Z POS6
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS6(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI6
    char INS_Vel_Hor_X_POI6[] = {local_data[792],local_data[793]};
    memcpy(&message.INSVelHorX_6 , &INS_Vel_Hor_X_POI6, sizeof(message.INSVelHorX_6));
    message.fINSVelHorX_6 = message.INSVelHorX_6 * 0.005;
    char INS_Vel_Hor_Y_POI6[] = {local_data[794],local_data[795]};
    memcpy(&message.INSVelHorY_6 , &INS_Vel_Hor_Y_POI6, sizeof(message.INSVelHorY_6));
    message.fINSVelHorY_6 = message.INSVelHorY_6 * 0.005;
    char INS_Vel_Hor_Z_POI6[] = {local_data[796],local_data[797]};
    memcpy(&message.INSVelHorZ_6 , &INS_Vel_Hor_Z_POI6, sizeof(message.INSVelHorZ_6));
    message.fINSVelHorZ_6 = message.INSVelHorZ_6 * 0.005;
}


/// \file
/// \brief  getINSVelHorXYZPOS7 function - ADMA INS Vel Hor X Y Z POS7
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS7(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI7
    char INS_Vel_Hor_X_POI7[] = {local_data[800],local_data[801]};
    memcpy(&message.INSVelHorX_7 , &INS_Vel_Hor_X_POI7, sizeof(message.INSVelHorX_7));
    message.fINSVelHorX_7 = message.INSVelHorX_7 * 0.005;
    char INS_Vel_Hor_Y_POI7[] = {local_data[802],local_data[803]};
    memcpy(&message.INSVelHorY_7 , &INS_Vel_Hor_Y_POI7, sizeof(message.INSVelHorY_7));
    message.fINSVelHorY_7 = message.INSVelHorY_7 * 0.005;
    char INS_Vel_Hor_Z_POI7[] = {local_data[804],local_data[805]};
    memcpy(&message.INSVelHorZ_7 , &INS_Vel_Hor_Z_POI7, sizeof(message.INSVelHorZ_7));
    message.fINSVelHorZ_7 = message.INSVelHorZ_7 * 0.005;
}


/// \file
/// \brief  getINSVelHorXYZPOS8 function - ADMA INS Vel Hor X Y Z POS8
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSVelHorXYZPOS8(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS Velocity Horizontal POI8
    char INS_Vel_Hor_X_POI8[] = {local_data[808],local_data[809]};
    memcpy(&message.INSVelHorX_8 , &INS_Vel_Hor_X_POI8, sizeof(message.INSVelHorX_8));
    message.fINSVelHorX_8 = message.INSVelHorX_8 * 0.005;
    char INS_Vel_Hor_Y_POI8[] = {local_data[810],local_data[811]};
    memcpy(&message.INSVelHorY_8 , &INS_Vel_Hor_Y_POI8, sizeof(message.INSVelHorY_8));
    message.fINSVelHorY_8 = message.INSVelHorY_8 * 0.005;
    char INS_Vel_Hor_Z_POI8[] = {local_data[812],local_data[813]};
    memcpy(&message.INSVelHorZ_8 , &INS_Vel_Hor_Z_POI8, sizeof(message.INSVelHorZ_8));
    message.fINSVelHorZ_8 = message.INSVelHorZ_8 * 0.005;
}

/// \file
/// \brief  getINSEPE function - ADMA INS SEPE
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getINSEPE(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS EPE
    char INS_Stddev_Lat[] = {local_data[816],local_data[817]};
    memcpy(&message.INSStddevLat , &INS_Stddev_Lat, sizeof(message.INSStddevLat));
    message.fINSStddevLat = message.INSStddevLat * 0.01;
    char INS_Stddev_Lon[] = {local_data[818],local_data[819]};
    memcpy(&message.INSStddevLong , &INS_Stddev_Lon, sizeof(message.INSStddevLong));
    message.fINSStddevLong = message.INSStddevLong * 0.01;
    char INS_Stddev_Height[] = {local_data[820],local_data[821]};
    memcpy(&message.INSStddevHeight , &INS_Stddev_Height, sizeof(message.INSStddevHeight));
    message.fINSStddevHeight = message.INSStddevHeight * 0.01;
}

/// \file
/// \brief  getINSEVEandETE function - ADMA INS EVE and ETE
/// \param  message ADMA Message to be loaded
void getINSEVEandETE(const std::string& local_data, adma_connect::Adma& message)
{
    //! INS EVE and INS ETE
    char INS_Stddev_Vel_X[] = {local_data[824]};
    memcpy(&message.INSStddevVelX , &INS_Stddev_Vel_X, sizeof(message.INSStddevVelX));
    message.fINSStddevVelX = message.INSStddevVelX * 0.01;  
    char INS_Stddev_Vel_Y[] = {local_data[825]};
    memcpy(&message.INSStddevVelY , &INS_Stddev_Vel_Y, sizeof(message.INSStddevVelY));
    message.fINSStddevVelY = message.INSStddevVelY * 0.01;
    char INS_Stddev_Vel_Z[] = {local_data[826]};
    memcpy(&message.INSStddevVelZ , &INS_Stddev_Vel_Z, sizeof(message.INSStddevVelZ));
    message.fINSStddevVelZ = message.INSStddevVelZ * 0.01;
    char INS_Stddev_Roll[] = {local_data[827]};
    memcpy(&message.INSStddevRoll , &INS_Stddev_Roll, sizeof(message.INSStddevRoll));
    message.fINSStddevRoll = message.INSStddevRoll * 0.01;
    char INS_Stddev_Pitch[] = {local_data[828]};
    memcpy(&message.INSStddevPitch , &INS_Stddev_Pitch, sizeof(message.INSStddevPitch));
    message.fINSStddevPitch = message.INSStddevPitch * 0.01;
    char INS_Stddev_Yaw[] = {local_data[829]};
    memcpy(&message.INSStddevYaw , &INS_Stddev_Yaw, sizeof(message.INSStddevYaw));
    message.fINSStddevYaw = message.INSStddevYaw * 0.01;
}

/// \file
/// \brief  getAnalog function - ADMA Analog
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getAnalog(const std::string& local_data, adma_connect::Adma& message)
{
    //! Analog In 1
    char AN1[] = {local_data[832],local_data[833]};
    memcpy(&message.AN1 , &AN1, sizeof(message.AN1));
    message.fAN1 = message.AN1 * 0.0005;
    char AN2[] = {local_data[834],local_data[835]};
    memcpy(&message.AN2 , &AN2, sizeof(message.AN2));
    message.fAN2 = message.AN2 * 0.0005;
    char AN3[] = {local_data[836],local_data[837]};
    memcpy(&message.AN3 , &AN3, sizeof(message.AN3));
    message.fAN3 = message.AN3 * 0.0005;
    char AN4[] = {local_data[838],local_data[839]};
    memcpy(&message.AN4 , &AN4, sizeof(message.AN4));
    message.fAN4 = message.AN4 * 0.0005;
}

/// \file
/// \brief  getKalmanFilter function - ADMA Kalman Filter
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getKalmanFilter(const std::string& local_data, adma_connect::Adma& message)
{
    //! Kalman Filer Status
    char KF_Lat_stimulated[] = {local_data[840]};
    memcpy(&message.KFLatStimulated , &KF_Lat_stimulated, sizeof(message.KFLatStimulated));
    char KF_Lon_stimulated[] = {local_data[841]};
    memcpy(&message.KFLongStimulated , &KF_Lon_stimulated, sizeof(message.KFLongStimulated));
    char KF_steady_state[] = {local_data[842]};
    memcpy(&message.KFSteadyState , &KF_steady_state, sizeof(message.KFSteadyState));
}

/// \file
/// \brief  getGNSSReceiver function - ADMA GPS Error Status
/// \param  local_data ADMA string
/// \param  message ADMA Message to be loaded
void getGNSSReceiver(const std::string& local_data, adma_connect::Adma& message)
{
    //! GPS Receiver Error
    char GPS_Receiver_Error[] = {local_data[848],local_data[849],local_data[850],local_data[851]};
    memcpy(&message.GPSReceiverError , &GPS_Receiver_Error, sizeof(message.GPSReceiverError));
    char GPS_Receiver_Status[] = {local_data[852],local_data[853],local_data[854],local_data[855]};
    memcpy(&message.GPSReceiverStatus , &GPS_Receiver_Status, sizeof(message.GPSReceiverStatus));
}

/// \file
/// \brief  Bit Shift function
/// \param  byte Byte information
/// \param  position message
/// \return an integer 0 upon exit success
bool getBit(unsigned char byte, int position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

