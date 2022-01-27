#include "adma_ros_driver/adma_parse.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cstring>
#include <iostream>
#include <sstream>
#include <math.h>
using namespace std;

#define PI 3.1415926535897932384626433832795028841971f

void getparseddata(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix, std_msgs::msg::Float64& msg_heading, std_msgs::msg::Float64& msg_velocity)
{
    getadmastaticheader(local_data,message);
    getadmadynamicheader(local_data,message);
    getstatusgps(local_data,message);
    getstatustrigger(local_data,message);
    getstatuscount(local_data,message); 
    getevkstatus(local_data,message);
    geterrorandwarning(local_data,message);
    getsensorbodyxyz(local_data,message);
    getratesbodyxyz(local_data,message);
    getgpsabs(local_data,message);
    getrateshorizontalxyz(local_data,message);
    getaccelerationbodypoi1(local_data,message);
    getaccelerationbodypoi2(local_data,message);
    getaccelerationbodypoi3(local_data,message);
    getaccelerationbodypoi4(local_data,message);
    getaccelerationbodypoi5(local_data,message);
    getaccelerationbodypoi6(local_data,message);
    getaccelerationbodypoi7(local_data,message);
    getaccelerationbodypoi8(local_data,message);
    getaccelerationhorpoi1(local_data,message);
    getaccelerationhorpoi2(local_data,message);
    getaccelerationhorpoi3(local_data,message);
    getaccelerationhorpoi4(local_data,message);
    getaccelerationhorpoi5(local_data,message);
    getaccelerationhorpoi6(local_data,message);
    getaccelerationhorpoi7(local_data,message);
    getaccelerationhorpoi8(local_data,message);
    getexternalvelocityanalog(local_data,message);
    getexternalvecovitydigpulses(local_data,message);
    getexternalvelocitycorrected(local_data,message);
    getbarometerpressure(local_data,message);
    getbarometerheight(local_data,message);
    getmiscellaneuos(local_data,message);
    getmiscellaneuospoi1(local_data,message);
    getmiscellaneuospoi2(local_data,message);
    getmiscellaneuospoi3(local_data,message);
    getmiscellaneuospoi4(local_data,message);
    getmiscellaneuospoi5(local_data,message);
    getmiscellaneuospoi6(local_data,message);
    getmiscellaneuospoi7(local_data,message);
    getmiscellaneuospoi8(local_data,message);
    gettriggers(local_data,message);
    getsystemdata(local_data,message);
    getgpsposrel(local_data,message);
    getgpsepe(local_data,message);
    getgpsvelframe(local_data,message,msg_velocity);
    getgpsveleve(local_data,message);
    getgpstimeutc(local_data,message);
    getgpsauxdata1(local_data,message);
    getgpsauxdata2(local_data,message);
    getinsanglegpscog(local_data,message, msg_heading);
    getgpsheight(local_data,message);
    getgpsdualanttimeutc(local_data,message);
    getgpsdualantangle(local_data,message);
    getgpsdualantangleete(local_data,message);
    getinspositionheight(local_data,message);
    getinspositionpoi(local_data,message);
    getinstimeutc(local_data,message);
    getinspositionabs(local_data,message,msg_fix);
    getinsposrel(local_data,message);
    getinspospoi1(local_data,message);
    getinspospoi2(local_data,message);
    getinspospoi3(local_data,message);
    getinspospoi4(local_data,message);
    getinspospoi5(local_data,message);
    getinspospoi6(local_data,message);
    getinspospoi7(local_data,message);
    getinspospoi8(local_data,message);
    getinsvelhorxyz(local_data,message);
    getinsvelframexyz(local_data,message);
    getinsvelhorxyzpos1(local_data,message);
    getinsvelhorxyzpos2(local_data,message);
    getinsvelhorxyzpos3(local_data,message);
    getinsvelhorxyzpos4(local_data,message);
    getinsvelhorxyzpos5(local_data,message);
    getinsvelhorxyzpos6(local_data,message);
    getinsvelhorxyzpos7(local_data,message);
    getinsvelhorxyzpos8(local_data,message);
    getinsepe(local_data,message);
    getinseveandete(local_data,message);
    getanalog(local_data,message);
    getkalmanfilter(local_data,message);
    getgnssreceiver(local_data,message);

}

/// \file
/// \brief  getadmastaticheader function - adma static header information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getadmastaticheader(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    char genesysid[] = {local_data[0],local_data[1],local_data[2],local_data[3]};
    char formatid[] = {local_data[8],local_data[9],local_data[10],local_data[11]};
    memcpy(&message.formatid , &formatid, sizeof(message.formatid));
    char serialno[] = {local_data[32],local_data[33],local_data[34],local_data[35]};
    memcpy(&message.serialno , &serialno, sizeof(message.serialno));
    message.genesysid =  genesysid;
    std::stringstream ss_hv;
    ss_hv <<  int(local_data[4]) << int(local_data[5]) << int(local_data[6]) << int(local_data[15]);
    message.headerversion = ss_hv.str();
    std::stringstream ss_fv;
    ss_fv <<  int(local_data[12]) << int(local_data[13]) << int(local_data[14]) << int(local_data[15]);
    message.formatversion = ss_fv.str();

}

/// \file
/// \brief  getadmastaticheader function - adma static header information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getadmadynamicheader(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    // ! adma dynamic header
    char configid[] = {local_data[68],local_data[69],local_data[70],local_data[71]};
    memcpy(&message.configid , &configid, sizeof(message.configid));
    char configformat[] = {local_data[72],local_data[73],local_data[74],local_data[75]};
    memcpy(&message.configformat , &configformat, sizeof(message.configformat));
    char configversion[] = {local_data[76],local_data[77],local_data[78],local_data[79]};
    memcpy(&message.configversion , &configversion, sizeof(message.configversion));
    char configsize[] = {local_data[80],local_data[81],local_data[82],local_data[83]};
    memcpy(&message.configsize , &configsize, sizeof(message.configsize));
    char byteoffset[] = {local_data[84],local_data[85],local_data[86],local_data[87]};
    memcpy(&message.byteoffset , &byteoffset, sizeof(message.byteoffset));
    char slicesize[] = {local_data[88],local_data[89],local_data[90],local_data[91]};
    memcpy(&message.slicesize , &slicesize, sizeof(message.slicesize));
    char slicedata[] = {local_data[92],local_data[93],local_data[94],local_data[95]};

}

/// \file
/// \brief  getstatusgps function - adma status information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatusgps(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statusgps;
    char status_gps[] = {local_data[96]};
    memcpy(&statusgps, &status_gps, sizeof(statusgps));
    bool status_external_vel = getbit(statusgps,7);
    bool status_skidding = getbit(statusgps,5);
    bool standstill_c = getbit(statusgps,4);
    bool rtk_precise = getbit(statusgps,3);
    bool rtk_coarse = getbit(statusgps,2);
    bool gps_mode = getbit(statusgps,1);
    bool gps_out = getbit(statusgps,0);

    /* status gps mode */
    if(gps_out)
    {
        message.statusgpsmode = 1;
    }
    else if (gps_mode) 
    {
        message.statusgpsmode = 2;
    }
    else if (rtk_coarse) 
    {
        message.statusgpsmode = 3;
    }
    else if (rtk_precise) 
    {
        message.statusgpsmode = 4;
    }
    /* status stand still */
    message.statusstandstill = standstill_c;
    /* status skidding */
    message.statusskidding = status_skidding;
    /* status external velocity slip */
    message.statusexternalvelout = status_external_vel;
}


/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatustrigger(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statustriggergps;
    char statustrigger[] = {local_data[96]};
    memcpy(&statustriggergps, &statustrigger, sizeof(statustriggergps));
    bool status_synclock = getbit(statustriggergps,7);
    bool status_dead_reckoning = getbit(statustriggergps,6);
    bool status_ahrs_ins = getbit(statustriggergps,5);
    bool status_alignment = getbit(statustriggergps,4);
    bool status_signal_in1 = getbit(statustriggergps,3);
    bool status_signal_in2 = getbit(statustriggergps,2);
    bool status_signal_in3 = getbit(statustriggergps,1);
    bool status_trig_gps = getbit(statustriggergps,0);
    /* status statustriggps */
    message.statustriggps = status_trig_gps;
    /* status statussignalin3 */
    message.statussignalin3 = status_signal_in3;
    /* status statussignalin2 */
    message.statussignalin2 = status_signal_in2;
    /* status statussignalin1 */
    message.statussignalin1 = status_signal_in1;
    /* status statusalignment */
    message.statusalignment = status_alignment;
    /* status statusahrsins */
    message.statusahrsins = status_ahrs_ins;
    /* status statusdeadreckoning */
    message.statusdeadreckoning = status_dead_reckoning;
    /* status statussynclock */
    message.statussynclock = status_synclock;
}

/// \file
/// \brief  getstatustrigger function - adma gps trigger information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getevkstatus(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    unsigned char statusevk;
    char statusevkdata[] = {local_data[96]};
    memcpy(&statusevk, &statusevkdata, sizeof(statusevk));
    bool status_pos_b2 = getbit(statusevk,7);
    bool status_pos_b1 = getbit(statusevk,6);
    bool status_tilt_b2 = getbit(statusevk,5);
    bool status_tilt_b1 = getbit(statusevk,4);
    bool status_configuration_changed = getbit(statusevk,3);
    bool status_heading_executed = getbit(statusevk,2);
    bool status_evk_estimates = getbit(statusevk,1);
    bool status_evk_activ = getbit(statusevk,0);
    /* status statustriggps */
    message.statusevkactiv = status_evk_activ;
    /* status status_evk_estimates */
    message.statusevkestimates = status_evk_estimates;
    /* status status_heading_executed */
    message.statusheadingexecuted = status_heading_executed;
    /* status status_configuration_changed */
    message.statusconfigurationchanged = status_configuration_changed;
    /* status pos */
    if(status_tilt_b1==0 && status_tilt_b2==0)
    {
        message.statustilt = 0;
    }
    else if(status_tilt_b1==0 && status_tilt_b2==1)
    {
        message.statustilt = 1;
    }
    else if(status_tilt_b1==1 && status_tilt_b2==0)
    {
        message.statustilt = 2;
    }
        /* status tilt */
    if(status_pos_b1==0 && status_pos_b2==0)
    {
        message.statuspos = 0;
    }
    else if(status_pos_b1==0 && status_pos_b2==1)
    {
        message.statuspos = 1;
    }
    else if(status_pos_b1==1 && status_pos_b2==0)
    {
        message.statuspos = 2;
    }
}

/// \file
/// \brief  getstatuscount function - adma status count
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getstatuscount(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! status count
    char status_count[] = {local_data[99]};
    message.statuscount = int(local_data[99]);
    unsigned char statuskf;
    char status_kf[] = {local_data[100]};
    memcpy(&statuskf, &status_kf, sizeof(statuskf));
    bool status_speed_b2 = getbit(statuskf,5);
    bool status_speed_b1 = getbit(statuskf,4);
    bool status_kf_steady_state = getbit(statuskf,3);
    bool status_kf_long_stimulated = getbit(statuskf,2);
    bool status_kf_lat_stimulated = getbit(statuskf,1);
    bool status_kalmanfilter_settled = getbit(statuskf,0);
    message.statuskalmanfiltersetteled = status_kalmanfilter_settled;
    message.statuskflatstimulated = status_kf_lat_stimulated;
    message.statuskflongstimulated = status_kf_long_stimulated;
    message.statuskfsteadystate = status_kf_steady_state;
    if(status_speed_b1==0 && status_speed_b2==0)
    {
        message.statusspeed = 0;
    }
    else if(status_speed_b1==0 && status_speed_b2==1)
    {
        message.statusspeed = 1;
    }
    else if(status_speed_b1==1 && status_speed_b2==0)
    {
        message.statusspeed = 2;
    }
}

/// \file
/// \brief  geterrorandwarning function - adma error and warning
/// \param  local_data adma string
/// \param  message adma message to be loaded
void geterrorandwarning(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    std::bitset<8> bitdataerror1 = local_data[104];
    std::bitset<8> bitdataerror2 = local_data[105];
    std::bitset<8> bitdatawarn3 = local_data[106];
    std::bitset<8> errorhw = local_data[107];
    std::bitset<4> erhw1;
    std::bitset<4> ermisc1;
    std::bitset<4> ermisc2;
    std::bitset<4> ermisc3;
    std::bitset<4> warngps;
    std::bitset<4> warnmisc1;
    std::bitset<1> erhwsticky;

    for(size_t i=0;i<4;i++)
    {
        erhw1[i]    = bitdataerror1[i];
        ermisc1[i]  = bitdataerror1[i+4];
        ermisc2[i]  = bitdataerror2[i];
        ermisc3[i]  = bitdataerror2[i+4];
        warngps[i]  = bitdatawarn3[i];
        warnmisc1[i]  = bitdatawarn3[i+4];
    }
    erhwsticky[0] = errorhw[1];
    message.errorhardware = erhw1.to_string();
    message.error_misc1 = ermisc1.to_string();
    message.error_misc2 = ermisc2.to_string();
    message.error_misc3 = ermisc3.to_string();
    message.warngps = warngps.to_string();
    message.warnmisc1 = warnmisc1.to_string();
    message.errorhwsticky = erhwsticky.to_string();
}
/// \file
/// \brief  getsensorbodyxyz function - adma sensor body x y z acc and rate information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getsensorbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! sensor body x
    char acc_body_hr_x[] = {local_data[112],local_data[113],local_data[114],local_data[115]};
    memcpy(&message.accbodyhrx , &acc_body_hr_x, sizeof(message.accbodyhrx));
    message.faccbodyhrx = message.accbodyhrx * 0.0001;
    
    char rate_body_hr_x[] = {local_data[116],local_data[117],local_data[118],local_data[119]};
    memcpy(&message.ratebodyhrx , &rate_body_hr_x, sizeof(message.ratebodyhrx));
    message.fratebodyhrx = message.ratebodyhrx * 0.0001;

    //! sensor body y
    char acc_body_hr_y[] = {local_data[120],local_data[121],local_data[122],local_data[123]};
    memcpy(&message.accbodyhry , &acc_body_hr_y, sizeof(message.accbodyhry));
    message.faccbodyhry = message.accbodyhry * 0.0001;   
    
    char rate_body_hr_y[] = {local_data[124],local_data[125],local_data[126],local_data[127]};
    memcpy(&message.ratebodyhry , &rate_body_hr_y, sizeof(message.ratebodyhry));
    message.fratebodyhry = message.ratebodyhry * 0.0001;

    //! sensor body z
    char acc_body_hr_z[] = {local_data[128],local_data[129],local_data[130],local_data[131]};
    memcpy(&message.accbodyhrz , &acc_body_hr_z, sizeof(message.accbodyhrz));
    message.faccbodyhrz = message.accbodyhrz * 0.0001;

    char rate_body_hr_z[] = {local_data[132],local_data[133],local_data[134],local_data[135]};
    memcpy(&message.ratebodyhrz , &rate_body_hr_z, sizeof(message.ratebodyhrz));
    message.fratebodyhrz = message.ratebodyhrz * 0.0001;

}

/// \file
/// \brief  getratesbodyxyz function - adma rate body information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getratesbodyxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! rates body
    char rate_body_x[] = {local_data[136],local_data[137]};
    memcpy(&message.ratebodyx , &rate_body_x, sizeof(message.ratebodyx));
    message.fratebodyx = message.ratebodyx * 0.01;

    char rate_body_y[] = {local_data[138],local_data[139]};
    memcpy(&message.ratebodyy , &rate_body_y, sizeof(message.ratebodyy));
    message.fratebodyy = message.ratebodyy * 0.01;

    char rate_body_z[] = {local_data[140],local_data[141]};
    memcpy(&message.ratebodyz , &rate_body_z, sizeof(message.ratebodyz));
    message.fratebodyz = message.ratebodyz * 0.01;
}

/// \file
/// \brief  getrateshorizontalxyz function - adma rates horizontal
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getrateshorizontalxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! rates horizontal
    char rate_hor_x[] = {local_data[144],local_data[145]};
    memcpy(&message.ratehorx , &rate_hor_x, sizeof(message.ratehorx));
    message.fratehorx = message.ratehorx * 0.01;

    char rate_hor_y[] = {local_data[146],local_data[147]};
    memcpy(&message.ratehory , &rate_hor_y, sizeof(message.ratehory));
    message.fratehory = message.ratehory * 0.01;

    char rate_hor_z[] = {local_data[148],local_data[149]};
    memcpy(&message.ratehorz , &rate_hor_z, sizeof(message.ratehorz));
    message.fratehorz = message.ratehorz * 0.01;
}

/// \file
/// \brief  getaccelerationbody function - adma acc body
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbody(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{

    //! acceleration body
    char acceleration_body_x[] = {local_data[152],local_data[153]};
    memcpy(&message.accbodyx , &acceleration_body_x, sizeof(message.accbodyx));
    message.faccbodyx = message.accbodyx * 0.0004;    

    char acceleration_body_y[] = {local_data[154],local_data[155]};
    memcpy(&message.accbodyy , &acceleration_body_y, sizeof(message.accbodyy));
    message.faccbodyy = message.accbodyy * 0.0004;

    char acceleration_body_z[] = {local_data[156],local_data[157]};
    memcpy(&message.accbodyz , &acceleration_body_z, sizeof(message.accbodyz));
    message.faccbodyz = message.accbodyz * 0.0004;
}

/// \file
/// \brief  getaccelerationhor function - adma acc horizontal
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhor(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration horizontal
    char acceleration_hor_x[] = {local_data[160],local_data[161]};
    memcpy(&message.acchorx , &acceleration_hor_x, sizeof(message.acchorx));
    message.facchorx = message.acchorx * 0.0004;

    char acceleration_hor_y[] = {local_data[162],local_data[163]};
    memcpy(&message.acchory , &acceleration_hor_y, sizeof(message.acchory));
    message.facchory = message.acchory * 0.0004;

    char acceleration_hor_z[] = {local_data[164],local_data[165]};
    memcpy(&message.acchorz , &acceleration_hor_z, sizeof(message.acchorz));
    message.facchorz = message.acchorz * 0.0004;

}

/// \file
/// \brief  getaccelerationbody poi1 function - adma acc body poi1
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    char acceleration_body_x_poi1[] = {local_data[168],local_data[169]};
    memcpy(&message.accbodyx_1 , &acceleration_body_x_poi1, sizeof(message.accbodyx_1));
    message.faccbodyx_1 = message.accbodyx_1 * 0.0004;

    char acceleration_body_y_poi1[] = {local_data[170],local_data[171]};
    memcpy(&message.accbodyy_1 , &acceleration_body_y_poi1, sizeof(message.accbodyy_1));
    message.faccbodyy_1 = message.accbodyy_1 * 0.0004;

    char acceleration_body_z_poi1[] = {local_data[172],local_data[173]};
    memcpy(&message.accbodyz_1 , &acceleration_body_z_poi1, sizeof(message.accbodyz_1));
    message.faccbodyz_1 = message.accbodyz_1 * 0.0004;
}

/// \file
/// \brief  getaccelerationbody poi2 function - adma acc body poi2
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration poi2 body
    char acceleration_body_x_poi2[] = {local_data[176],local_data[177]};
    memcpy(&message.accbodyx_2 , &acceleration_body_x_poi2, sizeof(message.accbodyx_2));
    message.faccbodyx_2 = message.accbodyx_2 * 0.0004;

    char acceleration_body_y_poi2[] = {local_data[178],local_data[179]};
    memcpy(&message.accbodyy_2 , &acceleration_body_y_poi2, sizeof(message.accbodyy_2));
    message.faccbodyy_2 = message.accbodyy_2 * 0.0004;

    char acceleration_body_z_poi2[] = {local_data[180],local_data[181]};
    memcpy(&message.accbodyz_2 , &acceleration_body_z_poi2, sizeof(message.accbodyz_2));
    message.faccbodyz_2 = message.accbodyz_2 * 0.0004;

}

/// \file
/// \brief  getaccelerationbody poi3 function - adma acc body poi3
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{

    //! acceleration poi3 body
    char acceleration_body_x_poi3[] = {local_data[184],local_data[185]};
    memcpy(&message.accbodyx_3 , &acceleration_body_x_poi3, sizeof(message.accbodyx_3));
    message.faccbodyx_3 = message.accbodyx_3 * 0.0004;

    char acceleration_body_y_poi3[] = {local_data[186],local_data[187]};
    memcpy(&message.accbodyy_3 , &acceleration_body_y_poi3, sizeof(message.accbodyy_3));
    message.faccbodyy_3 = message.accbodyy_3 * 0.0004;

    char acceleration_body_z_poi3[] = {local_data[188],local_data[189]};
    memcpy(&message.accbodyz_3 , &acceleration_body_z_poi3, sizeof(message.accbodyz_3));
    message.faccbodyz_3 = message.accbodyz_3 * 0.0004;

}


/// \file
/// \brief  getaccelerationbody poi4 function - adma acc body poi4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{

    //! acceleration poi4 body
    char acceleration_body_x_poi4[] = {local_data[192],local_data[193]};
    memcpy(&message.accbodyx_4 , &acceleration_body_x_poi4, sizeof(message.accbodyx_4));
    message.faccbodyx_4 = message.accbodyx_4 * 0.0004;

    char acceleration_body_y_poi4[] = {local_data[194],local_data[195]};
    memcpy(&message.accbodyy_4 , &acceleration_body_y_poi4, sizeof(message.accbodyy_4));
    message.faccbodyy_4 = message.accbodyy_4 * 0.0004;

    char acceleration_body_z_poi4[] = {local_data[196],local_data[197]};
    memcpy(&message.accbodyz_4 , &acceleration_body_z_poi4, sizeof(message.accbodyz_4));
    message.faccbodyz_4 = message.accbodyz_4 * 0.0004;
}


/// \file
/// \brief  getaccelerationbody poi5 function - adma acc body poi5
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{

    //! acceleration poi5 body
    char acceleration_body_x_poi5[] = {local_data[200],local_data[201]};
    memcpy(&message.accbodyx_5 , &acceleration_body_x_poi5, sizeof(message.accbodyx_5));
    message.faccbodyx_5 = message.accbodyx_5 * 0.0004;

    char acceleration_body_y_poi5[] = {local_data[202],local_data[203]};
    memcpy(&message.accbodyy_5 , &acceleration_body_y_poi5, sizeof(message.accbodyy_5));
    message.faccbodyy_5 = message.accbodyy_5 * 0.0004;

    char acceleration_body_z_poi5[] = {local_data[204],local_data[205]};
    memcpy(&message.accbodyz_5 , &acceleration_body_z_poi5, sizeof(message.accbodyz_5));
    message.faccbodyz_5 = message.accbodyz_5 * 0.0004;

}

/// \file
/// \brief  getaccelerationbody poi6 function - adma acc body poi6
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration poi6 body
    char acceleration_body_x_poi6[] = {local_data[208],local_data[209]};
    memcpy(&message.accbodyx_6 , &acceleration_body_x_poi6, sizeof(message.accbodyx_6));
    message.faccbodyx_6 = message.accbodyx_6 * 0.0004;

    char acceleration_body_y_poi6[] = {local_data[210],local_data[211]};
    memcpy(&message.accbodyy_6 , &acceleration_body_y_poi6, sizeof(message.accbodyy_6));
    message.faccbodyy_6 = message.accbodyy_6 * 0.0004;

    char acceleration_body_z_poi6[] = {local_data[212],local_data[213]};
    memcpy(&message.accbodyz_6 , &acceleration_body_z_poi6, sizeof(message.accbodyz_6));
    message.faccbodyz_6 = message.accbodyz_6 * 0.0004;
}

/// \file
/// \brief  getaccelerationbody poi7 function - adma acc body poi7
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration poi7 body
    char acceleration_body_x_poi7[] = {local_data[216],local_data[217]};
    memcpy(&message.accbodyx_7 , &acceleration_body_x_poi7, sizeof(message.accbodyx_7));
    message.faccbodyx_7 = message.accbodyx_7 * 0.0004;

    char acceleration_body_y_poi7[] = {local_data[218],local_data[219]};
    memcpy(&message.accbodyy_7 , &acceleration_body_y_poi7, sizeof(message.accbodyy_7));
    message.faccbodyy_7 = message.accbodyy_7 * 0.0004;

    char acceleration_body_z_poi7[] = {local_data[220],local_data[221]};
    memcpy(&message.accbodyz_7 , &acceleration_body_z_poi7, sizeof(message.accbodyz_7));
    message.faccbodyz_7 = message.accbodyz_7 * 0.0004;

}

/// \file
/// \brief  getaccelerationbody poi8 function - adma acc body poi8
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationbodypoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration poi8 body
    char acceleration_body_x_poi8[] = {local_data[224],local_data[225]};
    memcpy(&message.accbodyx_8 , &acceleration_body_x_poi8, sizeof(message.accbodyx_8));
    message.faccbodyx_8 = message.accbodyx_8 * 0.0004;

    char acceleration_body_y_poi8[] = {local_data[226],local_data[227]};
    memcpy(&message.accbodyy_8 , &acceleration_body_y_poi8, sizeof(message.accbodyy_8));
    message.faccbodyy_8 = message.accbodyy_8 * 0.0004;

    char acceleration_body_z_poi8[] = {local_data[228],local_data[229]};
    memcpy(&message.accbodyz_8 , &acceleration_body_z_poi8, sizeof(message.accbodyz_8));
    message.faccbodyz_8 = message.accbodyz_8 * 0.0004;
}

/// \file
/// \brief  getaccelerationhor poi1 function - adma acc hor poi1
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
      //! acceleration hor. poi1 body
    char acceleration_hor_x_poi1[] = {local_data[232],local_data[233]};
    memcpy(&message.acchorx_1 , &acceleration_hor_x_poi1, sizeof(message.acchorx_1));
    message.facchorx_1 = message.acchorx_1 * 0.0004;

    char acceleration_hor_y_poi1[] = {local_data[234],local_data[235]};
    memcpy(&message.acchory_1 , &acceleration_hor_y_poi1, sizeof(message.acchory_1));
    message.facchory_1 = message.acchory_1 * 0.0004;

    char acceleration_hor_z_poi1[] = {local_data[236],local_data[237]};
    memcpy(&message.acchorz_1 , &acceleration_hor_z_poi1, sizeof(message.acchorz_1));
    message.facchorz_1 = message.acchorz_1 * 0.0004;
}


/// \file
/// \brief  getaccelerationhor poi2 function - adma acc hor poi2
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi2 body
    char acceleration_hor_x_poi2[] = {local_data[240],local_data[241]};
    memcpy(&message.acchorx_2 , &acceleration_hor_x_poi2, sizeof(message.acchorx_2));
    message.facchorx_2 = message.acchorx_2 * 0.0004;

    char acceleration_hor_y_poi2[] = {local_data[242],local_data[243]};
    memcpy(&message.acchory_2 , &acceleration_hor_y_poi2, sizeof(message.acchory_2));
    message.facchory_2 = message.acchory_2 * 0.0004;

    char acceleration_hor_z_poi2[] = {local_data[244],local_data[245]};
    memcpy(&message.acchorz_2 , &acceleration_hor_z_poi2, sizeof(message.acchorz_2));
    message.facchorz_2 = message.acchorz_2 * 0.0004;
}


/// \file
/// \brief  getaccelerationhor poi3 function - adma acc hor poi3
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi3 body
    char acceleration_hor_x_poi3[] = {local_data[248],local_data[249]};
    memcpy(&message.acchorx_3 , &acceleration_hor_x_poi3, sizeof(message.acchorx_3));
    message.facchorx_3 = message.acchorx_3 * 0.0004;

    char acceleration_hor_y_poi3[] = {local_data[250],local_data[251]};
    memcpy(&message.acchory_3 , &acceleration_hor_y_poi3, sizeof(message.acchory_3));
    message.facchory_3 = message.acchory_3 * 0.0004;

    char acceleration_hor_z_poi3[] = {local_data[252],local_data[253]};
    memcpy(&message.acchorz_3 , &acceleration_hor_z_poi3, sizeof(message.acchorz_3));
    message.facchorz_3 = message.acchorz_3 * 0.0004;
}


/// \file
/// \brief  getaccelerationhor poi4 function - adma acc hor poi4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi4 body
    char acceleration_hor_x_poi4[] = {local_data[256],local_data[257]};
    memcpy(&message.acchorx_4 , &acceleration_hor_x_poi4, sizeof(message.acchorx_4));
    message.facchorx_4 = message.acchorx_4 * 0.0004;

    char acceleration_hor_y_poi4[] = {local_data[258],local_data[259]};
    memcpy(&message.acchory_4 , &acceleration_hor_y_poi4, sizeof(message.acchory_4));
    message.facchory_4 = message.acchory_4 * 0.0004;

    char acceleration_hor_z_poi4[] = {local_data[260],local_data[261]};
    memcpy(&message.acchorz_4 , &acceleration_hor_z_poi4, sizeof(message.acchorz_4));
    message.facchorz_4 = message.acchorz_4 * 0.0004;
}

/// \file
/// \brief  getaccelerationhorpoi5 function - adma acc hor poi5
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi5 body
    char acceleration_hor_x_poi5[] = {local_data[264],local_data[265]};
    memcpy(&message.acchorx_5 , &acceleration_hor_x_poi5, sizeof(message.acchorx_5));
    message.facchorx_5 = message.acchorx_5 * 0.0004;

    char acceleration_hor_y_poi5[] = {local_data[266],local_data[267]};
    memcpy(&message.acchory_5 , &acceleration_hor_y_poi5, sizeof(message.acchory_5));
    message.facchory_5 = message.acchory_5 * 0.0004;

    char acceleration_hor_z_poi5[] = {local_data[268],local_data[269]};
    memcpy(&message.acchorz_5 , &acceleration_hor_z_poi5, sizeof(message.acchorz_5));
    message.facchorz_5 = message.acchorz_5 * 0.0004;
}

/// \file
/// \brief  getaccelerationhorpoi6 function - adma acc hor poi6
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi6 body
    char acceleration_hor_x_poi6[] = {local_data[272],local_data[273]};
    memcpy(&message.acchorx_6 , &acceleration_hor_x_poi6, sizeof(message.acchorx_6));
    message.facchorx_6 = message.acchorx_6 * 0.0004;

    char acceleration_hor_y_poi6[] = {local_data[274],local_data[275]};
    memcpy(&message.acchory_6 , &acceleration_hor_y_poi6, sizeof(message.acchory_6));
    message.facchory_6 = message.acchory_6 * 0.0004;

    char acceleration_hor_z_poi6[] = {local_data[276],local_data[277]};
    memcpy(&message.acchorz_6 , &acceleration_hor_z_poi6, sizeof(message.acchorz_6));
    message.facchorz_6 = message.acchorz_6 * 0.0004;
}

/// \file
/// \brief  getaccelerationhorpoi7 function - adma acc hor poi7
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi7 body
    char acceleration_hor_x_poi7[] = {local_data[280],local_data[281]};
    memcpy(&message.acchorx_7 , &acceleration_hor_x_poi7, sizeof(message.acchorx_7));
    message.facchorx_7 = message.acchorx_7 * 0.0004;

    char acceleration_hor_y_poi7[] = {local_data[282],local_data[283]};
    memcpy(&message.acchory_7 , &acceleration_hor_y_poi7, sizeof(message.acchory_7));
    message.facchory_7 = message.acchory_7 * 0.0004;

    char acceleration_hor_z_poi7[] = {local_data[284],local_data[285]};
    memcpy(&message.acchorz_7 , &acceleration_hor_z_poi7, sizeof(message.acchorz_7));
    message.facchorz_7 = message.acchorz_7 * 0.0004;

}

/// \file
/// \brief  getaccelerationhorpoi8 function - adma acc hor poi8
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getaccelerationhorpoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! acceleration hor. poi8 body
    char acceleration_hor_x_poi8[] = {local_data[288],local_data[289]};
    memcpy(&message.acchorx_8 , &acceleration_hor_x_poi8, sizeof(message.acchorx_8));
    message.facchorx_8 = message.acchorx_8 * 0.0004;

    char acceleration_hor_y_poi8[] = {local_data[290],local_data[291]};
    memcpy(&message.acchory_8 , &acceleration_hor_y_poi8, sizeof(message.acchory_8));
    message.facchory_8 = message.acchory_8 * 0.0004;

    char acceleration_hor_z_poi8[] {local_data[292],local_data[293]};
    memcpy(&message.acchorz_8 , &acceleration_hor_z_poi8, sizeof(message.acchorz_8));
    message.facchorz_8 = message.acchorz_8 * 0.0004;
}

/// \file
/// \brief  getexternalvelocityanalog function - adma ext vel analog
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvelocityanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity analalog
    char ext_vel_an_x[] = {local_data[296],local_data[297]};
    memcpy(&message.extvelanx , &ext_vel_an_x, sizeof(message.extvelanx));
    message.fextvelanx = message.extvelanx * 0.005;

    char ext_vel_an_y[] = {local_data[298],local_data[299]};
    memcpy(&message.extvelany , &ext_vel_an_y, sizeof(message.extvelany));
    message.fextvelany = message.extvelany * 0.005;
}

/// \file
/// \brief  getexternalvecovitydigpulses function - adma ext vel dig pulses
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvecovitydigpulses(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity digital pulses
    char ext_vel_dig_x[] = {local_data[304],local_data[305]};
    memcpy(&message.extveldigx , &ext_vel_dig_x, sizeof(message.extveldigx));
    message.fextveldigx = message.extvelany * 0.005;

    char ext_vel_dig_y[] = {local_data[306],local_data[307]};
    memcpy(&message.extveldigy , &ext_vel_dig_y, sizeof(message.extveldigy));
    message.fextveldigy = message.extveldigy * 0.005;

    char ext_vel_dig_pulses_x[] = {local_data[308],local_data[309]};
    memcpy(&message.extveldigpulsesx , &ext_vel_dig_pulses_x, sizeof(message.extveldigpulsesx));

    char ext_vel_dig_pulses_y[] = {local_data[310],local_data[311]};
    memcpy(&message.extveldigpulsesy , &ext_vel_dig_pulses_y, sizeof(message.extveldigpulsesy));
}

/// \file
/// \brief  getexternalvelocitycorrected function - adma ext vel corrected
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getexternalvelocitycorrected(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! external velocity corrected
    char ext_vel_x_corrected[] = {local_data[312],local_data[313]};
    memcpy(&message.extvelxcorrected , &ext_vel_x_corrected, sizeof(message.extvelxcorrected));
    message.fextvelxcorrected = message.extvelxcorrected * 0.005;

    char ext_vel_y_corrected[] = {local_data[314],local_data[315]};
    memcpy(&message.extvelycorrected , &ext_vel_y_corrected, sizeof(message.extvelycorrected));
    message.fextvelycorrected = message.extvelycorrected * 0.005;
}

/// \file
/// \brief  getbarometerpressure function - adma barometer pressure
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getbarometerpressure(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! barometer pressure
    char ext_baro_pressue[] = {local_data[320],local_data[321],local_data[322],local_data[323]};
    memcpy(&message.extbaropressure , &ext_baro_pressue, sizeof(message.extbaropressure));
    message.fextbaropressure = message.extbaropressure * 0.01;
}

/// \file
/// \brief  getbarometerheight function - adma barometer height
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getbarometerheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! barometer height
    char ext_baro_height[] = {local_data[328],local_data[329],local_data[330],local_data[331]};
    memcpy(&message.extbaroheight , &ext_baro_height, sizeof(message.extbaroheight));
    message.fextbaroheight = message.extbaroheight * 0.01;

    char ext_baro_height_corrected[] = {local_data[332],local_data[333],
                                        local_data[334],local_data[335]};
    memcpy(&message.extbaroheightcorrected , &ext_baro_height_corrected, sizeof(message.extbaroheightcorrected));
    message.fextbaroheightcorrected = message.extbaroheightcorrected * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc.
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuos(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous
    char inv_path_radius[] = {local_data[344],local_data[345]};
    memcpy(&message.invpathradius , &inv_path_radius, sizeof(message.invpathradius));
    message.finvpathradius = message.invpathradius * 0.0001;

    char side_slip_angle[] = {local_data[346],local_data[347]};
    memcpy(&message.sideslipangle , &side_slip_angle, sizeof(message.sideslipangle));
    message.fsideslipangle = message.sideslipangle * 0.01;

    char dist_trav[] = {local_data[348],local_data[349],local_data[350],local_data[351]};
    memcpy(&message.disttrav , &dist_trav, sizeof(message.disttrav));
    message.fdisttrav = message.disttrav * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi1
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 1
    char inv_path_radius_poi1[] = {local_data[352],local_data[353]};
    memcpy(&message.invpathradius_1 , &inv_path_radius_poi1, sizeof(message.invpathradius_1));
    message.finvpathradius_1 = message.invpathradius_1 * 0.0001;

    char side_slip_angle_poi1[] = {local_data[354],local_data[355]};
    memcpy(&message.sideslipangle_1 , &side_slip_angle_poi1, sizeof(message.sideslipangle_1));
    message.fsideslipangle_1 = message.sideslipangle_1 * 0.01;

    char dist_trav_poi1[] = {local_data[356],local_data[357],local_data[358],local_data[359]};
    memcpy(&message.disttrav_1 , &dist_trav_poi1, sizeof(message.disttrav_1));
    message.fdisttrav_1 = message.disttrav_1 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi2
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 2
    char inv_path_radius_poi2[] = {local_data[360],local_data[361]};
    memcpy(&message.invpathradius_2 , &inv_path_radius_poi2, sizeof(message.invpathradius_2));
    message.finvpathradius_2 = message.invpathradius_2 * 0.0001;

    char side_slip_angle_poi2[] = {local_data[362],local_data[363]};
    memcpy(&message.sideslipangle_2 , &side_slip_angle_poi2, sizeof(message.sideslipangle_2));
    message.fsideslipangle_2 = message.sideslipangle_2 * 0.01;

    char dist_trav_poi2[] = {local_data[364],local_data[365],local_data[366],local_data[367]};
    memcpy(&message.disttrav_2 , &dist_trav_poi2, sizeof(message.disttrav_2));
    message.fdisttrav_2 = message.disttrav_2 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi3
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 3
    char inv_path_radius_poi3[] = {local_data[368],local_data[369]};
    memcpy(&message.invpathradius_3 , &inv_path_radius_poi3, sizeof(message.invpathradius_3));
    message.finvpathradius_3 = message.invpathradius_3 * 0.0001;

    char side_slip_angle_poi3[] = {local_data[370],local_data[371]};
    memcpy(&message.sideslipangle_3 , &side_slip_angle_poi3, sizeof(message.sideslipangle_3));
    message.fsideslipangle_3 = message.sideslipangle_3 * 0.01;

    char dist_trav_poi3[] = {local_data[372],local_data[373],local_data[374],local_data[375]};
    memcpy(&message.disttrav_3 , &dist_trav_poi3, sizeof(message.disttrav_3));
    message.fdisttrav_3 = message.disttrav_3 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 4
    char inv_path_radius_poi4[] = {local_data[376],local_data[377]};
    memcpy(&message.invpathradius_4 , &inv_path_radius_poi4, sizeof(message.invpathradius_4));
    message.finvpathradius_4 = message.disttrav * 0.0001;

    char side_slip_angle_poi4[] = {local_data[378],local_data[379]};
    memcpy(&message.sideslipangle_4 , &side_slip_angle_poi4, sizeof(message.sideslipangle_4));
    message.fsideslipangle_4 = message.sideslipangle_4 * 0.01;

    char dist_trav_poi4[] = {local_data[380],local_data[381],local_data[382],local_data[383]};
    memcpy(&message.disttrav_4 , &dist_trav_poi4, sizeof(message.disttrav_4));
    message.fdisttrav_4 = message.disttrav_4 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi5
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 5
    char inv_path_radius_poi5[] = {local_data[384],local_data[385]};
    memcpy(&message.invpathradius_5 , &inv_path_radius_poi5, sizeof(message.invpathradius_5));
    message.finvpathradius_5 = message.invpathradius_5 * 0.0001;

    char side_slip_angle_poi5[] = {local_data[386],local_data[387]};
    memcpy(&message.sideslipangle_5 , &side_slip_angle_poi5, sizeof(message.sideslipangle_5));
    message.fsideslipangle_5 = message.sideslipangle_5 * 0.01;

    char dist_trav_poi5[] = {local_data[388],local_data[389],local_data[390],local_data[391]};
    memcpy(&message.disttrav_5 , &dist_trav_poi5, sizeof(message.disttrav_5));
    message.fdisttrav_5 = message.disttrav_5 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi6
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 6
    char inv_path_radius_poi6[] = {local_data[392],local_data[393]};
    memcpy(&message.invpathradius_6 , &inv_path_radius_poi6, sizeof(message.invpathradius_6));
    message.finvpathradius_6 = message.invpathradius_6 * 0.0001;

    char side_slip_angle_poi6[] = {local_data[394],local_data[395]};
    memcpy(&message.sideslipangle_6 , &side_slip_angle_poi6, sizeof(message.sideslipangle_6));
    message.fsideslipangle_6 = message.sideslipangle_6 * 0.01;

    char dist_trav_poi6[] = {local_data[396],local_data[397],local_data[398],local_data[399]};
    memcpy(&message.disttrav_6 , &dist_trav_poi6, sizeof(message.disttrav_6));
    message.fdisttrav_6 = message.disttrav_6 * 0.01;
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi7
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 7
    char inv_path_radius_poi7[] = {local_data[400],local_data[401]};
    memcpy(&message.invpathradius_7 , &inv_path_radius_poi7, sizeof(message.invpathradius_7));
    message.finvpathradius_7 = message.invpathradius_7 * 0.0001;

    char side_slip_angle_poi7[] = {local_data[402],local_data[403]};
    memcpy(&message.sideslipangle_7 , &side_slip_angle_poi7, sizeof(message.sideslipangle_7));
    message.fsideslipangle_7 = message.sideslipangle_7 * 0.01;

    char dist_trav_poi7[] = {local_data[404],local_data[405],local_data[406],local_data[407]};
    memcpy(&message.disttrav_7 , &dist_trav_poi7, sizeof(message.disttrav_7));
    message.fdisttrav_7 = message.disttrav_7 * 0.01;
    
}

/// \file
/// \brief  getmiscellaneuos function - adma misc. poi8
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getmiscellaneuospoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! miscellaneous poi 8
    char inv_path_radius_poi8[] = {local_data[408],local_data[409]};
    memcpy(&message.invpathradius_8 , &inv_path_radius_poi8, sizeof(message.invpathradius_8));
    message.finvpathradius_8 = message.invpathradius_8 * 0.0001;

    char side_slip_angle_poi8[] = {local_data[410],local_data[411]};
    memcpy(&message.sideslipangle_8 , &side_slip_angle_poi8, sizeof(message.sideslipangle_8));
    message.fsideslipangle_8 = message.sideslipangle_8 * 0.01;

    char dist_trav_poi8[] = {local_data[412],local_data[413],local_data[414],local_data[415]};
    memcpy(&message.disttrav_8 , &dist_trav_poi8, sizeof(message.disttrav_8));
    message.fdisttrav_8 = message.disttrav_8 * 0.01;
}

/// \file
/// \brief  gettriggers function - adma triggers 1,2,3 and 4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void gettriggers(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! triggers 1 and 2
    char trigger_raising_1[] = {local_data[416],local_data[417]};
    memcpy(&message.trigrising1 , &trigger_raising_1, sizeof(message.trigrising1));
    char trigger_falling_1[] = {local_data[418],local_data[419]};
    memcpy(&message.trigfalling1 , &trigger_falling_1, sizeof(message.trigfalling1));
    char trigger_raising_2[] = {local_data[420],local_data[421]};
    memcpy(&message.trigrising2 , &trigger_raising_2, sizeof(message.trigrising2));
    char trigger_falling_2[] = {local_data[422],local_data[423]};
    memcpy(&message.trigfalling2 , &trigger_falling_2, sizeof(message.trigfalling2));
    //! triggers 3 and 4
    char trigger_raising_3[] = {local_data[424],local_data[425]};
    memcpy(&message.trigrising3 , &trigger_raising_3, sizeof(message.trigrising3));
    char trigger_falling_3[] = {local_data[426],local_data[427]};
    memcpy(&message.trigfalling3 , &trigger_falling_3, sizeof(message.trigfalling3));
    char trigger_raising_4[] = {local_data[428],local_data[429]};
    memcpy(&message.trigrising4 , &trigger_raising_4, sizeof(message.trigrising4));
    char trigger_falling_4[] = {local_data[430],local_data[431]};
    memcpy(&message.trigfalling4 , &trigger_falling_4, sizeof(message.trigfalling4));
}

/// \file
/// \brief  getsystemdata function - adma system data
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getsystemdata(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! system data
    char system_ta[] = {local_data[432],local_data[433]};
    memcpy(&message.systemta , &system_ta, sizeof(message.systemta));
    char system_temp[] = {local_data[434],local_data[435]};
    memcpy(&message.systemtemp , &system_temp, sizeof(message.systemtemp));
    message.fsystemtemp = message.systemtemp * 0.1;
    char system_timesinceinit[] = {local_data[436],local_data[437]};
    memcpy(&message.systemtimesinceinit , &system_timesinceinit, sizeof(message.systemtimesinceinit));
    char system_dsp_load[] = {local_data[438],local_data[439]};
    memcpy(&message.systemdspload , &system_dsp_load, sizeof(message.systemdspload));
    message.fsystemdspload = message.systemdspload * 0.1;

}

/// \file
/// \brief  getgpsabs function - adma absolute gps information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsabs(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps position absolute
    char gps_lat_abs[] = {local_data[440],local_data[441],local_data[442],local_data[443]};
    memcpy(&message.gpslatabs , &gps_lat_abs, sizeof(message.gpslatabs));
    message.fgpslatabs = message.gpslatabs * 0.0000001; 
    char gps_lon_abs[] = {local_data[444],local_data[445],local_data[446],local_data[447]};
    memcpy(&message.gpslonabs , &gps_lon_abs, sizeof(message.gpslonabs));
    message.fgpslonabs = message.gpslonabs * 0.0000001;
    
}

/// \file
/// \brief  getgpsposrel function - adma relative gps information
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps position relative
    char gps_lat_rel[] = {local_data[448],local_data[449],local_data[450],local_data[451]};
    memcpy(&message.gpslatrel , &gps_lat_rel, sizeof(message.gpslatrel));
    message.fgpslatrel = message.gpslatrel * 0.01;

    char gps_lon_rel[] = {local_data[452],local_data[453],local_data[454],local_data[455]};
    memcpy(&message.gpslonrel , &gps_lon_rel, sizeof(message.gpslonrel));
    message.fgpslonrel = message.gpslonrel * 0.01;
}

/// \file
/// \brief  getgpsepe function - adma position error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
   //! gps position error
    char gps_stddev_lat[] = {local_data[456],local_data[457]};
    memcpy(&message.gpsstddevlat , &gps_stddev_lat, sizeof(message.gpsstddevlat));
    message.fgpsstddevlat = message.gpsstddevlat * 0.001;
    char gps_stddev_lon[] = {local_data[458],local_data[459]};
    memcpy(&message.gpsstddevlon , &gps_stddev_lon, sizeof(message.gpsstddevlon));
    message.fgpsstddevlon = message.gpsstddevlon * 0.001;
    char gps_stddev_height[] = {local_data[460],local_data[461]};
    memcpy(&message.gpsstddevheight , &gps_stddev_height, sizeof(message.gpsstddevheight));
    message.fgpsstddevheight = message.gpsstddevheight * 0.001;
}

/// \file
/// \brief  getgpsabs function - adma velocity frmae x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsvelframe(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_velocity)
{
    //! gps velocity frame
    char gps_vel_frame_x[] = {local_data[464],local_data[465]};
    memcpy(&message.gpsvelframex , &gps_vel_frame_x, sizeof(message.gpsvelframex));
    message.fgpsvelframex = message.gpsvelframex * 0.005;
    char gps_vel_frame_y[] = {local_data[466],local_data[467]};
    memcpy(&message.gpsvelframey , &gps_vel_frame_y, sizeof(message.gpsvelframey));
    message.fgpsvelframey = message.gpsvelframey * 0.005;
    msg_velocity.data = std::sqrt(message.fgpsvelframex * message.fgpsvelframex + message.fgpsvelframey * message.fgpsvelframey) * 3.6;
    char gps_vel_frame_z[] = {local_data[468],local_data[469]};
    memcpy(&message.gpsvelframez , &gps_vel_frame_z, sizeof(message.gpsvelframez));
    message.fgpsvelframez = message.gpsvelframez * 0.005;
    char gps_vel_latency[] = {local_data[470],local_data[471]};
    memcpy(&message.gpsvellatency , &gps_vel_latency, sizeof(message.gpsvellatency));
    message.fgpsvellatency = message.gpsvellatency * 0.001;    
}

/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsveleve(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps velocity error
    char gps_stddev_vel_x[] = {local_data[472],local_data[473]};
    memcpy(&message.gpsstddevvelx , &gps_stddev_vel_x, sizeof(message.gpsstddevvelx));
    message.fgpsstddevvelx = message.gpsstddevvelx * 0.001;  
    char gps_stddev_vel_y[] = {local_data[474],local_data[475]};
    memcpy(&message.gpsstddevvely , &gps_stddev_vel_y, sizeof(message.gpsstddevvely));
    message.fgpsstddevvely = message.gpsstddevvely * 0.001;  
    char gps_stddev_vel_z[] = {local_data[476],local_data[477]};
    memcpy(&message.gpsstddevvelz , &gps_stddev_vel_z, sizeof(message.gpsstddevvelz));
    message.fgpsstddevvelz = message.gpsstddevvelz * 0.001;  
}

/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps time utc
    char gps_time_msec[] = {local_data[480],local_data[481],local_data[482],local_data[483]};
    memcpy(&message.gpstimemsec , &gps_time_msec, sizeof(message.gpstimemsec));
    char gps_time_weel[] = {local_data[484], local_data[485]};
    memcpy(&message.gpstimeweek , &gps_time_weel, sizeof(message.gpstimeweek));
    char trigger_gps[] = {local_data[486],local_data[487]};
    memcpy(&message.gpstrigger , &trigger_gps, sizeof(message.gpstrigger));
}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsauxdata1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps aux data 1
    char gps_diff_age[] = {local_data[488],local_data[489]};
    memcpy(&message.gpsdiffage , &gps_diff_age, sizeof(message.gpsdiffage));
    message.fgpsdiffage = message.gpsdiffage * 0.1;  
    char gps_stats_used[] = {local_data[490]};
    memcpy(&message.gpsstatsused , &gps_stats_used, sizeof(message.gpsstatsused));
    char gps_stats_visible[] = {local_data[491]};
    memcpy(&message.gpsstatsvisible , &gps_stats_visible, sizeof(message.gpsstatsvisible));

}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsauxdata2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps aux data 2
    char gps_log_delay[] = {local_data[496]};
    memcpy(&message.gpslogdelay , &gps_log_delay, sizeof(message.gpslogdelay));
    char gps_receiver_load[] = {local_data[497]};
    memcpy(&message.gpsreceiverload , &gps_receiver_load, sizeof(message.gpsreceiverload));
    message.fgpsreceiverload = message.gpsreceiverload * 0.5;  
    char gps_basenr[] = {local_data[498]};
}
/// \file
/// \brief  getgpsabs function - adma expected velocity error
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsanglegpscog(const std::string& local_data, adma_msgs::msg::AdmaData& message, std_msgs::msg::Float64& msg_heading)
{
    //! ins angle and gps cog
    char ins_roll[] = {local_data[504],local_data[505]};
    memcpy(&message.insroll , &ins_roll, sizeof(message.insroll));
    message.finsroll = message.insroll * 0.01;  
    char ins_pitch[] = {local_data[506],local_data[507]};
    memcpy(&message.inspitch , &ins_pitch, sizeof(message.inspitch));
    message.finspitch = message.inspitch * 0.01;  
    char ins_yaw[] = {local_data[508],local_data[509]};
    memcpy(&message.insyaw , &ins_yaw, sizeof(message.insyaw));
    message.finsyaw = message.insyaw * 0.01;  
    msg_heading.data = message.finsyaw;
    char gps_cog[] = {local_data[510],local_data[511]};
    memcpy(&message.gpscog , &gps_cog, sizeof(message.gpscog));
    message.fgpscog = message.gpscog * 0.01;  
}

/// \file
/// \brief  getgpsheight function - adma gps height
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps height (msl)
    char gps_height[] = {local_data[512],local_data[513],local_data[514],local_data[515]};
    memcpy(&message.gpsheight , &gps_height, sizeof(message.gpsheight));
    message.gpsheight = message.gpsheight * 0.01;  
    char undulation[] = {local_data[516],local_data[517]};
    memcpy(&message.undulation , &undulation, sizeof(message.undulation));
    message.fundulation = message.undulation * 0.01;  
}

/// \file
/// \brief  getgpsdualanttimeutc function - adma gps dual ant time utc
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualanttimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant time utc
    char gps_dualant_time_msec[] = {local_data[520],local_data[521],local_data[522],local_data[523]};
    memcpy(&message.gpsdualanttimemsec , &gps_dualant_time_msec, sizeof(message.gpsdualanttimemsec));
    char gps_dualant_time_week[] = {local_data[524],local_data[525]};
    memcpy(&message.gpsdualanttimeweek , &gps_dualant_time_week, sizeof(message.gpsdualanttimeweek));
}

/// \file
/// \brief  getgpsdualantangle function - adma gps dual ant angle
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualantangle(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant angle
    char gps_dualant_heading[] = {local_data[528],local_data[529]};
    memcpy(&message.gpsdualantheading , &gps_dualant_heading, sizeof(message.gpsdualantheading));
    message.fgpsdualantheading = message.gpsdualantheading * 0.01;  
    char gps_dualant_pitch[] = {local_data[530],local_data[531]};
    memcpy(&message.gpsdualantpitch , &gps_dualant_pitch, sizeof(message.gpsdualantpitch));
    message.fgpsdualantpitch = message.gpsdualantpitch * 0.01;  
}

/// \file
/// \brief  getgpsdualantangleete function - adma gps dual ant angle ete
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgpsdualantangleete(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps dualant angle ete
    char gps_dualant_stddev_heading[] =  {local_data[536]};
    memcpy(&message.gpsdualantstddevheading , &gps_dualant_stddev_heading, sizeof(message.gpsdualantstddevheading));
    message.fgpsdualantstddevheading = message.gpsdualantstddevheading * 0.01;  
    char gps_dualant_stddev_pitch[] = {local_data[537]};
    memcpy(&message.gpsdualantstddevpitch , &gps_dualant_stddev_pitch, sizeof(message.gpsdualantstddevpitch));
    message.fgpsdualantstddevpitch = message.gpsdualantstddevpitch * 0.01;  
}


/// \file
/// \brief  getgpsdualantangleete function - adma gps dual ant angle ete
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspositionheight(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    char ins_height[] = {local_data[544],local_data[545],local_data[546],local_data[547]};
    memcpy(&message.insheight , &ins_height, sizeof(message.insheight));
    message.finsheight = message.insheight * 0.01;
}

/// \file
/// \brief  getgpsdualantangleete function - adma gps dual ant angle ete
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspositionpoi(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins position height 1 and 2
    char ins_height_poi1[] = {local_data[552],local_data[553],local_data[554],local_data[555]};
    memcpy(&message.insheight_1 , &ins_height_poi1, sizeof(message.insheight_1));
    message.finsheight_1 = message.insheight_1 * 0.01;
    char ins_height_poi2[] = {local_data[556],local_data[557],local_data[558],local_data[559]};
    memcpy(&message.insheight_2 , &ins_height_poi2, sizeof(message.insheight_2));
    message.finsheight_2 = message.insheight_2 * 0.01;
    //! ins position height 3 and 4
    char ins_height_poi3[] = {local_data[560],local_data[561],local_data[562],local_data[563]};
    memcpy(&message.insheight_3 , &ins_height_poi3, sizeof(message.insheight_3));
    message.finsheight_3 = message.insheight_3 * 0.01;
    char ins_height_poi4[] = {local_data[564],local_data[565],local_data[566],local_data[567]};
    memcpy(&message.insheight_4 , &ins_height_poi4, sizeof(message.insheight_4));
    message.finsheight_4 = message.insheight_4 * 0.01;
    //! ins position height 5 and 6
    char ins_height_poi5[] = {local_data[568],local_data[569],local_data[570],local_data[571]};
    memcpy(&message.insheight_5 , &ins_height_poi5, sizeof(message.insheight_5));
    message.finsheight_5 = message.insheight_5 * 0.01;
    char ins_height_poi6[] = {local_data[572],local_data[573],local_data[574],local_data[575]};
    memcpy(&message.insheight_6 , &ins_height_poi6, sizeof(message.insheight_6));
    message.finsheight_6 = message.insheight_6 * 0.01;
    //! ins position height 7 and 8
    char ins_height_poi7[] = {local_data[576],local_data[577],local_data[578],local_data[579]};
    memcpy(&message.insheight_7 , &ins_height_poi7, sizeof(message.insheight_7));
    message.finsheight_7 = message.insheight_7 * 0.01;
    char ins_height_poi8[] = {local_data[580],local_data[581],local_data[582],local_data[583]};
    memcpy(&message.insheight_8 , &ins_height_poi8, sizeof(message.insheight_8));
    message.finsheight_8 = message.insheight_8 * 0.01;
}

/// \file
/// \brief  getinstimeutc function -adma ins time utc
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinstimeutc(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins time utc
    char ins_time_msec[] = {local_data[584],local_data[585],local_data[586],local_data[587]};
    memcpy(&message.instimemsec , &ins_time_msec, sizeof(message.instimemsec));
    char ins_time_week[] = {local_data[588],local_data[589]};
    memcpy(&message.instimeweek , &ins_time_week, sizeof(message.instimeweek));
    char leap_seconds[] = {local_data[590],local_data[591]};
    memcpy(&message.leapseconds , &leap_seconds, sizeof(message.leapseconds));
}

/// \file
/// \brief  getinspositionabs function -  adma ins pos abs
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspositionabs(const std::string& local_data, adma_msgs::msg::AdmaData& message, sensor_msgs::msg::NavSatFix& msg_fix)
{
    //! ins position abs
    char ins_lat_abs[] = {local_data[592],local_data[593],local_data[594],local_data[595]};
    memcpy(&message.inslatabs , &ins_lat_abs, sizeof(message.inslatabs));
    message.finslatabs = message.inslatabs * 0.0000001;
    msg_fix.latitude = message.fgpslatabs;
    char ins_lon_abs[] = {local_data[596],local_data[597],local_data[598],local_data[599]};
    memcpy(&message.inslonabs , &ins_lon_abs, sizeof(message.inslonabs));
    message.finslonabs = message.inslonabs * 0.0000001;
    msg_fix.longitude = message.finslonabs; 
}

/// \file
/// \brief  getinsposrel function -  adma ins pos rel 
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsposrel(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins position rel 
    char ins_lat_rel[] = {local_data[600],local_data[601],local_data[602],local_data[603]};
    memcpy(&message.inslatrel , &ins_lat_rel, sizeof(message.inslatrel));
    message.finslatrel = message.inslatrel * 0.01;
    char ins_lon_rel[] = {local_data[604],local_data[605],local_data[606],local_data[607]};
    memcpy(&message.inslonrel , &ins_lon_rel, sizeof(message.inslonrel));
    message.finslonrel = message.inslonrel * 0.01;
}

/// \file
/// \brief  getinspospoi1 function  adma ins pos abs and rel poi 1
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi1
    char ins_lat_abs_poi1[] = {local_data[608],local_data[609],local_data[610],local_data[611]};
    memcpy(&message.inslatabs_1 , &ins_lat_abs_poi1, sizeof(message.inslatabs_1));
    message.finslatabs_1 = message.inslatabs_1 * 0.0000001;
    char ins_lon_abs_poi1[] = {local_data[612],local_data[613],local_data[614],local_data[615]};
    memcpy(&message.inslonabs_1 , &ins_lon_abs_poi1, sizeof(message.inslonabs_1));
    message.finslonabs_1 = message.inslonabs_1 * 0.0000001;
    //! ins position rel poi1
    char ins_lat_rel_poi1[] = {local_data[616],local_data[617],local_data[618],local_data[619]};
    memcpy(&message.inslatrel_1 , &ins_lat_rel_poi1, sizeof(message.inslatrel_1));
    message.finslatrel_1 = message.inslatrel_1 * 0.01;
    char ins_lon_rel_poi1[] = {local_data[620],local_data[621],local_data[622],local_data[623]};
    memcpy(&message.inslonrel_1 , &ins_lon_rel_poi1, sizeof(message.inslonrel_1));
    message.finslonrel_1 = message.inslonrel_1 * 0.01;
}

/// \file
/// \brief  getinspospoi2 function  adma ins pos abs and rel poi 2
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi2
    char ins_lat_abs_poi2[] = {local_data[624],local_data[625],local_data[626],local_data[627]};
    memcpy(&message.inslatabs_2 , &ins_lat_abs_poi2, sizeof(message.inslatabs_2));
    message.finslatabs_2 = message.inslatabs_2 * 0.0000001;
    char ins_lon_abs_poi2[] = {local_data[628],local_data[629],local_data[630],local_data[631]};
    memcpy(&message.inslonabs_2 , &ins_lon_abs_poi2, sizeof(message.inslonabs_2));
    message.finslonabs_2 = message.inslonabs_2 * 0.0000001;
    //! ins position rel poi2
    char ins_lat_rel_poi2[] = {local_data[632],local_data[633],local_data[634],local_data[635]};
    memcpy(&message.inslatrel_2 , &ins_lat_rel_poi2, sizeof(message.inslatrel_2));
    message.finslatrel_2 = message.inslatrel_2 * 0.01;
    char ins_lon_rel_poi2[] = {local_data[636],local_data[637],local_data[638],local_data[639]};
    memcpy(&message.inslonrel_2 , &ins_lon_rel_poi2, sizeof(message.inslonrel_2));
    message.finslonrel_2 = message.inslonrel_2 * 0.01;
}

/// \file
/// \brief  getinspospoi3 function  adma ins pos abs and rel poi 3
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi3(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi3
    char ins_lat_abs_poi3[] = {local_data[640],local_data[641],local_data[642],local_data[643]};
    memcpy(&message.inslatabs_3 , &ins_lat_abs_poi3, sizeof(message.inslatabs_3));
    message.finslatabs_3 = message.inslatabs_3 * 0.0000001;
    char ins_lon_abs_poi3[] = {local_data[644],local_data[645],local_data[646],local_data[647]};
    memcpy(&message.inslonabs_3 , &ins_lon_abs_poi3, sizeof(message.inslonabs_3));
    message.finslonabs_3 = message.inslonabs_3 * 0.0000001;
    //! ins position rel poi3
    char ins_lat_rel_poi3[] = {local_data[648],local_data[649],local_data[650],local_data[651]};
    memcpy(&message.inslatrel_3 , &ins_lat_rel_poi3, sizeof(message.inslatrel_3));
    message.finslatrel_3 = message.inslatrel_3 * 0.01;
    char ins_lon_rel_poi3[] = {local_data[652],local_data[653],local_data[654],local_data[655]};
    memcpy(&message.inslonrel_3 , &ins_lon_rel_poi3, sizeof(message.inslonrel_3));
    message.finslonrel_3 = message.inslonrel_3 * 0.01;
}

/// \file
/// \brief  getinspospoi4 function  adma ins pos abs and rel poi 4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi4(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi4
    char ins_lat_abs_poi4[] = {local_data[656],local_data[657],local_data[658],local_data[659]};
    memcpy(&message.inslatabs_4 , &ins_lat_abs_poi4, sizeof(message.inslatabs_4));
    message.finslatabs_4 = message.inslatabs_4 * 0.0000001;
    char ins_lon_abs_poi4[] = {local_data[660],local_data[661],local_data[662],local_data[663]};
    memcpy(&message.inslonabs_4 , &ins_lon_abs_poi4, sizeof(message.inslonabs_4));
    message.finslonabs_4 = message.inslonabs_4 * 0.0000001;
    //! ins position rel poi4
    char ins_lat_rel_poi4[] = {local_data[664],local_data[665],local_data[666],local_data[667]};
    memcpy(&message.inslatrel_4 , &ins_lat_rel_poi4, sizeof(message.inslatrel_4));
    message.finslatrel_4 = message.inslatrel_4 * 0.01;
    char ins_lon_rel_poi4[] = {local_data[668],local_data[669],local_data[670],local_data[671]};
    memcpy(&message.inslonrel_4 , &ins_lon_rel_poi4, sizeof(message.inslonrel_4));
    message.finslonrel_4 = message.inslonrel_4 * 0.01;
}

/// \file
/// \brief  getinspospoi5 function  adma ins pos abs and rel poi 5
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi5(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi5
    char ins_lat_abs_poi5[] = {local_data[672],local_data[673],local_data[674],local_data[675]};
    memcpy(&message.inslatabs_5 , &ins_lat_abs_poi5, sizeof(message.inslatabs_5));
    message.finslatabs_5 = message.inslatabs_5 * 0.0000001;
    char ins_lon_abs_poi5[] = {local_data[676],local_data[677],local_data[678],local_data[679]};
    memcpy(&message.inslonabs_5 , &ins_lon_abs_poi5, sizeof(message.inslonabs_5));
    message.finslonabs_5 = message.inslonabs_5 * 0.0000001;
    //! ins position rel poi5
    char ins_lat_rel_poi5[] = {local_data[680],local_data[681],local_data[682],local_data[683]};
    memcpy(&message.inslatrel_5 , &ins_lat_rel_poi5, sizeof(message.inslatrel_5));
    message.finslatrel_5 = message.inslatrel_5 * 0.01;
    char ins_lon_rel_poi5[] = {local_data[684],local_data[685],local_data[686],local_data[687]};
    memcpy(&message.inslonrel_5 , &ins_lon_rel_poi5, sizeof(message.inslonrel_5));
    message.finslonrel_5 = message.inslonrel_5 * 0.01;
}

/// \file
/// \brief  getinspospoi6 function  adma ins pos abs and rel poi 6
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi6(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi6
    char ins_lat_abs_poi6[] = {local_data[688],local_data[689],local_data[690],local_data[691]};
    memcpy(&message.inslatabs_6 , &ins_lat_abs_poi6, sizeof(message.inslatabs_6));
    message.finslatabs_6 = message.inslatabs_6 * 0.0000001;
    char ins_lon_abs_poi6[] = {local_data[692],local_data[693],local_data[694],local_data[695]};
    memcpy(&message.inslonabs_6 , &ins_lon_abs_poi6, sizeof(message.inslonabs_6));
    message.finslonabs_6 = message.inslonabs_6 * 0.0000001;
    //! ins position rel poi6
    char ins_lat_rel_poi6[] = {local_data[696],local_data[697],local_data[698],local_data[699]};
    memcpy(&message.inslatrel_6 , &ins_lat_rel_poi6, sizeof(message.inslatrel_6));
    message.finslatrel_6 = message.inslatrel_6 * 0.01;
    char ins_lon_rel_poi6[] = {local_data[700],local_data[701],local_data[702],local_data[703]};
    memcpy(&message.inslonrel_6 , &ins_lon_rel_poi6, sizeof(message.inslonrel_6));
    message.finslonrel_6 = message.inslonrel_6 * 0.01;
}

/// \file
/// \brief  getinspospoi7 function  adma ins pos abs and rel poi 7
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi7(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi7
    char ins_lat_abs_poi7[] = {local_data[704],local_data[705],local_data[706],local_data[707]};
    memcpy(&message.inslatabs_7 , &ins_lat_abs_poi7, sizeof(message.inslatabs_7));
    message.finslatabs_7 = message.inslatabs_7 * 0.0000001;
    char ins_lon_abs_poi7[] = {local_data[708],local_data[709],local_data[710],local_data[711]};
    memcpy(&message.inslonabs_7 , &ins_lon_abs_poi7, sizeof(message.inslonabs_7));
    message.finslonabs_7 = message.inslonabs_7 * 0.0000001;
    //! ins position rel poi7
    char ins_lat_rel_poi7[] = {local_data[712],local_data[713],local_data[714],local_data[715]};
    memcpy(&message.inslatrel_7 , &ins_lat_rel_poi7, sizeof(message.inslatrel_7));
    message.finslatrel_7 = message.inslatrel_7 * 0.01;
    char ins_lon_rel_poi7[] = {local_data[716],local_data[717],local_data[718],local_data[719]};
    memcpy(&message.inslonrel_7 , &ins_lon_rel_poi7, sizeof(message.inslonrel_7));
    message.finslonrel_7 = message.inslonrel_7 * 0.01;
}

/// \file
/// \brief  getinspospoi8 function - adma ins pos abs and rel poi 8
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinspospoi8(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins postion abs poi8
    char ins_lat_abs_poi8[] = {local_data[720],local_data[721],local_data[722],local_data[723]};
    memcpy(&message.inslatabs_8 , &ins_lat_abs_poi8, sizeof(message.inslatabs_8));
    message.finslatabs_8 = message.inslatabs_8 * 0.0000001;
    char ins_lon_abs_poi8[] = {local_data[724],local_data[725],local_data[726],local_data[727]};
    memcpy(&message.inslonabs_8 , &ins_lon_abs_poi8, sizeof(message.inslonabs_8));
    message.finslonabs_8 = message.inslonabs_8 * 0.0000001;
    //! ins position rel poi8
    char ins_lat_rel_poi8[] = {local_data[728],local_data[729],local_data[730],local_data[731]};
    memcpy(&message.inslatrel_8 , &ins_lat_rel_poi8, sizeof(message.inslatrel_8));
    message.finslatrel_8 = message.inslatrel_8 * 0.01;
    char ins_lon_rel_poi8[] = {local_data[732],local_data[733],local_data[734],local_data[735]};
    memcpy(&message.inslonrel_8 , &ins_lon_rel_poi8, sizeof(message.inslonrel_8));
    message.finslonrel_8 = message.inslonrel_8 * 0.01;
}

/// \file
/// \brief  getinsvelhorxyz function - adma ins vel hor x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal
    char ins_vel_hor_x[] = {local_data[736],local_data[737]};
    memcpy(&message.insvelhorx , &ins_vel_hor_x, sizeof(message.insvelhorx));
    message.finsvelhorx = message.insvelhorx * 0.005;
    char ins_vel_hor_y[] = {local_data[738],local_data[739]};
    memcpy(&message.insvelhory , &ins_vel_hor_y, sizeof(message.insvelhory));
    message.finsvelhory = message.insvelhory * 0.005;
    char ins_vel_hor_z[] = {local_data[740],local_data[741]};
    memcpy(&message.insvelhorz , &ins_vel_hor_z, sizeof(message.insvelhorz));
    message.finsvelhorz = message.insvelhorz * 0.005;  
} 

/// \file
/// \brief  getinsvelframexyz function - adma ins vel frame x y z
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelframexyz(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity frame
    char ins_vel_frame_x[] = {local_data[744],local_data[745]};
    memcpy(&message.insvelframex , &ins_vel_frame_x, sizeof(message.insvelframex));
    message.finsvelframex = message.insvelframex * 0.005;
    char ins_vel_frame_y[] = {local_data[746],local_data[747]};
    memcpy(&message.insvelframex , &ins_vel_frame_y, sizeof(message.insvelframex));
    message.finsvelframex = message.insvelframex * 0.005;
    char ins_vel_frame_z[] = {local_data[748],local_data[749]};
    memcpy(&message.insvelframex , &ins_vel_frame_z, sizeof(message.insvelframex));
    message.finsvelframex = message.insvelframex * 0.005;
}

/// \file
/// \brief  getinsvelhorxyzpos1 function - adma ins vel hor x y z pos1
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos1(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi1
    char ins_vel_hor_x_poi1[] = {local_data[752],local_data[753]};
    memcpy(&message.insvelhorx_1 , &ins_vel_hor_x_poi1, sizeof(message.insvelhorx_1));
    message.finsvelhorx_1 = message.insvelhorx_1 * 0.005;
    char ins_vel_hor_y_poi1[] = {local_data[754],local_data[755]};
    memcpy(&message.insvelhory_1 , &ins_vel_hor_y_poi1, sizeof(message.insvelhory_1));
    message.finsvelhory_1 = message.insvelhory_1 * 0.005;
    char ins_vel_hor_z_poi1[] = {local_data[756],local_data[757]};
    memcpy(&message.insvelhorz_1 , &ins_vel_hor_z_poi1, sizeof(message.insvelhorz_1));
    message.finsvelhorz_1 = message.insvelhorz_1 * 0.005;
}

/// \file
/// \brief  getinsvelhorxyzpos2 function - adma ins vel hor x y z pos2
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos2(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi2
    char ins_vel_hor_x_poi2[] = {local_data[760],local_data[761]};
    memcpy(&message.insvelhorx_2 , &ins_vel_hor_x_poi2, sizeof(message.insvelhorx_2));
    message.finsvelhorx_2 = message.insvelhorx_2 * 0.005;
    char ins_vel_hor_y_poi2[] = {local_data[762],local_data[763]};
    memcpy(&message.insvelhory_2 , &ins_vel_hor_y_poi2, sizeof(message.insvelhory_2));
    message.finsvelhory_2 = message.insvelhory_2 * 0.005;
    char ins_vel_hor_z_poi2[] = {local_data[764],local_data[765]};
    memcpy(&message.insvelhorz_2 , &ins_vel_hor_z_poi2, sizeof(message.insvelhorz_2));
    message.finsvelhorz_2 = message.insvelhorz_2 * 0.005;
}

/// \file
/// \brief  getinsvelhorxyzpos3 function - adma ins vel hor x y z pos3
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos3(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi3
    char ins_vel_hor_x_poi3[] = {local_data[768],local_data[769]};
    memcpy(&message.insvelhorx_3 , &ins_vel_hor_x_poi3, sizeof(message.insvelhorx_3));
    message.finsvelhorx_3 = message.insvelhorx_3 * 0.005;
    char ins_vel_hor_y_poi3[] = {local_data[770],local_data[771]};
    memcpy(&message.insvelhory_3 , &ins_vel_hor_y_poi3, sizeof(message.insvelhory_3));
    message.finsvelhory_3 = message.insvelhory_3 * 0.005;
    char ins_vel_hor_z_poi3[] = {local_data[772],local_data[773]};
    memcpy(&message.insvelhorz_3 , &ins_vel_hor_z_poi3, sizeof(message.insvelhorz_3));
    message.finsvelhorz_3 = message.insvelhorz_3 * 0.005;
}

/// \file
/// \brief  getinsvelhorxyzpos4 function - adma ins vel hor x y z pos4
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos4(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi4
    char ins_vel_hor_x_poi4[] = {local_data[776],local_data[777]};
    memcpy(&message.insvelhorx_4 , &ins_vel_hor_x_poi4, sizeof(message.insvelhorx_4));
    message.finsvelhorx_4 = message.insvelhorx_4 * 0.005;
    char ins_vel_hor_y_poi4[] = {local_data[778],local_data[779]};
    memcpy(&message.insvelhory_4 , &ins_vel_hor_y_poi4, sizeof(message.insvelhory_4));
    message.finsvelhory_4 = message.insvelhory_4 * 0.005;
    char ins_vel_hor_z_poi4[] = {local_data[780],local_data[781]};
    memcpy(&message.insvelhorz_4 , &ins_vel_hor_z_poi4, sizeof(message.insvelhorz_4));
    message.finsvelhorz_4 = message.insvelhorz_4 * 0.005;
}


/// \file
/// \brief  getinsvelhorxyzpos5 function - adma ins vel hor x y z pos5
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos5(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi5
    char ins_vel_hor_x_poi5[] = {local_data[784],local_data[785]};
    memcpy(&message.insvelhorx_5 , &ins_vel_hor_x_poi5, sizeof(message.insvelhorx_5));
    message.finsvelhorx_5 = message.insvelhorx_5 * 0.005;
    char ins_vel_hor_y_poi5[] = {local_data[786],local_data[787]};
    memcpy(&message.insvelhory_5 , &ins_vel_hor_y_poi5, sizeof(message.insvelhory_5));
    message.finsvelhory_5 = message.insvelhory_5 * 0.005;
    char ins_vel_hor_z_poi5[] = {local_data[788],local_data[789]};
    memcpy(&message.insvelhorz_5 , &ins_vel_hor_z_poi5, sizeof(message.insvelhorz_5));
    message.finsvelhorz_5 = message.insvelhorz_5 * 0.005;
}

/// \file
/// \brief  getinsvelhorxyzpos6 function - adma ins vel hor x y z pos6
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos6(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi6
    char ins_vel_hor_x_poi6[] = {local_data[792],local_data[793]};
    memcpy(&message.insvelhorx_6 , &ins_vel_hor_x_poi6, sizeof(message.insvelhorx_6));
    message.finsvelhorx_6 = message.insvelhorx_6 * 0.005;
    char ins_vel_hor_y_poi6[] = {local_data[794],local_data[795]};
    memcpy(&message.insvelhory_6 , &ins_vel_hor_y_poi6, sizeof(message.insvelhory_6));
    message.finsvelhory_6 = message.insvelhory_6 * 0.005;
    char ins_vel_hor_z_poi6[] = {local_data[796],local_data[797]};
    memcpy(&message.insvelhorz_6 , &ins_vel_hor_z_poi6, sizeof(message.insvelhorz_6));
    message.finsvelhorz_6 = message.insvelhorz_6 * 0.005;
}


/// \file
/// \brief  getinsvelhorxyzpos7 function - adma ins vel hor x y z pos7
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos7(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi7
    char ins_vel_hor_x_poi7[] = {local_data[800],local_data[801]};
    memcpy(&message.insvelhorx_7 , &ins_vel_hor_x_poi7, sizeof(message.insvelhorx_7));
    message.finsvelhorx_7 = message.insvelhorx_7 * 0.005;
    char ins_vel_hor_y_poi7[] = {local_data[802],local_data[803]};
    memcpy(&message.insvelhory_7 , &ins_vel_hor_y_poi7, sizeof(message.insvelhory_7));
    message.finsvelhory_7 = message.insvelhory_7 * 0.005;
    char ins_vel_hor_z_poi7[] = {local_data[804],local_data[805]};
    memcpy(&message.insvelhorz_7 , &ins_vel_hor_z_poi7, sizeof(message.insvelhorz_7));
    message.finsvelhorz_7 = message.insvelhorz_7 * 0.005;
}


/// \file
/// \brief  getinsvelhorxyzpos8 function - adma ins vel hor x y z pos8
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsvelhorxyzpos8(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins velocity horizontal poi8
    char ins_vel_hor_x_poi8[] = {local_data[808],local_data[809]};
    memcpy(&message.insvelhorx_8 , &ins_vel_hor_x_poi8, sizeof(message.insvelhorx_8));
    message.finsvelhorx_8 = message.insvelhorx_8 * 0.005;
    char ins_vel_hor_y_poi8[] = {local_data[810],local_data[811]};
    memcpy(&message.insvelhory_8 , &ins_vel_hor_y_poi8, sizeof(message.insvelhory_8));
    message.finsvelhory_8 = message.insvelhory_8 * 0.005;
    char ins_vel_hor_z_poi8[] = {local_data[812],local_data[813]};
    memcpy(&message.insvelhorz_8 , &ins_vel_hor_z_poi8, sizeof(message.insvelhorz_8));
    message.finsvelhorz_8 = message.insvelhorz_8 * 0.005;
}

/// \file
/// \brief  getinsepe function - adma ins sepe
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getinsepe(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins epe
    char ins_stddev_lat[] = {local_data[816],local_data[817]};
    memcpy(&message.insstddevlat , &ins_stddev_lat, sizeof(message.insstddevlat));
    message.finsstddevlat = message.insstddevlat * 0.01;
    char ins_stddev_lon[] = {local_data[818],local_data[819]};
    memcpy(&message.insstddevlong , &ins_stddev_lon, sizeof(message.insstddevlong));
    message.finsstddevlong = message.insstddevlong * 0.01;
    char ins_stddev_height[] = {local_data[820],local_data[821]};
    memcpy(&message.insstddevheight , &ins_stddev_height, sizeof(message.insstddevheight));
    message.finsstddevheight = message.insstddevheight * 0.01;
}

/// \file
/// \brief  getinseveandete function - adma ins eve and ete
/// \param  message adma message to be loaded
void getinseveandete(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! ins eve and ins ete
    char ins_stddev_vel_x[] = {local_data[824]};
    memcpy(&message.insstddevvelx , &ins_stddev_vel_x, sizeof(message.insstddevvelx));
    message.finsstddevvelx = message.insstddevvelx * 0.01;  
    char ins_stddev_vel_y[] = {local_data[825]};
    memcpy(&message.insstddevvely , &ins_stddev_vel_y, sizeof(message.insstddevvely));
    message.finsstddevvely = message.insstddevvely * 0.01;
    char ins_stddev_vel_z[] = {local_data[826]};
    memcpy(&message.insstddevvelz , &ins_stddev_vel_z, sizeof(message.insstddevvelz));
    message.finsstddevvelz = message.insstddevvelz * 0.01;
    char ins_stddev_roll[] = {local_data[827]};
    memcpy(&message.insstddevroll , &ins_stddev_roll, sizeof(message.insstddevroll));
    message.finsstddevroll = message.insstddevroll * 0.01;
    char ins_stddev_pitch[] = {local_data[828]};
    memcpy(&message.insstddevpitch , &ins_stddev_pitch, sizeof(message.insstddevpitch));
    message.finsstddevpitch = message.insstddevpitch * 0.01;
    char ins_stddev_yaw[] = {local_data[829]};
    memcpy(&message.insstddevyaw , &ins_stddev_yaw, sizeof(message.insstddevyaw));
    message.finsstddevyaw = message.insstddevyaw * 0.01;
}

/// \file
/// \brief  getanalog function - adma analog
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getanalog(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! analog in 1
    char an1[] = {local_data[832],local_data[833]};
    memcpy(&message.an1 , &an1, sizeof(message.an1));
    message.fan1 = message.an1 * 0.0005;
    char an2[] = {local_data[834],local_data[835]};
    memcpy(&message.an2 , &an2, sizeof(message.an2));
    message.fan2 = message.an2 * 0.0005;
    char an3[] = {local_data[836],local_data[837]};
    memcpy(&message.an3 , &an3, sizeof(message.an3));
    message.fan3 = message.an3 * 0.0005;
    char an4[] = {local_data[838],local_data[839]};
    memcpy(&message.an4 , &an4, sizeof(message.an4));
    message.fan4 = message.an4 * 0.0005;
}

/// \file
/// \brief  getkalmanfilter function - adma kalman filter
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getkalmanfilter(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! kalman filer status
    char kf_lat_stimulated[] = {local_data[840]};
    memcpy(&message.kflatstimulated , &kf_lat_stimulated, sizeof(message.kflatstimulated));
    char kf_lon_stimulated[] = {local_data[841]};
    memcpy(&message.kflongstimulated , &kf_lon_stimulated, sizeof(message.kflongstimulated));
    char kf_steady_state[] = {local_data[842]};
    memcpy(&message.kfsteadystate , &kf_steady_state, sizeof(message.kfsteadystate));
}

/// \file
/// \brief  getgnssreceiver function - adma gps error status
/// \param  local_data adma string
/// \param  message adma message to be loaded
void getgnssreceiver(const std::string& local_data, adma_msgs::msg::AdmaData& message)
{
    //! gps receiver error
    char gps_receiver_error[] = {local_data[848],local_data[849],local_data[850],local_data[851]};
    memcpy(&message.gpsreceivererror , &gps_receiver_error, sizeof(message.gpsreceivererror));
    char gps_receiver_status[] = {local_data[852],local_data[853],local_data[854],local_data[855]};
    memcpy(&message.gpsreceiverstatus , &gps_receiver_status, sizeof(message.gpsreceiverstatus));
}

/// \file
/// \brief  bit shift function
/// \param  byte byte information
/// \param  position message
/// \return an integer 0 upon exit success
bool getbit(unsigned char byte, int position) // position in range 0-7
{
    return (byte >> position) & 0x1;
}

