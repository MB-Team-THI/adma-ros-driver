Header header

# ADMA Static Header
string GeneSysID
string HeaderVersion
uint32 FormatID
string FormatVersion
uint32 SerialNo
uint32 Alias

# ADMA Dynamic Header
uint32 ConfigID
uint32 ConfigFormat
uint32 ConfigVersion
uint32 ConfigSize
uint32 ByteOffset
uint32 SliceSize
uint32 SliceData

# Status
int32 StatusGPSMode
int32 StatusStandStill
int32 StatusSkidding
int32 StatusExternalVelOut
int32 StatusTrigGps
int32 StatusSignalIN3
int32 StatusSignalIn2
int32 StatusSignalIn1
int32 StatusAlignment
int32 StatusAHRSINS
int32 StatusDeadReckoning
int32 StatusSyncLock
int32 StatusEVKActiv
int32 StatusEVKEstimates
int32 StatusHeadingExecuted
int32 StatusConfigurationChanged
int32 StatusTilt
int32 StatusPos
int8 StatusCount
int32 StatusKalmanFilterSetteled
int32 StatusKFLatStimulated
int32 StatusKFLongStimulated
int32 StatusKFSteadyState
int32 StatusSpeed

# Error and Warning
string ErrorHardware
string Error_Misc1
string Error_Misc2
string Error_Misc3
string WarnGPS
string WarnMisc1
string ErrorHWSticky

# Sensors body X
# Acc in g
int32 AccBodyHRX
float32 fAccBodyHRX
# Rate in deg/s
int32 RateBodyHRX
float32 fRateBodyHRX

# Sensors body Y
# Acc in g
int32 AccBodyHRY
float32 fAccBodyHRY

# Rate in deg/s
int32 RateBodyHRY
float32 fRateBodyHRY


# Sensors body Z
# Acc in g
int32 AccBodyHRZ
float32 fAccBodyHRZ

# Rate in deg/s
int32 RateBodyHRZ
float32 fRateBodyHRZ


# Rates body
# Rates in deg/s
int16 RateBodyX
int16 RateBodyY
int16 RateBodyZ
float32 fRateBodyX
float32 fRateBodyY
float32 fRateBodyZ


# Rates Horz
# Rates in deg/s
int16 RateHorX
int16 RateHorY
int16 RateHorZ
float32 fRateHorX
float32 fRateHorY
float32 fRateHorZ

# Accelerations Body
# Acc in g
int16 AccBodyX
int16 AccBodyY
int16 AccBodyZ
float32 fAccBodyX
float32 fAccBodyY
float32 fAccBodyZ

# Acceleration Horizontal
# Acc in g
int16 AccHorX
int16 AccHorY
int16 AccHorZ
float32 fAccHorX
float32 fAccHorY
float32 fAccHorZ

# Accelerations Body POI1
# Acc in g
int16 AccBodyX_1
int16 AccBodyY_1
int16 AccBodyZ_1
float32 fAccBodyX_1
float32 fAccBodyY_1
float32 fAccBodyZ_1

# Accelerations Body POI2
# Acc in g
int16 AccBodyX_2
int16 AccBodyY_2
int16 AccBodyZ_2
float32 fAccBodyX_2
float32 fAccBodyY_2
float32 fAccBodyZ_2

# Accelerations Body POI3
# Acc in g
int16 AccBodyX_3
int16 AccBodyY_3
int16 AccBodyZ_3
float32 fAccBodyX_3
float32 fAccBodyY_3
float32 fAccBodyZ_3

# Accelerations Body POI4
# Acc in g
int16 AccBodyX_4
int16 AccBodyY_4
int16 AccBodyZ_4
float32 fAccBodyX_4
float32 fAccBodyY_4
float32 fAccBodyZ_4

# Accelerations Body POI5
# Acc in g
int16 AccBodyX_5
int16 AccBodyY_5
int16 AccBodyZ_5
float32 fAccBodyX_5
float32 fAccBodyY_5
float32 fAccBodyZ_5

# Accelerations Body POI6
# Acc in g
int16 AccBodyX_6
int16 AccBodyY_6
int16 AccBodyZ_6
float32 fAccBodyX_6
float32 fAccBodyY_6
float32 fAccBodyZ_6

# Accelerations Body POI7
# Acc in g
int16 AccBodyX_7
int16 AccBodyY_7
int16 AccBodyZ_7
float32 fAccBodyX_7
float32 fAccBodyY_7
float32 fAccBodyZ_7

# Accelerations Body POI8
# Acc in g
int16 AccBodyX_8
int16 AccBodyY_8
int16 AccBodyZ_8
float32 fAccBodyX_8
float32 fAccBodyY_8
float32 fAccBodyZ_8

# Acceleration Horizontal POI1
# Acc in g
int16 AccHorX_1
int16 AccHorY_1
int16 AccHorZ_1
float32 fAccHorX_1
float32 fAccHorY_1
float32 fAccHorZ_1

# Acceleration Horizontal POI2
# Acc in g
int16 AccHorX_2
int16 AccHorY_2
int16 AccHorZ_2
float32 fAccHorX_2
float32 fAccHorY_2
float32 fAccHorZ_2

# Acceleration Horizontal POI3
# Acc in g
int16 AccHorX_3
int16 AccHorY_3
int16 AccHorZ_3
float32 fAccHorX_3
float32 fAccHorY_3
float32 fAccHorZ_3

# Acceleration Horizontal POI4
# Acc in g
int16 AccHorX_4
int16 AccHorY_4
int16 AccHorZ_4
float32 fAccHorX_4
float32 fAccHorY_4
float32 fAccHorZ_4

# Acceleration Horizontal POI5
# Acc in g
int16 AccHorX_5
int16 AccHorY_5
int16 AccHorZ_5
float32 fAccHorX_5
float32 fAccHorY_5
float32 fAccHorZ_5

# Acceleration Horizontal POI6
# Acc in g
int16 AccHorX_6
int16 AccHorY_6
int16 AccHorZ_6
float32 fAccHorX_6
float32 fAccHorY_6
float32 fAccHorZ_6

# Acceleration Horizontal POI7
# Acc in g
int16 AccHorX_7
int16 AccHorY_7
int16 AccHorZ_7
float32 fAccHorX_7
float32 fAccHorY_7
float32 fAccHorZ_7

# Acceleration Horizontal POI8
# Acc in g
int16 AccHorX_8
int16 AccHorY_8
int16 AccHorZ_8
float32 fAccHorX_8
float32 fAccHorY_8
float32 fAccHorZ_8

# External Velocity Analog
#  Vel in m/s
int16 ExtVelAnX
int16 ExtVelAnY
float32 fExtVelAnX
float32 fExtVelAnY

# External Velocity Digital Pulses
#  Vel in m/s
int16 ExtVelDigX
int16 ExtVelDigY
int16 ExtVelDigPulsesX
int16 ExtVelDigPulsesY
float32 fExtVelDigX
float32 fExtVelDigY


# Externaal Velocity Corrected
#  Vel in m/s
int16 ExtVelXCorrected
int16 ExtVelYCorrected
float32 fExtVelXCorrected
float32 fExtVelYCorrected

# Barometer Pressure
# Pressure in mBar
uint32 ExtBaroPressure
float32 fExtBaroPressure

# Barometer Height
# Height in m
int32 ExtBaroHeight
int32 ExtBaroHeightCorrected
float32 fExtBaroHeight
float32 fExtBaroHeightCorrected

# Miscellaneous
# InvPath Radius in 1/m
int16 InvPathRadius
float32 fInvPathRadius
# Side Slip in degrees
int16 SideSlipAngle
float32 fSideSlipAngle
# Dis_trav in m
uint32 DistTrav
float32 fDistTrav

# Miscellaneous_POI1
# InvPath Radius in 1/m
int16 InvPathRadius_1
float32 fInvPathRadius_1
# Side Slip in degrees
int16 SideSlipAngle_1
float32 fSideSlipAngle_1
# Dis_trav in m
uint32 DistTrav_1
float32 fDistTrav_1

# Miscellaneous_POI2
# InvPath Radius in 1/m
int16 InvPathRadius_2
float32 fInvPathRadius_2
# Side Slip in degrees
int16 SideSlipAngle_2
float32 fSideSlipAngle_2
# Dis_trav in m
uint32 DistTrav_2
float32 fDistTrav_2

# Miscellaneous_POI3
# InvPath Radius in 1/m
int16 InvPathRadius_3
float32 fInvPathRadius_3
# Side Slip in degrees
int16 SideSlipAngle_3
float32 fSideSlipAngle_3
# Dis_trav in m
uint32 DistTrav_3
float32 fDistTrav_3

# Miscellaneous_POI4
# InvPath Radius in 1/m
int16 InvPathRadius_4
float32 fInvPathRadius_4
# Side Slip in degrees
int16 SideSlipAngle_4
float32 fSideSlipAngle_4
# Dis_trav in m
uint32 DistTrav_4
float32 fDistTrav_4

# Miscellaneous_POI5
# InvPath Radius in 1/m
int16 InvPathRadius_5
float32 fInvPathRadius_5
# Side Slip in degrees
int16 SideSlipAngle_5
float32 fSideSlipAngle_5
# Dis_trav in m
uint32 DistTrav_5
float32 fDistTrav_5

# Miscellaneous_POI6
# InvPath Radius in 1/m
int16 InvPathRadius_6
float32 fInvPathRadius_6
# Side Slip in degrees
int16 SideSlipAngle_6
float32 fSideSlipAngle_6
# Dis_trav in m
uint32 DistTrav_6
float32 fDistTrav_6

# Miscellaneous_POI7
# InvPath Radius in 1/m
int16 InvPathRadius_7
float32 fInvPathRadius_7
# Side Slip in degrees
int16 SideSlipAngle_7
float32 fSideSlipAngle_7
# Dis_trav in m
uint32 DistTrav_7
float32 fDistTrav_7

# Miscellaneous_POI8
# InvPath Radius in 1/m
int16 InvPathRadius_8
float32 fInvPathRadius_8
# Side Slip in degrees
int16 SideSlipAngle_8
float32 fSideSlipAngle_8
# Dis_trav in m
uint32 DistTrav_8
float32 fDistTrav_8


# Triggers 1 and 2
# Triggers in mu.s
uint16 TrigRising1
uint16 TrigFalling1
uint16 TrigRising2
uint16 TrigFalling2


# Triggers 3 and 4
# Triggers in mu.s
uint16 TrigRising3
uint16 TrigFalling3
uint16 TrigRising4
uint16 TrigFalling4

# System Data
# SystemTa in mu.s
uint16 SystemTa
# SystemTemp in deg. C
int16 SystemTemp
float32 fSystemTemp
# SystemTimeSinceInit in s
uint16 SystemTimeSinceInit
# System DSP load in %
uint16 SystemDSPLoad
float32 fSystemDSPLoad


# GPS Position Absolute
# Position in deg
int32 GPSLatAbs
int32 GPSLonAbs
float64 fGPSLatAbs
float64 fGPSLonAbs

# GPS Position Relative
# Position in m
int32 GPSLatRel
int32 GPSLonRel
float32 fGPSLatRel
float32 fGPSLonRel

# GPS EPE
#  Error in m
uint16 GPSStddevLat
uint16 GPSStddevLon
uint16 GPSStddevHeight
float32 fGPSStddevLat
float32 fGPSStddevLon
float32 fGPSStddevHeight


# GPS Velocity Frame
# Vel in m/s
int16 GPSVelFrameX
int16 GPSVelFrameY
int16 GPSVelFrameZ
float32 fGPSVelFrameX
float32 fGPSVelFrameY
float32 fGPSVelFrameZ
# Latency in s
uint16 GPSVelLatency
float32 fGPSVelLatency


# GPS EVE
# Error in m/s
uint16 GPSStddevVelX
uint16 GPSStddevVelY
uint16 GPSStddevVelZ
float32 fGPSStddevVelX
float32 fGPSStddevVelY
float32 fGPSStddevVelZ

# GPS Time UTC
# GPSTimemsec in ms
uint32 GPSTimemsec
# GPSTimeWeek in week
uint16 GPSTimeWeek
# GPSTrigger in mu.s
uint16 GPSTrigger

# GPS AuxData 1
# GPSDiffAge in s
uint16 GPSDiffAge
float32 fGPSDiffAge
int8 GPSStatsUsed
int8 GPSStatsVisible
int8 GPSSatsDualAntUsed
int8 GPSSatsDualAntVisible

# GPS AuxData 2
# GPSLogDelay in Ta
int8 GPSLogDelay
# GPSReceiverLoad in %
int8 GPSReceiverLoad
float32 fGPSReceiverLoad

int32 GPSBaseNr

# INS Angle and GPS COG
# All units in deg
int16 INSRoll
int16 INSPitch
uint16 INSYaw
uint16 GPSCOG
float32 fINSRoll
float32 fINSPitch
float32 fINSYaw
float32 fGPSCOG

# GPS Height(MSL)
int32 GPSHeight
int16 Undulation
float32 fGPSHeight
float32 fUndulation

# GPS DualAnt Time UTC
# GPSDualAntTimemsec in ms
uint32 GPSDualAntTimemsec
# GPSDualAntTimeWeek in week
uint16 GPSDualAntTimeWeek

# GPS DualAnt Angle
# All units in deg
uint16 GPSDualAntHeading
int16 GPSDualAntPitch
float32 fGPSDualAntHeading
float32 fGPSDualAntPitch

# GPS DualAnt Angle ETE
# All units in deg
int8 GPSDualAntStdDevHeading
int8 GPSDualAntStdDevPitch
float32 fGPSDualAntStdDevHeading
float32 fGPSDualAntStdDevPitch
int8 GPSDualAntStdDevHeading_HR
int8 GPSDualAntStdDevPitch_HR
float32 fGPSDualAntStdDevHeading_HR
float32 fGPSDualAntStdDevPitch_HR

# INS Position Height (MSL)
int32 INSHeight
float32 fINSHeight


# INS Position Height POI1 and POI2 (MSL)
int32 INSHeight_1
int32 INSHeight_2
float32 fINSHeight_1
float32 fINSHeight_2

# INS Position Height POI3 and POI4 (MSL)
int32 INSHeight_3
int32 INSHeight_4
float32 fINSHeight_3
float32 fINSHeight_4

# INS Position Height POI5 and POI6 (MSL)
int32 INSHeight_5
int32 INSHeight_6
float32 fINSHeight_5
float32 fINSHeight_6

# INS Position Height POI7 and POI8 (MSL)
int32 INSHeight_7
int32 INSHeight_8
float32 fINSHeight_7
float32 fINSHeight_8

# INS Time UTC
# INSTimemsec in ms
uint32 INSTimemsec
# INSTimeWeek in week
uint16 INSTimeWeek
# LeapSeconds in s
int16 LeapSeconds

# INS Position Absolute
# Position in deg
int32 INSLatAbs
int32 INSLonAbs
float64 fINSLatAbs
float64 fINSLonAbs

# INS Position Relative
# Position in m
int32 INSLatRel
int32 INSLonRel
float32 fINSLatRel
float32 fINSLonRel

# INS Position Absolute POS1
# Position in deg
int32 INSLatAbs_1
int32 INSLonAbs_1
float64 fINSLatAbs_1
float64 fINSLonAbs_1

# INS Position Relative POS1
# Position in m
int32 INSLatRel_1
int32 INSLonRel_1
float32 fINSLatRel_1
float32 fINSLonRel_1

# INS Position Absolute POS2
# Position in deg
int32 INSLatAbs_2
int32 INSLonAbs_2
float64 fINSLatAbs_2
float64 fINSLonAbs_2

# INS Position Relative POS2
# Position in m
int32 INSLatRel_2
int32 INSLonRel_2
float32 fINSLatRel_2
float32 fINSLonRel_2

# INS Position Absolute POS3
# Position in deg
int32 INSLatAbs_3
int32 INSLonAbs_3
float64 fINSLatAbs_3
float64 fINSLonAbs_3

# INS Position Relative POS3
# Position in m
int32 INSLatRel_3
int32 INSLonRel_3
float32 fINSLatRel_3
float32 fINSLonRel_3

# INS Position Absolute POS4
# Position in deg
int32 INSLatAbs_4
int32 INSLonAbs_4
float64 fINSLatAbs_4
float64 fINSLonAbs_4

# INS Position Relative POS4
# Position in m
int32 INSLatRel_4
int32 INSLonRel_4
float32 fINSLatRel_4
float32 fINSLonRel_4

# INS Position Absolute POS5
# Position in deg
int32 INSLatAbs_5
int32 INSLonAbs_5
float64 fINSLatAbs_5
float64 fINSLonAbs_5

# INS Position Relative POS5
# Position in m
int32 INSLatRel_5
int32 INSLonRel_5
float32 fINSLatRel_5
float32 fINSLonRel_5

# INS Position Absolute POS6
# Position in deg
int32 INSLatAbs_6
int32 INSLonAbs_6
float64 fINSLatAbs_6
float64 fINSLonAbs_6

# INS Position Relative POS6
# Position in m
int32 INSLatRel_6
int32 INSLonRel_6
float32 fINSLatRel_6
float32 fINSLonRel_6

# INS Position Absolute POS7
# Position in deg
int32 INSLatAbs_7
int32 INSLonAbs_7
float64 fINSLatAbs_7
float64 fINSLonAbs_7

# INS Position Relative POS7
# Position in m
int32 INSLatRel_7
int32 INSLonRel_7
float32 fINSLatRel_7
float32 fINSLonRel_7

# INS Position Absolute POS8
# Position in deg
int32 INSLatAbs_8
int32 INSLonAbs_8
float64 fINSLatAbs_8
float64 fINSLonAbs_8

# INS Position Relative POS8
# Position in m
int32 INSLatRel_8
int32 INSLonRel_8
float32 fINSLatRel_8
float32 fINSLonRel_8

# INS Velocity Horizontal
# Velocity in m/s
int16 INSVelHorX
int16 INSVelHorY
int16 INSVelHorZ
float32 fINSVelHorX
float32 fINSVelHorY
float32 fINSVelHorZ

# INS Velocity Frame
# Velocity in m/s
int16 INSVelFrameX
int16 INSVelFrameY
int16 INSVelFrameZ
float32 fINSVelFrameX
float32 fINSVelFrameY
float32 fINSVelFrameZ


# INS Velocity Horizontal POI1
# Velocity in m/s
int16 INSVelHorX_1
int16 INSVelHorY_1
int16 INSVelHorZ_1
float32 fINSVelHorX_1
float32 fINSVelHorY_1
float32 fINSVelHorZ_1

# INS Velocity Horizontal POI2
# Velocity in m/s
int16 INSVelHorX_2
int16 INSVelHorY_2
int16 INSVelHorZ_2
float32 fINSVelHorX_2
float32 fINSVelHorY_2
float32 fINSVelHorZ_2

# INS Velocity Horizontal POI3
# Velocity in m/s
int16 INSVelHorX_3
int16 INSVelHorY_3
int16 INSVelHorZ_3
float32 fINSVelHorX_3
float32 fINSVelHorY_3
float32 fINSVelHorZ_3

# INS Velocity Horizontal POI4
# Velocity in m/s
int16 INSVelHorX_4
int16 INSVelHorY_4
int16 INSVelHorZ_4
float32 fINSVelHorX_4
float32 fINSVelHorY_4
float32 fINSVelHorZ_4

# INS Velocity Horizontal POI5
# Velocity in m/s
int16 INSVelHorX_5
int16 INSVelHorY_5
int16 INSVelHorZ_5
float32 fINSVelHorX_5
float32 fINSVelHorY_5
float32 fINSVelHorZ_5

# INS Velocity Horizontal POI6
# Velocity in m/s
int16 INSVelHorX_6
int16 INSVelHorY_6
int16 INSVelHorZ_6
float32 fINSVelHorX_6
float32 fINSVelHorY_6
float32 fINSVelHorZ_6

# INS Velocity Horizontal POI7
# Velocity in m/s
int16 INSVelHorX_7
int16 INSVelHorY_7
int16 INSVelHorZ_7
float32 fINSVelHorX_7
float32 fINSVelHorY_7
float32 fINSVelHorZ_7

# INS Velocity Horizontal POI8
# Velocity in m/s
int16 INSVelHorX_8
int16 INSVelHorY_8
int16 INSVelHorZ_8
float32 fINSVelHorX_8
float32 fINSVelHorY_8
float32 fINSVelHorZ_8

# INS EPE
# Error in m
uint16 INSStddevLat
uint16 INSStddevLong
uint16 INSStddevHeight
float32 fINSStddevLat
float32 fINSStddevLong
float32 fINSStddevHeight

# INS EVE and INS ETE
#  Std vel in m/s
int8 INSStddevVelX
int8 INSStddevVelY
int8 INSStddevVelZ
float32 fINSStddevVelX
float32 fINSStddevVelY
float32 fINSStddevVelZ
# Std ETE in deg
int8 INSStddevRoll
int8 INSStddevPitch
int8 INSStddevYaw
float32 fINSStddevRoll
float32 fINSStddevPitch
float32 fINSStddevYaw

# Analog In1
# Analog in V
int16 AN1
int16 AN2
int16 AN3
int16 AN4
float32 fAN1
float32 fAN2
float32 fAN3
float32 fAN4

# Kalman Filter Status
# All uints in %
uint8 KFLatStimulated
uint8 KFLongStimulated
uint8 KFSteadyState

# GNSS Receiver Status
uint32 GPSReceiverError
uint32 GPSReceiverStatus


# MobileEye Time Comparison
float32 TimeMsec
float32 TimeNsec
