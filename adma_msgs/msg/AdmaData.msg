# adma static header
string genesysid
string headerversion
uint32 formatid
string formatversion
uint32 serialno
uint32 alias

# adma dynamic header
uint32 configid
uint32 configformat
uint32 configversion
uint32 configsize
uint32 byteoffset
uint32 slicesize
uint32 slicedata

# status
int32 statusgpsmode
int32 statusstandstill
int32 statusskidding
int32 statusexternalvelout
int32 statustriggps
int32 statussignalin3
int32 statussignalin2
int32 statussignalin1
int32 statusalignment
int32 statusahrsins
int32 statusdeadreckoning
int32 statussynclock
int32 statusevkactiv
int32 statusevkestimates
int32 statusheadingexecuted
int32 statusconfigurationchanged
int32 statustilt
int32 statuspos
int8 statuscount
int32 statuskalmanfiltersetteled
int32 statuskflatstimulated
int32 statuskflongstimulated
int32 statuskfsteadystate
int32 statusspeed

# error and warning
string errorhardware
string error_misc1
string error_misc2
string error_misc3
string warngps
string warnmisc1
string errorhwsticky

# sensors body x
# acc in g
int32 accbodyhrx
float64 faccbodyhrx
# rate in deg/s
int32 ratebodyhrx
float64 fratebodyhrx

# sensors body y
# acc in g
int32 accbodyhry
float64 faccbodyhry

# rate in deg/s
int32 ratebodyhry
float64 fratebodyhry


# sensors body z
# acc in g
int32 accbodyhrz
float64 faccbodyhrz

# rate in deg/s
int32 ratebodyhrz
float64 fratebodyhrz


# rates body
# rates in deg/s
int16 ratebodyx
int16 ratebodyy
int16 ratebodyz
float64 fratebodyx
float64 fratebodyy
float64 fratebodyz


# rates horz
# rates in deg/s
int16 ratehorx
int16 ratehory
int16 ratehorz
float64 fratehorx
float64 fratehory
float64 fratehorz

# accelerations body
# acc in g
int16 accbodyx
int16 accbodyy
int16 accbodyz
float64 faccbodyx
float64 faccbodyy
float64 faccbodyz

# acceleration horizontal
# acc in g
int16 acchorx
int16 acchory
int16 acchorz
float64 facchorx
float64 facchory
float64 facchorz

# accelerations body poi1
# acc in g
int16 accbodyx_1
int16 accbodyy_1
int16 accbodyz_1
float64 faccbodyx_1
float64 faccbodyy_1
float64 faccbodyz_1

# accelerations body poi2
# acc in g
int16 accbodyx_2
int16 accbodyy_2
int16 accbodyz_2
float64 faccbodyx_2
float64 faccbodyy_2
float64 faccbodyz_2

# accelerations body poi3
# acc in g
int16 accbodyx_3
int16 accbodyy_3
int16 accbodyz_3
float64 faccbodyx_3
float64 faccbodyy_3
float64 faccbodyz_3

# accelerations body poi4
# acc in g
int16 accbodyx_4
int16 accbodyy_4
int16 accbodyz_4
float64 faccbodyx_4
float64 faccbodyy_4
float64 faccbodyz_4

# accelerations body poi5
# acc in g
int16 accbodyx_5
int16 accbodyy_5
int16 accbodyz_5
float64 faccbodyx_5
float64 faccbodyy_5
float64 faccbodyz_5

# accelerations body poi6
# acc in g
int16 accbodyx_6
int16 accbodyy_6
int16 accbodyz_6
float64 faccbodyx_6
float64 faccbodyy_6
float64 faccbodyz_6

# accelerations body poi7
# acc in g
int16 accbodyx_7
int16 accbodyy_7
int16 accbodyz_7
float64 faccbodyx_7
float64 faccbodyy_7
float64 faccbodyz_7

# accelerations body poi8
# acc in g
int16 accbodyx_8
int16 accbodyy_8
int16 accbodyz_8
float64 faccbodyx_8
float64 faccbodyy_8
float64 faccbodyz_8

# acceleration horizontal poi1
# acc in g
int16 acchorx_1
int16 acchory_1
int16 acchorz_1
float64 facchorx_1
float64 facchory_1
float64 facchorz_1

# acceleration horizontal poi2
# acc in g
int16 acchorx_2
int16 acchory_2
int16 acchorz_2
float64 facchorx_2
float64 facchory_2
float64 facchorz_2

# acceleration horizontal poi3
# acc in g
int16 acchorx_3
int16 acchory_3
int16 acchorz_3
float64 facchorx_3
float64 facchory_3
float64 facchorz_3

# acceleration horizontal poi4
# acc in g
int16 acchorx_4
int16 acchory_4
int16 acchorz_4
float64 facchorx_4
float64 facchory_4
float64 facchorz_4

# acceleration horizontal poi5
# acc in g
int16 acchorx_5
int16 acchory_5
int16 acchorz_5
float64 facchorx_5
float64 facchory_5
float64 facchorz_5

# acceleration horizontal poi6
# acc in g
int16 acchorx_6
int16 acchory_6
int16 acchorz_6
float64 facchorx_6
float64 facchory_6
float64 facchorz_6

# acceleration horizontal poi7
# acc in g
int16 acchorx_7
int16 acchory_7
int16 acchorz_7
float64 facchorx_7
float64 facchory_7
float64 facchorz_7

# acceleration horizontal poi8
# acc in g
int16 acchorx_8
int16 acchory_8
int16 acchorz_8
float64 facchorx_8
float64 facchory_8
float64 facchorz_8

# external velocity analog
#  vel in m/s
int16 extvelanx
int16 extvelany
float64 fextvelanx
float64 fextvelany

# external velocity digital pulses
#  vel in m/s
int16 extveldigx
int16 extveldigy
int16 extveldigpulsesx
int16 extveldigpulsesy
float64 fextveldigx
float64 fextveldigy


# externaal velocity corrected
#  vel in m/s
int16 extvelxcorrected
int16 extvelycorrected
float64 fextvelxcorrected
float64 fextvelycorrected

# barometer pressure
# pressure in mbar
uint32 extbaropressure
float64 fextbaropressure

# barometer height
# height in m
int32 extbaroheight
int32 extbaroheightcorrected
float64 fextbaroheight
float64 fextbaroheightcorrected

# miscellaneous
# invpath radius in 1/m
int16 invpathradius
float64 finvpathradius
# side slip in degrees
int16 sideslipangle
float64 fsideslipangle
# dis_trav in m
uint32 disttrav
float64 fdisttrav

# miscellaneous_poi1
# invpath radius in 1/m
int16 invpathradius_1
float64 finvpathradius_1
# side slip in degrees
int16 sideslipangle_1
float64 fsideslipangle_1
# dis_trav in m
uint32 disttrav_1
float64 fdisttrav_1

# miscellaneous_poi2
# invpath radius in 1/m
int16 invpathradius_2
float64 finvpathradius_2
# side slip in degrees
int16 sideslipangle_2
float64 fsideslipangle_2
# dis_trav in m
uint32 disttrav_2
float64 fdisttrav_2

# miscellaneous_poi3
# invpath radius in 1/m
int16 invpathradius_3
float64 finvpathradius_3
# side slip in degrees
int16 sideslipangle_3
float64 fsideslipangle_3
# dis_trav in m
uint32 disttrav_3
float64 fdisttrav_3

# miscellaneous_poi4
# invpath radius in 1/m
int16 invpathradius_4
float64 finvpathradius_4
# side slip in degrees
int16 sideslipangle_4
float64 fsideslipangle_4
# dis_trav in m
uint32 disttrav_4
float64 fdisttrav_4

# miscellaneous_poi5
# invpath radius in 1/m
int16 invpathradius_5
float64 finvpathradius_5
# side slip in degrees
int16 sideslipangle_5
float64 fsideslipangle_5
# dis_trav in m
uint32 disttrav_5
float64 fdisttrav_5

# miscellaneous_poi6
# invpath radius in 1/m
int16 invpathradius_6
float64 finvpathradius_6
# side slip in degrees
int16 sideslipangle_6
float64 fsideslipangle_6
# dis_trav in m
uint32 disttrav_6
float64 fdisttrav_6

# miscellaneous_poi7
# invpath radius in 1/m
int16 invpathradius_7
float64 finvpathradius_7
# side slip in degrees
int16 sideslipangle_7
float64 fsideslipangle_7
# dis_trav in m
uint32 disttrav_7
float64 fdisttrav_7

# miscellaneous_poi8
# invpath radius in 1/m
int16 invpathradius_8
float64 finvpathradius_8
# side slip in degrees
int16 sideslipangle_8
float64 fsideslipangle_8
# dis_trav in m
uint32 disttrav_8
float64 fdisttrav_8


# triggers 1 and 2
# triggers in mu.s
uint16 trigrising1
uint16 trigfalling1
uint16 trigrising2
uint16 trigfalling2


# triggers 3 and 4
# triggers in mu.s
uint16 trigrising3
uint16 trigfalling3
uint16 trigrising4
uint16 trigfalling4

# system data
# systemta in mu.s
uint16 systemta
# systemtemp in deg. c
int16 systemtemp
float64 fsystemtemp
# systemtimesinceinit in s
uint16 systemtimesinceinit
# system dsp load in %
uint16 systemdspload
float64 fsystemdspload


# gps position absolute
# position in deg
int32 gpslatabs
int32 gpslonabs
float64 fgpslatabs
float64 fgpslonabs

# gps position relative
# position in m
int32 gpslatrel
int32 gpslonrel
float64 fgpslatrel
float64 fgpslonrel

# gps epe
#  error in m
uint16 gpsstddevlat
uint16 gpsstddevlon
uint16 gpsstddevheight
float64 fgpsstddevlat
float64 fgpsstddevlon
float64 fgpsstddevheight


# gps velocity frame
# vel in m/s
int16 gpsvelframex
int16 gpsvelframey
int16 gpsvelframez
float64 fgpsvelframex
float64 fgpsvelframey
float64 fgpsvelframez
# latency in s
uint16 gpsvellatency
float64 fgpsvellatency


# gps eve
# error in m/s
uint16 gpsstddevvelx
uint16 gpsstddevvely
uint16 gpsstddevvelz
float64 fgpsstddevvelx
float64 fgpsstddevvely
float64 fgpsstddevvelz

# gps time utc
# gpstimemsec in ms
uint32 gpstimemsec
# gpstimeweek in week
uint16 gpstimeweek
# gpstrigger in mu.s
uint16 gpstrigger

# gps auxdata 1
# gpsdiffage in s
uint16 gpsdiffage
float64 fgpsdiffage
int8 gpssatsused
int8 gpssatsvisible
int8 gpssatsdualantused
int8 gpssatsdualantvisible

# gps auxdata 2
# gpslogdelay in ta
int8 gpslogdelay
# gpsreceiverload in %
int8 gpsreceiverload
float64 fgpsreceiverload

int32 gpsbasenr

# ins angle and gps cog
# all units in deg
int16 insroll
int16 inspitch
uint16 insyaw
uint16 gpscog
float64 finsroll
float64 finspitch
float64 finsyaw
float64 fgpscog

# gps height(msl)
int32 gpsheight
int16 undulation
float64 fgpsheight
float64 fundulation

# gps dualant time utc
# gpsdualanttimemsec in ms
uint32 gpsdualanttimemsec
# gpsdualanttimeweek in week
uint16 gpsdualanttimeweek

# gps dualant angle
# all units in deg
uint16 gpsdualantheading
int16 gpsdualantpitch
float64 fgpsdualantheading
float64 fgpsdualantpitch

# gps dualant angle ete
# all units in deg
int8 gpsdualantstddevheading
int8 gpsdualantstddevpitch
float64 fgpsdualantstddevheading
float64 fgpsdualantstddevpitch
int8 gpsdualantstddevheading_hr
int8 gpsdualantstddevpitch_hr
float32 fgpsdualantstddevheading_hr
float32 fgpsdualantstddevpitch_hr

# ins position height (msl)
int32 insheight
float64 finsheight


# ins position height poi1 and poi2 (msl)
int32 insheight_1
int32 insheight_2
float64 finsheight_1
float64 finsheight_2

# ins position height poi3 and poi4 (msl)
int32 insheight_3
int32 insheight_4
float64 finsheight_3
float64 finsheight_4

# ins position height poi5 and poi6 (msl)
int32 insheight_5
int32 insheight_6
float64 finsheight_5
float64 finsheight_6

# ins position height poi7 and poi8 (msl)
int32 insheight_7
int32 insheight_8
float64 finsheight_7
float64 finsheight_8

# ins time utc
# instimemsec in ms
uint32 instimemsec
# instimeweek in week
uint16 instimeweek
# leapseconds in s
int16 leapseconds

# ins position absolute
# position in deg
int32 inslatabs
int32 inslonabs
float64 finslatabs
float64 finslonabs

# ins position relative
# position in m
int32 inslatrel
int32 inslonrel
float64 finslatrel
float64 finslonrel

# ins position absolute pos1
# position in deg
int32 inslatabs_1
int32 inslonabs_1
float64 finslatabs_1
float64 finslonabs_1

# ins position relative pos1
# position in m
int32 inslatrel_1
int32 inslonrel_1
float64 finslatrel_1
float64 finslonrel_1

# ins position absolute pos2
# position in deg
int32 inslatabs_2
int32 inslonabs_2
float64 finslatabs_2
float64 finslonabs_2

# ins position relative pos2
# position in m
int32 inslatrel_2
int32 inslonrel_2
float64 finslatrel_2
float64 finslonrel_2

# ins position absolute pos3
# position in deg
int32 inslatabs_3
int32 inslonabs_3
float64 finslatabs_3
float64 finslonabs_3

# ins position relative pos3
# position in m
int32 inslatrel_3
int32 inslonrel_3
float64 finslatrel_3
float64 finslonrel_3

# ins position absolute pos4
# position in deg
int32 inslatabs_4
int32 inslonabs_4
float64 finslatabs_4
float64 finslonabs_4

# ins position relative pos4
# position in m
int32 inslatrel_4
int32 inslonrel_4
float64 finslatrel_4
float64 finslonrel_4

# ins position absolute pos5
# position in deg
int32 inslatabs_5
int32 inslonabs_5
float64 finslatabs_5
float64 finslonabs_5

# ins position relative pos5
# position in m
int32 inslatrel_5
int32 inslonrel_5
float64 finslatrel_5
float64 finslonrel_5

# ins position absolute pos6
# position in deg
int32 inslatabs_6
int32 inslonabs_6
float64 finslatabs_6
float64 finslonabs_6

# ins position relative pos6
# position in m
int32 inslatrel_6
int32 inslonrel_6
float64 finslatrel_6
float64 finslonrel_6

# ins position absolute pos7
# position in deg
int32 inslatabs_7
int32 inslonabs_7
float64 finslatabs_7
float64 finslonabs_7

# ins position relative pos7
# position in m
int32 inslatrel_7
int32 inslonrel_7
float64 finslatrel_7
float64 finslonrel_7

# ins position absolute pos8
# position in deg
int32 inslatabs_8
int32 inslonabs_8
float64 finslatabs_8
float64 finslonabs_8

# ins position relative pos8
# position in m
int32 inslatrel_8
int32 inslonrel_8
float64 finslatrel_8
float64 finslonrel_8

# ins velocity horizontal
# velocity in m/s
int16 insvelhorx
int16 insvelhory
int16 insvelhorz
float64 finsvelhorx
float64 finsvelhory
float64 finsvelhorz

# ins velocity frame
# velocity in m/s
int16 insvelframex
int16 insvelframey
int16 insvelframez
float64 finsvelframex
float64 finsvelframey
float64 finsvelframez


# ins velocity horizontal poi1
# velocity in m/s
int16 insvelhorx_1
int16 insvelhory_1
int16 insvelhorz_1
float64 finsvelhorx_1
float64 finsvelhory_1
float64 finsvelhorz_1

# ins velocity horizontal poi2
# velocity in m/s
int16 insvelhorx_2
int16 insvelhory_2
int16 insvelhorz_2
float64 finsvelhorx_2
float64 finsvelhory_2
float64 finsvelhorz_2

# ins velocity horizontal poi3
# velocity in m/s
int16 insvelhorx_3
int16 insvelhory_3
int16 insvelhorz_3
float64 finsvelhorx_3
float64 finsvelhory_3
float64 finsvelhorz_3

# ins velocity horizontal poi4
# velocity in m/s
int16 insvelhorx_4
int16 insvelhory_4
int16 insvelhorz_4
float64 finsvelhorx_4
float64 finsvelhory_4
float64 finsvelhorz_4

# ins velocity horizontal poi5
# velocity in m/s
int16 insvelhorx_5
int16 insvelhory_5
int16 insvelhorz_5
float64 finsvelhorx_5
float64 finsvelhory_5
float64 finsvelhorz_5

# ins velocity horizontal poi6
# velocity in m/s
int16 insvelhorx_6
int16 insvelhory_6
int16 insvelhorz_6
float64 finsvelhorx_6
float64 finsvelhory_6
float64 finsvelhorz_6

# ins velocity horizontal poi7
# velocity in m/s
int16 insvelhorx_7
int16 insvelhory_7
int16 insvelhorz_7
float64 finsvelhorx_7
float64 finsvelhory_7
float64 finsvelhorz_7

# ins velocity horizontal poi8
# velocity in m/s
int16 insvelhorx_8
int16 insvelhory_8
int16 insvelhorz_8
float64 finsvelhorx_8
float64 finsvelhory_8
float64 finsvelhorz_8

# ins epe
# error in m
uint16 insstddevlat
uint16 insstddevlong
uint16 insstddevheight
float64 finsstddevlat
float64 finsstddevlong
float64 finsstddevheight

# ins eve and ins ete
#  std vel in m/s
int8 insstddevvelx
int8 insstddevvely
int8 insstddevvelz
float64 finsstddevvelx
float64 finsstddevvely
float64 finsstddevvelz
# std ete in deg
int8 insstddevroll
int8 insstddevpitch
int8 insstddevyaw
float64 finsstddevroll
float64 finsstddevpitch
float64 finsstddevyaw

# analog in1
# analog in v
int16 an1
int16 an2
int16 an3
int16 an4
float64 fan1
float64 fan2
float64 fan3
float64 fan4

# kalman filter status
# all uints in %
uint8 kflatstimulated
uint8 kflongstimulated
uint8 kfsteadystate

# gnss receiver status
uint32 gpsreceivererror
uint32 gpsreceiverstatus


# mobileeye time comparison
float64 timemsec
float64 timensec