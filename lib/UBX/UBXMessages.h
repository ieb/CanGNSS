#pragma once

#include <Arduino.h>
/**
 * 
 * To use
 * 
 * Get the buffer using UBXRead.read(); uBbxRead.getMesage(); and cast a struct to the buffer eg 
 * NavClock * navClock = (NavClock *)uBbxRead.getMesage();
 * Serial.print("TimeOfWeek"); Serial.println(navClock->iTOW);
 *
 * Some messages have repeats, which always come at blocks after the message.
 * NavDGPS * navDGPS = (NavDGPS *)uBbxRead.getMesage();
 * NavDGPS_CH * navDGPSCH = (NavDGPS_CH *)(uBbxRead.getMesage()+sizeof(NavDGPS));
 * for ( int i = ; i < navDGPS->numchan; i++ ) {
 *     Serial.print("Satelite ID:"); Serial.println(navDGPSCH[i].svid);
 * }
 */

#define ULBOX_SYNC1 0xB5
#define ULBOX_SYNC2 0x62


typedef struct _UbloxHeader {
    uint8_t sync1; // 0x5B
    uint8_t sync2; // 0x62
    uint8_t messageClass;
    uint8_t messageId;
    uint16_t payloadLength;
} UbloxHeader;


// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

// sections  from https://github.com/GAVLab/ubloxM8/blob/master/include/ubloxM8/ublox_m8_structures.h
// with additional information from http://docs.ros.org/en/noetic/api/ublox_msgs/html/index-msg.html

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))


//! UBX Protocol Class/Message ID's
#define MSG_CLASS_ACK 0X05
    #define MSG_ID_ACK_ACK 0x01
    #define MSG_ID_ACK_NAK 0x00
#define MSG_CLASS_CFG 0x06
    #define MSG_ID_CFG_ANT 0X13
    #define MSG_ID_CFG_CFG 0x09
    #define MSG_ID_CFG_DAT 0x06
    #define MSG_ID_CFG_DOSC 0x61
    #define MSG_ID_CFG_DYNSEED 0x85
    #define MSG_ID_CFG_ESRC 0x60
    #define MSG_ID_CFG_FIXSEED 0x84
    #define MSG_ID_CFG_GEOFENCE 0x69
    #define MSG_ID_CFG_GNSS 0x3E
    #define MSG_ID_CFG_INF 0x02
    #define MSG_ID_CFG_ITFM 0x39
    #define MSG_ID_CFG_LOGFILTER 0x47
    #define MSG_ID_CFG_MSG 0x01
    #define MSG_ID_CFG_NAV5 0x24
    #define MSG_ID_CFG_NAVX5 0x23
    #define MSG_ID_CFG_NMEA 0x17
    #define MSG_ID_CFG_ODO 0x1E
    #define MSG_ID_CFG_PM2 0x3B
    #define MSG_ID_CFG_PRT 0x00
    #define MSG_ID_CFG_PWR 0x57
    #define MSG_ID_CFG_RATE 0x08
    #define MSG_ID_CFG_RINV 0x34
    #define MSG_ID_CFG_RST 0x04
    #define MSG_ID_CFG_RXM 0x11
    #define MSG_ID_CFG_SBAS 0x16
    #define MSG_ID_CFG_SMGR 0x62
    #define MSG_ID_CFG_TMODE2 0x3D
    #define MSG_ID_CFG_TP5 0x31
    #define MSG_ID_CFG_TXSLOT 0x53
    #define MSG_ID_CFG_USB 0x1B
#define MSG_CLASS_INF 0x04
    #define MSG_ID_INF_DEBUG 0x04
    #define MSG_ID_INF_ERROR 0x00
    #define MSG_ID_INF_NOTICE 0x02
    #define MSG_ID_INF_TEST 0x03
    #define MSG_ID_INF_WARNING 0x01
#define MSG_CLASS_LOG 0x21
    #define MSG_ID_LOG_CREATE 0x07
    #define MSG_ID_LOG_ERASE 0x03
    #define MSG_ID_LOG_FINDTIME 0x0E
    #define MSG_ID_LOG_INFO 0x08
    #define MSG_ID_LOG_RETRIEVEPOSEXTRA 0x0F
    #define MSG_ID_LOG_RETRIEVEPOS 0x0B
    #define MSG_ID_LOG_RETRIEVESTRING 0x0d
    #define MSG_ID_LOG_RETRIEVE 0x09
    #define MSG_ID_LOG_STRING 0x04
#define MSG_CLASS_MGA 0x13
    #define MSG_ID_MGA_ACK 0x60
    #define MSG_ID_MGA_ANO 0x20
    #define MSG_ID_MGA_DBD 0x80
    #define MSG_ID_MGA_FLASH_DATA 0x21
    #define MSG_ID_MGA_FLASH_STOP 0x21
    #define MSG_ID_MGA_FLASH 0x21
    #define MSG_ID_MGA_GAL_EPH 0x02
    #define MSG_ID_MGA_GAL_ALM 0x02
    #define MSG_ID_MGA_GAL_TIMEOFFSET 0x02
    #define MSG_ID_MGA_GAL_UTC 0x02
    #define MSG_ID_MGA_GLO_EPH 0x06
    #define MSG_ID_MGA_GLO_ALM 0x06
    #define MSG_ID_MGA_GLO_TIMEOFFSET 0x06
    #define MSG_ID_MGA_GPS_EPH 0x00
    #define MSG_ID_MGA_GPS_ALM 0x00
    #define MSG_ID_MGA_GPS_UTC 0x00
    #define MSG_ID_MGA_GPS_IONO 0x00
    #define MSG_ID_MGA_INI_POS_XYZ 0x40
    #define MSG_ID_MGA_INI_POS_LLH 0x40
    #define MSG_ID_MGA_INI_TIME_UTC 0x40
    #define MSG_ID_MGA_INI_TIME_GNSS 0x40
    #define MSG_ID_MGA_INI_CLKD 0x40
    #define MSG_ID_MGA_INI_FREQ 0x40
    #define MSG_ID_MGA_INI_EOP 0x40
    #define MSG_ID_MGA_QZSS_EPH 0x05
    #define MSG_ID_MGA_QZSS_ALM 0x05
    #define MSG_ID_MGA_QZSS_HEALTH 0x05
    //Leaving for backward compatability
    #define MSG_ID_MGA_FLASH 0x21
    #define MSG_ID_MGA_GLO 0x06
    #define MSG_ID_MGA_GPS 0x00
    #define MSG_ID_MGA_INI 0x40
    #define MSG_ID_MGA_QZSS 0x05
#define MSG_CLASS_MON 0x0A
    #define MSG_ID_MON_GNSS 0x28
    #define MSG_ID_MON_HW2 0x0B
    #define MSG_ID_MON_HW 0x09
    #define MSG_ID_MON_IO 0x02
    #define MSG_ID_MON_MSGPP 0x06
    #define MSG_ID_MON_PATCH 0x27
    #define MSG_ID_MON_RXBUF 0x07
    #define MSG_ID_MON_RXR 0x21
    #define MSG_ID_MON_SMGR 0x2E
    #define MSG_ID_MON_TXBUF 0X08
    #define MSG_ID_MON_VER 0x04
#define MSG_CLASS_NAV 0x01
    #define MSG_ID_NAV_AOPSTATUS 0x60
    #define MSG_ID_NAV_CLOCK 0x22
    #define MSG_ID_NAV_DGPS 0x31
    #define MSG_ID_NAV_DOP 0x04
    #define MSG_ID_NAV_EOE 0x61
    #define MSG_ID_NAV_GEOFENCE 0x39
    #define MSG_ID_NAV_ODO 0x09
    #define MSG_ID_NAV_ORB 0x34
    #define MSG_ID_NAV_POSECEF 0x01
    #define MSG_ID_NAV_POSLLH 0x02
    #define MSG_ID_NAV_PVT 0x07
    #define MSG_ID_NAV_RESETODO 0x10
    #define MSG_ID_NAV_SAT 0x35
    #define MSG_ID_NAV_SBAS 0x32
    #define MSG_ID_NAV_SOL 0x06
    #define MSG_ID_NAV_STATUS 0x03
    #define MSG_ID_NAV_SVINFO 0x30
    #define MSG_ID_NAV_TIMEBDS 0x24
    #define MSG_ID_NAV_TIMEGAL 0x25
    #define MSG_ID_NAV_TIMEGLO 0x23
    #define MSG_ID_NAV_TIMEGPS 0x20
    #define MSG_ID_NAV_TIMELS 0x26
    #define MSG_ID_NAV_TIMEUTC 0x21
    #define MSG_ID_NAV_VELECEF 0x11
    #define MSG_ID_NAV_VELNED 0x12
#define MSG_CLASS_RXM 0x02
    #define MSG_ID_RXM_IMES 0x61
    #define MSG_ID_RXM_PMREQ 0x41
    #define MSG_ID_RXM_RAWX 0x15
    #define MSG_ID_RXM_RLM 0x59
    #define MSG_ID_RXM_SFRBX 0x13
    #define MSG_ID_RXM_SVSI 0x20
#define MSG_CLASS_SEC 0x27
    #define MSG_ID_SEC_SIGN 0x01
    #define MSG_ID_SEC_UNIQID 0x03
#define MSG_CLASS_TIM 0x0D
    #define MSG_ID_TIM_DOSC 0x11
    #define MSG_ID_TIM_FCHG 0x16
    #define MSG_ID_TIM_HOC 0x17
    #define MSG_ID_TIM_SMEAS 0x13
    #define MSG_ID_TIM_SVIN 0x04
    #define MSG_ID_TIM_TM2 0x03
    #define MSG_ID_TIM_TOS 0x12
    #define MSG_ID_TIM_TP 0x01
    #define MSG_ID_TIM_VCOCAL 0x15
    #define MSG_ID_TIM_VRFY 0x06
#define MSG_CLASS_UPD 0x09
    #define MSG_ID_UPD_SOS 0x14
// message type structures including the checksum byte.
// cls and id are always the first 2 bytes
// checksum is always the last byte
// we dont allocate these, they are used
// as pointers to the packet buffer to simplify decoding.

PACK(
struct CfgReset {
    UbloxHeader header;
    uint16_t navBbrMask;
    // 0x0000 hot start
    // 0x0001 warm start
    // 0xFFFF cold start
    uint8_t resetMode;
    // 0x00 harware, 
    // 0x01 controlled sofware
    // 0x02 controlled software GNSS
    // 0x04 Hadrware watchdog after shutdown
    // 0x08 Controlled GNSS stop
    // 0x09 Controlled GNSS start  
    uint8_t reserved;
    uint8_t checksum[2];
});


PACK(
struct CfgMsg {
    UbloxHeader header;
    uint8_t messageClass;
    uint8_t messageId;
    uint8_t rates[6];
    uint8_t checksum[2];
});

PACK(
struct CfgRate {
    UbloxHeader header;
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
    uint8_t checksum[2];
});


PACK(
struct CfgUart {
    UbloxHeader header;
    uint8_t portID;
    uint8_t reserved1;
    uint16_t txReady;  // 0 no
    uint32_t mode; 
    uint32_t baud;  // 0x000008C0  0b100011000000  8-N-1
    uint16_t inProtoMask; // 3  ubx + nemea
    uint16_t outProtoMask; // 3 ubx + nemea
    uint16_t flags; // 0 , no extended tx buffer.
    uint8_t reserved2[2];
    uint8_t checksum[2];
});

/*!
* NAV-CLOCK Message Structure
* Clock Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x22  Payload Length= 20 bytes
* Total Lenght 6+20 == 26
*/

#define NAV_CLOCK_BUFSIZE (6+20+2)

    PACK(
            struct NavClock{
                UbloxHeader header;
                uint32_t iTOW;
                int32_t clkbias;    // clock bias in nanoseconds
                int32_t clkdrift;   // clock drift in ns/s
                uint32_t tacc;      // time accuracy estimate (ns)
                uint32_t facc;      // frequency accuracy estimate (ps/s)
                uint8_t checksum[2];
            });

/*!
* NAV-DGPS Message Structure
* DGPS Data Used for NAV
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x31  Payload Length= (16 + 12*numChannels bytes)
* Total length 6 + (16 + 12*numChannels bytes)
*/

#define NAV_DGPS_BUFSIZE(n) (6+16+12*(n)+2)

    PACK(
            struct NavDGPS_CH{
                uint8_t svid;
                uint8_t flags;  // bitfield containing channel each sv is on and DGPS status
                uint16_t agecorr;   // age of latest correction data (ms)
                float prcorr;   // psuedorange correction   (m)
                float prrcorr;  // psuedorange rate correction (m/sec)
            });

    PACK(
            struct NavDGPS{
                UbloxHeader header;
                uint32_t iTOW;  // GPS ms time of week
                int32_t age;    // age of newest correction data (ms)
                int16_t baseID; // DGPS base station ID
                int16_t basehealth; // DGPS base station health
                uint8_t numchan;    // nomber of channels for which correction data is following
                uint8_t status; // DGPS correction type status
                uint16_t reserved;  // reserved
                NavDGPS_CH satelites[1]; 
            });

/*!
* NAV-DOP Message Structure
* This message outputs various DOPs. All
* DOP values are scaled by a factor of 100.
* Ex. If gdop contains a value 156, the true
* value is 1.56
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x04  Payload Length= 18 bytes
* Total length = 6+18 = 24
*/

/*
# NAV-DOP (0x01 0x04)
# Dilution of precision
#
# - DOP values are dimensionless.
# - All DOP values are scaled by a factor of 100. If the unit transmits a value 
#   of e.g. 156, the DOP value is 1.56.
#
*/

#define NAV_DOP_BUFSIZE (6+18+2)


    PACK(
            struct NavDOP{
                UbloxHeader header;
                uint32_t iTOW;  // GPS ms time of week (ms)
                uint16_t gdop;  // Geometric DOP [1 / 0.01]
                uint16_t pdop;  // Position DOP [1 / 0.01]
                uint16_t tdop;  // Time DOP [1 / 0.01]
                uint16_t vdop;  // Vertical DOP [1 / 0.01]
                uint16_t hdop;  // Horizontal DOP [1 / 0.01]
                uint16_t ndop;  // Northing DOP [1 / 0.01]
                uint16_t edop;  // Easting DOP [1 / 0.01]
                uint8_t checksum[2];
            });

/*!
* NAV-POSLLH Message Structure
* This message outputs the Geodetic position in
* the currently selected Ellipsoid. The default is
* the WGS84 Ellipsoid, but can be changed with the
* message CFG-DAT.
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x02  Payload Length=28 bytes
* Total length = 6+28 = 34
*/

#define NAV_POSLLH_BUFSIZE (6+28+2)

    PACK(
            struct NavPosLLH{
                UbloxHeader header;     //!< Ublox header
                uint32_t iTOW;          //!< GPS millisecond time of week
                int32_t longitude_scaled; //!< longitude in degrees. Scaling 1e-7
                int32_t latitude_scaled; //!< latitude in degrees. Scaling 1e-7
                int32_t height;          //!< height above ellipsoid [mm]
                int32_t height_mean_sea_level; //!< height above mean sea level [mm]
                uint32_t horizontal_accuracy; //!< horizontal accuracy estimate [mm]
                uint32_t vertical_accuracy; //!< vertical accuracy estimate [mm]
                uint8_t checksum[2];
            });

/*!
* NAV-PVT Message Structure
* Navigation Position Velocity Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x07  Payload Length=92 bytes
* Total length = 6+92 = 98
*/

#define NAV_PVT_BUFSIZE (6+92+2)

    PACK(
            struct NavPVT{
                UbloxHeader header;
                uint32_t iTOW;     // GPS time of week of the navigation epoch. [ms] See the description of iTOW for details.
                uint16_t year;     // Year (UTC)
                uint8_t month;     // Month, range 1..12 (UTC)
                uint8_t day;       // Day of month, range 1..31 (UTC)
                uint8_t hour;      // Hour of day, range 0..23 (UTC)
                uint8_t min;       // Minute of hour, range 0..59 (UTC)
                uint8_t sec;       // Seconds of minute, range 0..60 (UTC)
                uint8_t valid;     // Validity Flags (see graphic below)
                // 0x01 valid date
                // 0x02 valid time
                // 0x04 fully resolved
                // 0x08 valid magnetic declanation
                uint32_t tAcc;     // Time accuracy estimate (UTC) [ns]
                int32_t nano;      // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
                uint8_t fixType;       // GNSSfix Type, range 0..5 0x00 = No Fix
                // 0x01 = Dead Reckoning only 0x02 = 2D-Fix
                // 0x03 = 3D-Fix
                // 0x04 = GNSS + dead reckoning combined 
                // 0x05 = Time only fix
                // 0x06..0xff: reserved
                uint8_t flags;     // Fix Status Flags (see graphic below)
                // 1 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks
                // 2 FLAGS_DIFF_SOLN = 2            # DGPS used
                // 28 FLAGS_PSM_MASK = 28            # Power Save Mode
                // 0 PSM_OFF = 0                       # PSM is off
                // 4 PSM_ENABLED = 4                   # Enabled (state before acquisition)
                // 8 PSM_ACQUIRED = 8                  # Acquisition
                // 12 PSM_TRACKING = 12                 # Tracking
                // 16 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking
                // 20 PSM_INACTIVE = 20                 # Inactive
                // 32 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid
                // 192 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     
                // 0 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution
                // 64 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed 
                //                                        # integer measurements have been used to 
                //                                        # calculate the solution)
                // 128 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer 
                //                                        # carrier phase range measurements have 
                //                                        # been used to calculate  the solution)
                uint8_t additionalFlags;     // Additional Flags
                // 32 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of 
                //                                        # Day validity confirmation is available
                // 64 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed
                // 128 FLAGS2_CONFIRMED_TIME = 128                   
                uint8_t numSV;     // Number of satellites used in Nav Solution
                int32_t lon;       // Longitude deg/1e-7
                int32_t lat;       // Latitude deg/1e-7
                int32_t height;        // Height above ellipsoid [mm]
                int32_t hMSL;      // Height above mean sea level [mm]
                uint32_t hAcc;     // Horizontal accuracy estimate [mm]
                uint32_t vAcc;     // Vertical accuracy estimate [mm]
                int32_t velN;      // NED north velocity [mm/s]
                int32_t velE;      // NED east velocity [mm/s]
                int32_t velD;      // NED down velocity [mm/s]
                int32_t gSpeed;        // Ground Speed (2-D)  [mm/s]
                int32_t headMot;       // Heading of motion (2-D)  [deg/1e-5]
                uint32_t sAcc;     // Speed accuracy estimate [mm/s]
                uint32_t headAcc;      // Heading accuracy estimate (both motion and vehicle) [deg/1e-5]
                uint16_t pDOP;     // Position DOP  [1/0.01]
                uint8_t reserved2[6];  // Reserved 
                //
                // Note, fields below are missing from V7 firmware messages which end here.
                //
                int32_t headVeh;       // Heading of vehicle (2-D) [deg / 1e-5]
                int16_t magDec;         // Magnetic declination [deg / 1e-2]
                uint16_t magAcc;        // Magnetic declination accuracy [deg / 1e-2]
                uint8_t checksum[2];

            });


/*!
* NAV-SAT Message Structure
* Satellite Information
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x35  Payload Length=8 + 12*numSvs bytes
* Total = 6 + 8 + 12*numSvs
*/

#define NAV_SAT_BUFSIZE(n) (6+8+12*(n)+2)


    PACK(
            struct NavSat_SV{
                uint8_t gnssId;        // GNSS identifier (see Satellite numbering) for assignment
                uint8_t svId;      // Satellite identifier (see Satellite numbering) for assignment
                uint8_t cno;       // Carrier to noise ratio (signal strength dbHz)
                int8_t elev;       // Elevation (range: +/-90), unknown if out of range deg
                int16_t azim;      // Azimuth (range +/-180), unknown if elevation is out of range
                int16_t prRes;     // Pseudo range residual [0.1m]
                uint32_t flags;        // Bitmask (see graphic below)

/*
uint32 FLAGS_QUALITY_IND_MASK = 7     # Signal quality indicator:
uint8 QUALITY_IND_NO_SIGNAL = 0                     # no signal
uint8 QUALITY_IND_SEARCHING_SIGNAL = 1              # searching signal
uint8 QUALITY_IND_SIGNAL_ACQUIRED = 2               # signal acquired
uint8 QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE = 3  # signal detected but 
                                                    # unusable
uint8 QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC = 4     # code locked and time 
                                                    # synchronized
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1 = 5 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 1
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2 = 6 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 2
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 = 7 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 3
# Note: Since IMES signals are not time synchronized, a channel tracking an IMES 
# signal can never reach a quality indicator value of higher than 3.
uint32 FLAGS_SV_USED = 8                      # whether SV is currently being 
                                              # used for navigation
uint32 FLAGS_HEALTH_MASK = 48                 # SV health flag:
uint32 HEALTH_UNKNOWN = 0                       # unknown
uint32 HEALTH_HEALTHY = 1                       # healthy
uint32 HEALTH_UNHEALTHY = 2                     # unhealthy
uint32 FLAGS_DIFF_CORR = 64                   # whether differential correction 
                                              # data is available for this SV
uint32 FLAGS_SMOOTHED = 128                   # whether carrier smoothed 
                                              # pseudorange used
uint32 FLAGS_ORBIT_SOURCE_MASK = 1792         # Orbit source:
uint32 ORBIT_SOURCE_UNAVAILABLE = 0             # no orbit information is 
                                              # available for this SV
uint32 ORBIT_SOURCE_EPH = 256                   # ephemeris is used
uint32 ORBIT_SOURCE_ALM = 512                   # almanac is used
uint32 ORBIT_SOURCE_ASSIST_OFFLINE = 768        # AssistNow Offline orbit is 
                                                # used
uint32 ORBIT_SOURCE_ASSIST_AUTONOMOUS = 1024    # AssistNow Autonomous orbit is 
                                                # used
uint32 ORBIT_SOURCE_OTHER1 = 1280               # other orbit information is 
                                                # used
uint32 ORBIT_SOURCE_OTHER2 = 1536               # other orbit information is 
                                                # used
uint32 ORBIT_SOURCE_OTHER3 = 1792               # other orbit information is 
                                                # used
uint32 FLAGS_EPH_AVAIL = 2048                 # whether ephemeris is available 
                                              # for this SV
uint32 FLAGS_ALM_AVAIL = 4096                 # whether almanac is available for 
                                              # this SV
uint32 FLAGS_ANO_AVAIL = 8192                 # whether AssistNow Offline data 
                                              # is available for this SV
uint32 FLAGS_AOP_AVAIL = 16384                # whether AssistNow Autonomous 
                                              # data is available for this SV
uint32 FLAGS_SBAS_CORR_USED = 65536           # whether SBAS corrections have 
                                              # been used for this SV
uint32 FLAGS_RTCM_CORR_USED = 131072          # whether RTCM corrections have 
                                              # been used for this SV
uint32 FLAGS_PR_CORR_USED = 1048576           # whether Pseudorange corrections 
                                              # have been used for this SV
uint32 FLAGS_CR_CORR_USED = 2097152           # whether Carrier range 
                                              # corrections have been used for 
                                              # this SV
uint32 FLAGS_DO_CORR_USED = 4194304           # whether Range rate (Doppler) 
                                              # corrections have been used for 
                                              # this SV

*/
            });

    PACK(
            struct NavSat{
                UbloxHeader header;
                uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
                uint8_t version;       // Message version (1 for this version)
                uint8_t numSvs;        // Number of satellites
                uint8_t reserved1[2];     // Reserved
                NavSat_SV satelites[1]; // point to the first entry
                // NavSat_SV[numSvs]
            });

/*!
* NAV-SBAS Message Structure
* SBAS Status Data
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x32  Payload Length=12 + 12*cnt bytes
* Total = 6+ 12 +12 *count
*/

#define NAV_SBAS_BUFSIZE(n) (6+12+12*(n)+2)





    PACK(
            struct NavSBAS_SV {
                uint8_t svid;              // SV Id
                uint8_t flags;             // Flags for this SV
                uint8_t udre;              // Monitoring status
                uint8_t svSys;             // System (WAAS/EGNOS/...), same as SYS
                uint8_t svService;         // Services available, same as SERVICE
                uint8_t reserved1;         // Reserved
                int16_t prc;               // Pseudo Range correction in [cm]
                uint16_t reserved2;        // Reserved
                int16_t ic;                // Ionosphere correction in [cm]
            });

    PACK(
            struct NavSBAS {
                UbloxHeader header;
                uint32_t iTOW;      // Time of Week (ms)
                uint8_t geo;    // # PRN Number of the GEO where correction and integrity 
                                // # data is used from
                uint8_t mode;   // SBAS Mode
                //uint8 MODE_DISABLED = 0
                //uint8 MODE_ENABLED_INTEGRITY = 1
                // uint8 MODE_ENABLED_TESTMODE = 3
                int8_t system;       // SBAS System (WAAS/EGNOS/...)
                // int8 SYS_UNKNOWN = -1
                // int8 SYS_WAAS = 0
                // int8 SYS_EGNOS = 1
                // int8 SYS_MSAS = 2
                // int8 SYS_GAGAN = 3
                // int8 SYS_GPS = 16
                uint8_t service;           // SBAS Services available
                //uint8 SERVICE_RANGING = 1
                //uint8 SERVICE_CORRECTIONS = 2
                //uint8 SERVICE_INTEGRITY = 4
                //uint8 SERVICE_TESTMODE = 8
                uint8_t count;              // # Number of SV data following
                uint8_t reserved0[3];      // # Reserved
                NavSBAS_SV satelites[1]; // first entry
                // NavSBAS_SV[count] sv

            });

/*!
 * NAV-STATUS Message Structure
 * Receiver Navigation Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x01 0x03 Payload Length=16 bytes
 * Total 6+16 = 22
 */

 #define NAV_STATUS_BUFSIZE (6+16+2)
   
    PACK(
            struct NavStatus {
                UbloxHeader header;
                uint32_t iTOW;      // Time of Week (ms)
                uint8_t fixtype;    // 
                // no fix=0x00, 
                // deadreckoning only=0x01, 
                // 2D=0x02, 
                // 3D=0x03, 
                // deadreck+GPS=0x04, 
                // time fix only=0x05, 
                // reserved=0x06..0xff
                uint8_t flags;
                //uint8 FLAGS_GPS_FIX_OK = 1      # position & velocity valid & within DOP & ACC 
                //                # Masks
                //uint8 FLAGS_DIFF_SOLN = 2       # Differential corrections were applied
                //uint8 FLAGS_WKNSET = 4          # Week Number valid
                //uint8 FLAGS_TOWSET = 8          # Time of Week valid
                //
                //uint8 fixStat           # Fix Status Information
                //uint8 FIX_STAT_DIFF_CORR_MASK = 1       # 1 = differential corrections available
                uint8_t fixstat;
                //uint8 FIX_STAT_DIFF_CORR_MASK = 1       # 1 = differential corrections available
                //# map matching status:
                //uint8 FIX_STAT_MAP_MATCHING_MASK = 192
                //uint8 MAP_MATCHING_NONE = 0      # none
                //uint8 MAP_MATCHING_VALID = 64    # valid but not used, i.e. map matching data 
                //                                 # was received, but was too old
                //uint8 MAP_MATCHING_USED = 128    # valid and used, map matching data was applied
                //uint8 MAP_MATCHING_DR = 192      # valid and used, map matching data was 
                //                                 # applied. In case of sensor unavailability map
                //                                 # matching data enables dead reckoning. 
                //                                 # This requires map matched latitude/longitude 
                //                                 # or heading data.
                uint8_t flags2;
                //# power safe mode state (Only for FW version >= 7.01; undefined otherwise)
                //uint8 FLAGS2_PSM_STATE_MASK = 3
                //uint8 PSM_STATE_ACQUISITION = 0                # ACQUISITION 
                //                                               # [or when psm disabled]
                //uint8 PSM_STATE_TRACKING = 1                   # TRACKING
                //uint8 PSM_STATE_POWER_OPTIMIZED_TRACKING = 2   # POWER OPTIMIZED TRACKING
                //uint8 PSM_STATE_INACTIVE = 3                   # INACTIVE
                //# Note that the spoofing state value only reflects the detector state for the 
                //# current navigation epoch. As spoofing can be detected most easily at the 
                //# transition from real signal to spoofing signal, this is also where the 
                //# detector is triggered the most. I.e. a value of 1 - No spoofing indicated does
                //# not mean that the receiver is not spoofed, it #simply states that the detector
                //# was not triggered in this epoch.
                //uint8 FLAGS2_SPOOF_DET_STATE_MASK = 24 
                //uint8 SPOOF_DET_STATE_UNKNOWN = 0    # Unknown or deactivated
                //uint8 SPOOF_DET_STATE_NONE = 8       # No spoofing indicated
                //uint8 SPOOF_DET_STATE_SPOOFING = 16  # Spoofing indicated
                //uint8 SPOOF_DET_STATE_MULTIPLE = 24  # Multiple spoofing indication
                uint32_t ttff;      // TTFF (ms)
                uint32_t msss;      // Milliseconds since startup/reset
                uint8_t checksum[2];
            });


/*!
* NAV-SVINFO Message Structure
* This message outputs info about SVs each
* channel is tracking
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x30  Payload Length= (8+12*NumChannels bytes)
* Total = 6+ 8+12*NumChannels
*/

#define NAV_SVINFO_BUFSIZE(n) (6+8+12*(n)+2)

    PACK(
            struct NavSVInfo_SV{
                uint8_t ch_num;     //!< Channel Number (255 if SV isn't assigned to channel)
                uint8_t svid;       // Satellite ID number
                uint8_t flags;      // bitfield (description of contents follows)
                //uint8 FLAGS_SV_USED = 1                     # SV is used for navigation
                //uint8 FLAGS_DIFF_CORR = 2                   # Differential correction data 
                //                                            # is available for this SV
                //uint8 FLAGS_ORBIT_AVAIL = 4                 # Orbit information is available for 
                //                                            # this SV (Ephemeris or Almanach)
                //uint8 FLAGS_ORBIT_EPH = 8                   # Orbit information is Ephemeris
                //uint8 FLAGS_UNHEALTHY = 16                  # SV is unhealthy / shall not be 
                //                                            # used
                //uint8 FLAGS_ORBIT_ALM = 32                  # Orbit information is Almanac Plus
                //uint8 FLAGS_ORBIT_AOP = 64                  # Orbit information is AssistNow 
                //                                            # Autonomous
                //uint8 FLAGS_SMOOTHED = 128                  # Carrier smoothed pseudorange used

                uint8_t quality;    // signal quality indicator bitfield
                //# qualityInd: Signal Quality indicator (range 0..7). The following list shows 
                //# the meaning of the different QI values:
                //# Note: Since IMES signals are not time synchronized, a channel tracking an IMES
                //# signal can never reach a quality indicator value of higher than 3.
                //uint8 QUALITY_IDLE = 0                      # This channel is idle
                //uint8 QUALITY_SEARCHING = 1                 # Channel is searching
                //uint8 QUALITY_ACQUIRED = 2                   # Signal acquired
                //uint8 QUALITY_DETECTED = 3                  # Signal detected but unusable
                //uint8 QUALITY_CODE_LOCK = 4                 # Code Lock on Signal
                //uint8 QUALITY_CODE_AND_CARRIER_LOCKED1 = 5  # Code and Carrier locked 
                //                                            # and time synchronized
                //uint8 QUALITY_CODE_AND_CARRIER_LOCKED2 = 6  # Code and Carrier locked 
                //                                            # and time synchronized
                //uint8 QUALITY_CODE_AND_CARRIER_LOCKED3 = 7  # Code and Carrier locked 
                //                                            # and time synchronized
                uint8_t cno;        // carrier to noise ratio (dbHz)
                int8_t elev;        // elevation (deg)
                int16_t azim;       // azimuth (deg)
                int32_t prRes;      // Psuedorange residual (centimeters)
            });

    PACK(
            struct NavSVInfo{
                UbloxHeader header;     //!< Ublox header
                uint32_t iTOW;  // GPS time of week (ms)
                uint8_t numch;  //! number of channels following
                uint8_t global_flags;   // Chip and Hardware Generation
                //# Chip Hardware generation flags
                //uint8 CHIPGEN_ANTARIS = 0   # Antaris, Antaris 4
                //uint8 CHIPGEN_UBLOX5 = 1    # u-blox 5
                //uint8 CHIPGEN_UBLOX6 = 2    # u-blox 6
                //uint8 CHIPGEN_UBLOX7 = 3    # u-blox 7
                //uint8 CHIPGEN_UBLOX8 = 4    # u-blox 8 / u-blox M8
                uint16_t reserved2;
                NavSVInfo_SV satelites[1]; // first entry.
                //  NavSVInfo_SV[numch]
            });
// Description of flags bitfield
#define NAV_SVINFO_FLAGS_USED4NAV 0B00000001 // SV used in NAV sol
#define NAV_SVINFO_FLAGS_DGPS_AVAIL 0B00000010 // DGPS corr data available for SV
#define NAV_SVINFO_FLAGS_ORBIT_INFO_AVAIL 0B00000100 // Ephemeris of Almanac orbit info available for SV
#define NAV_SVINFO_FLAGS_EPHEMS_AVAIL 0B00001000 // Ephemeris orbit info available for SV
#define NAV_SVINFO_FLAGS_SV_UNHEALTHY 0B00010000 // SV unhealthy and not used
#define NAV_SVINFO_FLAGS_ALMPLUS_AVAIL 0B00100000 // Almanac Plus orbit info used
#define NAV_SVINFO_FLAGS_ASSNOW_AUTO 0B01000000 // AssistNow Autonomous orbit info used
#define NAV_SVINFO_FLAGS_PR_SMOOTHED 0B10000000 // Carrier Smoothed pseudorange used (PPP)


/*!
* NAV-TIMEGLO Message Structure
* GLONASS Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x23  Payload Length= 20 bytes
* Total = 6+20 = 26
*/

#define NAV_TIMEGLO_BUFSIZE (6+20+2)

    PACK(
            struct NavTimeGLO{
                UbloxHeader header;
                uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
                uint32_t TOD;      // GLONASS time of day (rounded to integer seconds)
                int32_t fTOD;      // Fractional part of TOD (range: +/-500000000). The precise GLONASS time of day in seconds is: TOD + fTOD * 1e-9
                uint16_t Nt;       // Current date (range: 1-1461), starting at 1 from the 1st Jan of the year indicated by N4 and ending at 1461 at the 31st Dec of the third year after that indicated by N4
                uint8_t N4;        // Four-year interval number starting from 1996 (1=1996, 2=2000, 3=2004...)
                uint8_t valid;     // Validity flags (see graphic below)
                uint32_t tAcc;     // Time Accuracy Estimate
                uint8_t checksum[2];
            });


/*!
* NAV-TIMEGPS Message Structure
* GPS Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x20  Payload Length= 16 bytes
* Total = 6+16 = 22
*/

#define NAV_TIMEGPS_BUFSIZE (6+16+2)

    PACK(
            struct NavTimeGPS{
                UbloxHeader header;
                uint32_t iTOW;  // GPS ms time of week
                int32_t ftow;   // fractional nanoseconds remainder f rounded
                                // # ms above, range -500000 .. 500000 [ns]
                int16_t week;   // GPS week
                int8_t leapsecs;// GPS UTC leap seconds [s]
                uint8_t valid;  // validity flags
                //uint8 VALID_TOW = 1        # Valid Time of Week
                //uint8 VALID_WEEK = 2       # Valid Week Number
                //uint8 VALID_LEAP_S = 4     # Valid Leap Seconds
                uint32_t tacc;  // time accuracy measurement (nanosecs)
                uint8_t checksum[2];
            });


/*!
* NAV-TIMEUTC Message Structure
* UTC Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x21  Payload Length= 20 bytes
* Total = 6+20 = 26
*/

#define NAV_TIMEUTC_BUFSIZE (6+20+2)

    PACK(
            struct NavTimeUTC{
                UbloxHeader header;
                uint32_t iTOW;  // GPS time of week (msec)
                uint32_t tacc;  // time accuracy measurement [ns]
                int32_t nano;   // Nanoseconds of second [ns]
                uint16_t year;  // year  1999..2099 (UTC) [y]
                uint8_t month;  // month 1..12 (UTC) [month]
                uint8_t day;    // day range 1..31 (UTC) [d]
                uint8_t hour;   // hour range 0..23 (UTC) [h]
                uint8_t min;    // minute range 0..59 (UTC) [min]
                uint8_t sec;    // second range 0..60 (UTC) [s] (60 for 
                                // # leap second)
                uint8_t valid;  // validity flags
                //uint8 VALID_TOW = 1         # Valid Time of Week
                //uint8 VALID_WKN = 2         # Valid Week Number
                //uint8 VALID_UTC = 4         # Valid Leap Seconds, i.e. Leap Seconds already known
                //uint8 VALID_UTC_STANDARD_MASK = 240  # UTC standard Identifier Bit mask:
                //uint8 UTC_STANDARD_NOT_AVAILABLE = 0    # Information not available
                //uint8 UTC_STANDARD_CRL = 16             # Communications Research Labratory
                //uint8 UTC_STANDARD_NIST = 32            # National Institute of Standards and 
                //                                        # Technology (NIST)
                //uint8 UTC_STANDARD_USNO = 48            # U.S. Naval Observatory (USNO)
                //uint8 UTC_STANDARD_BIPM = 64            # International Bureau of Weights and 
                //                                        # Measures (BIPM)
                //uint8 UTC_STANDARD_EL = 80              # European Laboratory (tbd)
                //uint8 UTC_STANDARD_SU = 96              # Former Soviet Union (SU)
                //uint8 UTC_STANDARD_NTSC = 112           # National Time Service Center, China
                //uint8 UTC_STANDARD_UNKNOWN = 240
            });


/*!
* NAV-VELNED Message Structure
* Velocity Solution in NED
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x12  Payload Length=36 bytes
*/

#define NAV_VELNED_BUFSIZE (6+36+2)

    PACK(
            struct NavVelNED{
                UbloxHeader header;     //!< Ublox header
                uint32_t iTOW;
                int32_t velocity_north; //!< north velocity [cm/s]
                int32_t velocity_east; //!< east velocity [cm/s]
                int32_t velocity_down; //!< down velocity [cm/s]
                uint32_t speed; //!< 3D speed [cm/s]
                uint32_t ground_speed; //!< 2D (ground) speed [cm/s]
                int32_t heading_scaled; //!< heading [deg]. Scaling 1e-5
                uint32_t speed_accuracy; //!< speed accuracy estimate [cm/s]
                uint32_t heading_accuracy; //!< course/heading accuracy estimate [deg]. Scaling 1e-5
                uint8_t checksum[2];
            });

