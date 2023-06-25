
#include <Arduino.h>

#include "UBXMessages.h"
#include "GNSSReciever.h"
#include <XYZgeomag.hpp>
#include <time.h>

typedef union _opbuf {
    uint8_t buf[8];
    int64_t i64;
    int64_t u64;
    int32_t i32;
    uint32_t u32;
    int16_t i16;
    uint16_t u16;
} OpBuf;

void GNSSReciever::update(UbloxHeader *message) {
  switch(message->messageClass) {
    case MSG_CLASS_NAV:
          switch(message->messageId) {
            case MSG_ID_NAV_POSLLH:  // 34 bytes
                sendRapidPossitionUpdate((NavPosLLH *) message);
                break;
            case MSG_ID_NAV_VELNED: // 36 bytes
                sendCOGSOG((NavVelNED *) message);
                break;
            case MSG_ID_NAV_PVT: // 36 bytes
                // 129029L, // Position data 1Hz
                sendPossition((NavPVT *) message);
                // 127258L, // Magnetic Variation
                sendMagneticVariation((NavPVT *) message);
                // 126992L, // System Time, 1Hz
                sendTimeUTC((NavPVT *) message);
                break;
            case MSG_ID_NAV_DOP: // 28 bytes
                // 129539L, // GNSS DOPs 1Hz
                sendDOP((NavDOP *) message);
                break;
            case MSG_ID_NAV_SAT: // 28 bytes
                // 129540L, // GNSS Satellites in View
                sendSatelitesInView((NavSat *) message);
                break;
          }
        break;
    default:
        break;
  }    
}

void GNSSReciever::sendRapidPossitionUpdate(NavPosLLH *possition) {
    MessageHeader messageHeader(129025L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    // UBX PosLLM uses the same scales as NMEA2000 1e-7, no conversion required.
    outputBytes((uint8_t *)&(possition->latitude_scaled),4);   
    outputBytes((uint8_t *)&(possition->longitude_scaled),4);    
    finishPacket();
}

void GNSSReciever::sendCOGSOG(NavVelNED *velned) {
    MessageHeader messageHeader(129026L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(0xff&(velned->iTOW>>2));
    outputByte(0x00 | 0xfc); // 0x00= assuming true cog. M8N doesn't have a declination model.
    // cog needs to be in radians
    double cogRad = 1.74532925E-7*(velned->heading_scaled);
    output2ByteDouble(cogRad,0.001);
    // sog is in the correct units already
    output2ByteInt(velned->ground_speed);
    outputByte(0xff);
    outputByte(0xff);
    finishPacket();
}
void GNSSReciever::sendPossition(NavPVT *pvt) {
    if ( (pvt->valid&0x04) == 0x04 ) {
        // only send the possition if data is fully resolved.
        MessageHeader messageHeader(129029L, 3, getAddress(), SNMEA2000::broadcastAddress);
        startFastPacket(&messageHeader, 43);
        outputByte(0xff&(pvt->iTOW>>2));
        output2ByteUInt(getDaysSince1970(pvt));
        uint32_t secondsSinceMidnight = getSecondsSinceMidnight(pvt);
        outputBytes((uint8_t *)(&secondsSinceMidnight),4);
        OpBuf op;
        op.i64 = (int64_t)1E9*(int64_t)pvt->lat;  // 1E-16 units, 1E-16/1E-7=1E9
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E9*(int64_t)pvt->lon; // 1E-16 units, 1E-16/1E-7=1E9
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E3*(int64_t)pvt->height;  // 1E-6 units, 1E-6/1E-3 = 1E-3 
        outputBytes(&op.buf[0],8);  
        // type = GPS SBAS WAAS GLOLONASS (+Galeleo) 4 is closest
        // method == GNSS (0x10) if 3D Fix is available, otherwise no fix  (0x00)
        uint8_t methodType = 0x04;
        if ( pvt->fixType == 0x01 ) {
            // dead reconing
            methodType = 0x64;
            actualMode = 0; // 1D Fix
        } else if (pvt->fixType == 0x02 ) {
            // 2D fix.
            methodType = 0x14;
            actualMode = 1; // 2D Fix
        } else if (pvt->fixType == 0x03 ) {
            // 3D fix
            actualMode = 3; // 3D Fix
            if ( (pvt->flags&0x02) == 0x02 ) {
                // DGPS used
                methodType = 0x24;
            } else {
                methodType = 0x14;
            }
        }

        outputByte(methodType);
        outputByte(1 | 0xfc);  // Integrity 2 bit, reserved 6 bits
        outputByte(pvt->numSV);
        output2ByteInt(hdop);
        output2ByteInt(pdop);
        op.i32 = 0;
        outputBytes(&op.buf[0],4); // geoidal separation, set to 0.
        outputByte(0); // no reference stations as not using RTK mode
        finishFastPacket();
    } else {
        actualMode = 7; // fix not available.
    }
}
void GNSSReciever::sendMagneticVariation(NavPVT *pvt) {
    if ( (pvt->valid&0x04) == 0x04 ) {
        // need a fully resolved fix to calculate the variation.
        // variation is not available from standard precision Neo M8N modules.
        MessageHeader messageHeader(127258L, 6, getAddress(), SNMEA2000::broadcastAddress);
        outputByte(0xff&(pvt->iTOW>>2));
        outputByte(0x08); // WMN2020
        output2ByteUInt(getDaysSince1970(pvt));
        output2ByteDouble(calculateVariationRadians(pvt),0.0001);
        outputByte(0xff);
        outputByte(0xff);
        finishPacket();
    }
}

void GNSSReciever::sendTimeUTC(NavPVT *pvt) {
    if ( (pvt->valid&0x04) == 0x04 || (pvt->valid&0x02) == 0x02 ) {
        // date and time valid.
        MessageHeader messageHeader(126992L, 6, getAddress(), SNMEA2000::broadcastAddress);
        outputByte(0xff&(pvt->iTOW>>2));
        outputByte(0xf0); // 0=GPS source
        output2ByteUInt(getDaysSince1970(pvt));
        output2ByteDouble(calculateVariationRadians(pvt),0.0001);
        finishPacket();
    }
}

void GNSSReciever::sendSatelitesInView(NavSat *sat) {
    MessageHeader messageHeader(129029L, 3, getAddress(), SNMEA2000::broadcastAddress);
    startFastPacket(&messageHeader, 2+12*sat->numSvs);
    outputByte(0xff&(sat->iTOW>>2));
    // range residuals are after
    outputByte(0xfc | 0x01);
    outputByte(sat->numSvs);
    for (int i = 0; i < sat->numSvs; i++) {
        outputByte(sat->satelites[i].svId);
        output2ByteDouble(((float)M_PI/180)*sat->satelites[i].elev, 0.0001);
        output2ByteDouble(((float)M_PI/180)*sat->satelites[i].azim, 0.0001);
        output2ByteDouble(sat->satelites[i].cno, 0.01);
        output4ByteDouble(10.0*sat->satelites[i].prRes, 0.00001);
        uint8_t usage = 0x00;
        uint8_t quality = sat->satelites[i].flags&0x07; 
        bool hasDiff = ((sat->satelites[i].flags & 0x40) == 0x40);
        if ((sat->satelites[i].flags & 0x08) == 0x08 ) {
            // used.
            if ( (sat->satelites[i].flags & 0x40) == 0x40 ) {
                usage = 5; // with diff
            } else {
                usage = 2; // no diff
            }
        } else if ( quality > 6 ) {
            // being tracked
            if ( hasDiff ) {
                usage = 4; // tracked with diff, not used.
            } else {
                usage = 1; // tracked not used.
            }
        } else if ( hasDiff ) {
                usage = 3; // diff correction available
        }
        outputByte(0xf0 | usage);
    }
}


void GNSSReciever::sendDOP(NavDOP *dop) {
    //129539L
    MessageHeader messageHeader(129539L, 6, getAddress(), SNMEA2000::broadcastAddress);
    outputByte(0xff&(dop->iTOW>>2));

    // desired mode is 3D == 2, actualMode depends on last fix.
    outputByte(((2 & 0x07) << 5) | ((actualMode & 0x07) << 2));
    hdop = (int16_t)dop->hdop;
    output2ByteInt(hdop); // 0.01
    output2ByteInt((int16_t)dop->vdop); // 0.01
    output2ByteInt((int16_t)dop->tdop); // 0.01  
    finishPacket();
}

float GNSSReciever::calculateVariationRadians(NavPVT *pvt) {
    unsigned long now = millis();
    if ( now > lastVariationCalc + 60000) {
        float lat = 1.0E-7*(pvt->lat);
        float lon = 1.0E-7*(pvt->lon);
        float height = 0.0;     // sea level
        float dyear = decimalYear(pvt);
        geomag::Vector position = geomag::geodetic2ecef(lat,lon,height);
        geomag::Vector mag_field = geomag::GeoMag(dyear,position,geomag::WMM2020);
        geomag::Elements out = geomag::magField2Elements(mag_field, lat, lon);
        variation = ((float) M_PI/180.0) * out.declination;
    }
    return variation;
}

uint16_t GNSSReciever::getDaysSince1970(NavPVT *pvt) {
    struct tm posTime;
    posTime.tm_year = pvt->year - 1900;
    posTime.tm_mon = pvt->month - 1;
    posTime.tm_mday = pvt->day;
    posTime.tm_hour = pvt->hour;
    posTime.tm_min = pvt->min;
    posTime.tm_sec = pvt->sec; // leap seconds are supported.
    posTime.tm_isdst = 0;
    time_t t = mktime(&posTime);
    return (uint16_t)(t/86400);
}

float GNSSReciever::decimalYear(NavPVT *pvt) {
    struct tm posTime;
    posTime.tm_year = pvt->year - 1900;
    posTime.tm_mon = 0;
    posTime.tm_mday = 1;
    posTime.tm_hour = 0;
    posTime.tm_min = 0;
    posTime.tm_sec = 0; 
    posTime.tm_isdst = 0;
    time_t startOfYear = mktime(&posTime);
    posTime.tm_year = pvt->year - 1900;
    posTime.tm_mon = pvt->month - 1;
    posTime.tm_mday = pvt->day;
    posTime.tm_hour = pvt->hour;
    posTime.tm_min = pvt->min;
    posTime.tm_sec = pvt->sec; 
    posTime.tm_isdst = 0;
    time_t pvtTime = mktime(&posTime);
    return 1.0*pvt->year+difftime(pvtTime, startOfYear)/(365*86400);
}



uint32_t GNSSReciever::getSecondsSinceMidnight(NavPVT *pvt) {
    return pvt->hour*36000000 + 
    pvt->min * 600000 + 
    pvt->sec * 10000 + 
    pvt->nano/100000;
}

void GNSSReciever::outputBytes(uint8_t *p, uint8_t len) {
    for (int i = 0; i < len; i++) {
        outputByte(p[i]);
    }    
}

