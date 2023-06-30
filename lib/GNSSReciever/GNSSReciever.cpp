
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

extern void dumpMessage(UbloxHeader *message);
void GNSSReciever::update(UbloxHeader *message) {
  if ( message->messageClass == MSG_CLASS_NAV) {
    if (message->messageId == MSG_ID_NAV_POSLLH ) {  // 34 bytes
        NavPosLLH *possition = (NavPosLLH *) message;
        gnss.latitude_scaled = possition->latitude_scaled;
        gnss.longitude_scaled = possition->longitude_scaled;
        sendRapidPossitionUpdate();
    } else if (message->messageId == MSG_ID_NAV_VELNED ) { // 36 bytes
        NavVelNED * velned = (NavVelNED *) message;
        gnss.sid = 0xff&(velned->iTOW>>2);
        gnss.heading_scaled = velned->heading_scaled;
        gnss.ground_speed = velned->ground_speed;
        sendCOGSOG();
    } else if (message->messageId == MSG_ID_NAV_PVT ) {// 36 bytes
        updateGnssFromPVT((NavPVT *) message);
        // 129029L, // Position data 1Hz
        sendPossition();
        // 127258L, // Magnetic Variation
        sendMagneticVariation();
        // 126992L, // System Time, 1Hz
        sendTimeUTC();
    } else if (message->messageId == MSG_ID_NAV_DOP ) { // 28 bytes
        NavDOP * dop = (NavDOP *) message;
        gnss.sid = 0xff&(dop->iTOW>>2);
        gnss.hdop = (int16_t)dop->hdop;
        gnss.vdop = (int16_t)dop->vdop;
        gnss.tdop = (int16_t)dop->tdop;
        // 129539L, // GNSS DOPs 1Hz
        sendDOP();
    } else if (message->messageId == MSG_ID_NAV_SAT ) { // 28 bytes
        // 129540L, // GNSS Satellites in View
        sendSatelitesInView((NavSat *) message);
    } else {
        console->print("N>?"); 
        console->println(message->messageId,HEX); 
    }
  } else if ( message->messageClass == MSG_CLASS_INF ) {
    // ignore, when these were decoded the str was not ASCII
    // always 11 chars long eg HEX 0 0 F9 BD 6C 0 6B 70 0 0 DD
    // not as per spec which says should be ASCII.
  } else {
    console->print("?");
    console->print(message->messageClass,HEX); 
    console->print(">?"); 
    console->println(message->messageId,HEX); 
    
  }
}



void GNSSReciever::updateGnssFromPVT(NavPVT * pvt) {
    gnss.sid = 0xff&(pvt->iTOW>>2);
    gnss.daysSince1970 = getDaysSince1970(pvt); // uint16_t
    gnss.secondsSinceMidnight = getSecondsSinceMidnight(pvt);  // uint32_t
    gnss.lat = pvt->lat;  // 1E-16 units, 1E-16/1E-7=1E9
    gnss.lon = pvt->lon; // 1E-16 units, 1E-16/1E-7=1E9
    gnss.height = pvt->height;
    gnss.fixType = pvt->fixType;
    calculateVariationDegrees(pvt);
    gnss.numSV = pvt->numSV;
    gnss.pdop = (int16_t)pvt->pDOP;    
    gnss.valid = pvt->valid;
    gnss.methodType = 0x04;
    if ( gnss.fixType == 0x01 ) {
        // dead reckoning
        gnss.methodType = 0x64;
        gnss.actualMode = 0; // 1D Fix
    } else if (gnss.fixType == 0x02 ) {
        // 2D fix.
        gnss.methodType = 0x14;
        gnss.actualMode = 1; // 2D Fix
    } else if (gnss.fixType == 0x03 ) {
        // 3D fix
        gnss.actualMode = 3; // 3D Fix
        if ( (pvt->flags&0x02) == 0x02 ) {
            // DGPS used
            gnss.methodType = 0x24;
        } else {
            gnss.methodType = 0x14;
        }
    }
}


void GNSSReciever::sendRapidPossitionUpdate() {
    MessageHeader messageHeader(129025L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    // UBX PosLLM uses the same scales as NMEA2000 1e-7, no conversion required.
    outputBytes((uint8_t *)&(gnss.latitude_scaled),4);   
    outputBytes((uint8_t *)&(gnss.longitude_scaled),4);    
    finishPacket();
}

void GNSSReciever::sendCOGSOG() {
    MessageHeader messageHeader(129026L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(gnss.sid);
    outputByte(0x00 | 0xfc); // 0x00= assuming true cog. M8N doesn't have a declination model.
    // cog needs to be in radians
    double cogRad = 1.74532925E-7*(gnss.heading_scaled);
    output2ByteDouble(cogRad,0.001);
    // sog is in the correct units already
    output2ByteInt(gnss.ground_speed);
    outputByte(0xff);
    outputByte(0xff);
    finishPacket();

}
void GNSSReciever::sendPossition() {
    if ( (gnss.valid&0x04) == 0x04 ) {
        // only send the possition if data is fully resolved.
        MessageHeader messageHeader(129029L, 3, getAddress(), SNMEA2000::broadcastAddress);
        startFastPacket(&messageHeader, 43);
        outputByte(gnss.sid);
        output2ByteUInt(gnss.daysSince1970);
        outputBytes((uint8_t *)(&gnss.secondsSinceMidnight),4);
        OpBuf op;
        op.i64 = (int64_t)1E9*(int64_t)gnss.lat;  // 1E-16 units, 1E-16/1E-7=1E9
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E9*(int64_t)gnss.lon; // 1E-16 units, 1E-16/1E-7=1E9
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E3*(int64_t)gnss.height;  // 1E-6 units, 1E-6/1E-3 = 1E-3 
        outputBytes(&op.buf[0],8);  
        // type = GPS SBAS WAAS GLOLONASS (+Galeleo) 4 is closest
        // method == GNSS (0x10) if 3D Fix is available, otherwise no fix  (0x00)

        outputByte(gnss.methodType);
        outputByte(1 | 0xfc);  // Integrity 2 bit, reserved 6 bits
        outputByte(gnss.numSV);
        output2ByteInt(gnss.hdop);
        output2ByteInt(gnss.pdop);
        op.i32 = 0;
        outputBytes(&op.buf[0],4); // geoidal separation, set to 0.
        outputByte(0); // no reference stations as not using RTK mode
        finishFastPacket();
    } else {
        gnss.actualMode = 7; // fix not available.
    }
}
void GNSSReciever::sendMagneticVariation() {
    if ( (gnss.valid&0x04) == 0x04 ) {
        // need a fully resolved fix to calculate the variation.
        // variation is not available from standard precision Neo M8N modules.
        MessageHeader messageHeader(127258L, 6, getAddress(), SNMEA2000::broadcastAddress);
        startPacket(&messageHeader);
        outputByte(gnss.sid);
        outputByte(0x08); // WMN2020
        output2ByteUInt(gnss.daysSince1970);
        output2ByteDouble(gnss.variation*0.01745329251,0.0001);
        outputByte(0xff);
        outputByte(0xff);
        finishPacket();
    }
}

void GNSSReciever::sendTimeUTC() {
    if ( (gnss.valid&0x04) == 0x04 || (gnss.valid&0x02) == 0x02 ) {
        // date and time valid.
        MessageHeader messageHeader(126992L, 6, getAddress(), SNMEA2000::broadcastAddress);
        startPacket(&messageHeader);
        outputByte(gnss.sid);
        outputByte(0xf0); // 0=GPS source
        output2ByteUInt(gnss.daysSince1970);
        output2ByteDouble(gnss.variation*0.01745329251,0.0001);
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
    gnss.numSvu = 0;
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
            gnss.numSvu++;
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
    finishFastPacket();
}


void GNSSReciever::sendDOP() {
    //129539L
    MessageHeader messageHeader(129539L, 6, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(gnss.sid);

    // desired mode is 3D == 2, actualMode depends on last fix.
    outputByte(((2 & 0x07) << 5) | ((gnss.actualMode & 0x07) << 2));
    output2ByteInt(gnss.hdop); // 0.01
    output2ByteInt(gnss.vdop); // 0.01
    output2ByteInt(gnss.tdop); // 0.01  
    finishPacket();
}

void GNSSReciever::calculateVariationDegrees(NavPVT *pvt) {
    unsigned long now = millis();
    if ( now > lastVariationCalc + 60000L) {
        lastVariationCalc = now;
        float lat = 1.0E-7*(pvt->lat);
        float lon = 1.0E-7*(pvt->lon);
        float height = 0.0;     // sea level
        float dyear = decimalYear(pvt);
        geomag::Vector position = geomag::geodetic2ecef(lat,lon,height);
        console->println(millis()-now);
        geomag::Vector mag_field = geomag::GeoMag(dyear,position,geomag::WMM2020);
        console->println(millis()-now);
        geomag::Elements out = geomag::magField2Elements(mag_field, lat, lon);
        console->println(millis()-now);
        gnss.variation = out.declination;
        console->print(F("Variation:"));
        console->print(gnss.variation);
        console->print(F(" ms:"));
        console->println(millis()-now);
    }
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

