
#include <Arduino.h>

#include "UBXMessages.h"
#include "GNSSReciever.h"
#include <XYZgeomag.hpp>
#include <TimeLib.h>

typedef union _opbuf {
    uint8_t buf[8];
    int64_t i64;
    int64_t u64;
    int32_t i32;
    uint32_t u32;
    int16_t i16;
    uint16_t u16;
} OpBuf;

#define toSID(x)  (((x)/1000)%254)

extern void dumpMessage(UbloxHeader *message);
void GNSSReciever::update(UbloxHeader *message) {
  if ( message->messageClass == MSG_CLASS_NAV) {
    if (message->messageId == MSG_ID_NAV_POSLLH ) {  // 34 bytes
        NavPosLLH *possition = (NavPosLLH *) message;
        gnss.latitude_scaled = possition->latitude_scaled;
        gnss.longitude_scaled = possition->longitude_scaled;
        metrics.posllh++;
        sendRapidPossitionUpdate();
    } else if (message->messageId == MSG_ID_NAV_VELNED ) { // 36 bytes
        NavVelNED * velned = (NavVelNED *) message;
        gnss.sid = (((velned->iTOW)/500)%254); //2Hz rate. 
        gnss.heading_scaled = velned->heading_scaled;
        gnss.ground_speed = velned->ground_speed; // need in cm/s 
        metrics.velned++;
        cogSent = true;
        sendCOGSOG();
    } else if (message->messageId == MSG_ID_NAV_PVT ) {// 36 bytes
        metrics.pvt++;
        updateGnssFromPVT((NavPVT *) message);
        // 129029L, // Position data 1Hz
        sendPossition();
        // 127258L, // Magnetic Variation
        sendMagneticVariation();
        // 126992L, // System Time, 1Hz
        sendTimeUTC();
        if ( cogSent ) {
            cogSent = false;
        } else {
            sendCOGSOG();
        }
    } else if (message->messageId == MSG_ID_NAV_DOP ) { // 28 bytes
        metrics.dop++;
        NavDOP * dop = (NavDOP *) message;
        gnss.sid = toSID(dop->iTOW);
        gnss.hdop = (int16_t)dop->hdop;
        gnss.vdop = (int16_t)dop->vdop;
        gnss.tdop = (int16_t)dop->tdop;
        // 129539L, // GNSS DOPs 1Hz
        sendDOP();
    } else if (message->messageId == MSG_ID_NAV_SAT ) { // 28 bytes
        // 129540L, // GNSS Satellites in View
        metrics.sat++;
        sendSatelitesInView((NavSat *) message);
    } else {
        metrics.unknown++;
        console->print("N>?"); 
        console->println(message->messageId,HEX); 
    }
  } else if ( message->messageClass == MSG_CLASS_INF ) {
    // ignore, when these were decoded the str was not ASCII
    // always 11 chars long eg HEX 0 0 F9 BD 6C 0 6B 70 0 0 DD
    // not as per spec which says should be ASCII.
  } else {
    metrics.unknown++;
    console->print("?");
    console->print(message->messageClass,HEX); 
    console->print(">?"); 
    console->println(message->messageId,HEX); 
    
  }
}



void GNSSReciever::updateGnssFromPVT(NavPVT * pvt) {
    gnss.sid = toSID(pvt->iTOW);
    gnss.daysSince1970 = getDaysSince1970(pvt); // uint16_t
    gnss.secondsSinceMidnight = getSecondsSinceMidnight(pvt);  // uint32_t
    gnss.lat = pvt->lat;  // 1E-7 units
    gnss.lon = pvt->lon; // 1E-7 units
    gnss.height = pvt->hMSL; // in mm, height above mean sea level.
    gnss.fixType = pvt->fixType;
    calculateVariationDegrees(pvt);
    gnss.numSV = pvt->numSV;
    gnss.pdop = (int16_t)pvt->pDOP;    
    gnss.valid = pvt->valid;
    gnss.methodType = 0x04;
    gnss.heading_scaled = pvt->headMot; // deg 1E-5
    gnss.ground_speed = pvt->gSpeed/10; // cm/s

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
        gnss.actualMode = 2; // 3D Fix
        if ( (pvt->flags&0x02) == 0x02 ) {
            // DGPS used
            gnss.methodType = 0x24;
        } else {
            gnss.methodType = 0x14;
        }
    }
}

// {"timestamp":"2023-07-03-06:38:55.857","prio":2,"src":30,"dst":255,"pgn":129025,"description":"Position, Rapid Upda
// te","fields":{"Latitude":52.1879932,"Longitude": 0.1204546}}
// Ok
void GNSSReciever::sendRapidPossitionUpdate() {
    MessageHeader messageHeader(129025L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    // UBX PosLLM uses the same scales as NMEA2000 1e-7, no conversion required.
    outputBytes((uint8_t *)&(gnss.latitude_scaled),4);   
    outputBytes((uint8_t *)&(gnss.longitude_scaled),4);    
    finishPacket();
}

// {"timestamp":"2023-07-03-11:34:22.559","prio":2,"src":30,"dst":255,"pgn":129026,"description":"COG & SOG, Rapid Upd
// ate","fields":{"SID":64,"COG Reference":"True","COG":102.9,"SOG":0.01}}
// Ok
void GNSSReciever::sendCOGSOG() {
    MessageHeader messageHeader(129026L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(gnss.sid);
    outputByte(0x00 | 0xfc); // 0x00= assuming true cog. M8N doesn't have a declination model.
    // cog needs to be in radians
    // 10293041  == 102.93041 degrees.
    double cogRad = 1.74532925E-7*(gnss.heading_scaled);
    output2ByteDouble(cogRad,0.0001);
    // sog is in the correct units already
    output2ByteInt(gnss.ground_speed);
    outputByte(0xff);
    outputByte(0xff);
    finishPacket();

}
//{"timestamp":"2023-07-03-07:43:33.557","prio":3,"src":30,"dst":255,"pgn":129029,"description":"GNSS Position Data",
//"fields":{"SID":26,"Date":"2023.07.03","Time":"07:43:00.6320","Latitude":52.1880263,"Longitude": 0.1204592,"Altitud
//e":14.074000,"GNSS type":"GPS+SBAS/WAAS+GLONASS","Method":"GNSS fix","Integrity":"Safe","Number of SVs":17,"HDOP":0
//.66,"PDOP":1.03,"Geoidal Separation":0.00,"Reference Stations":0}}
// Ok
void GNSSReciever::sendPossition() {
    if ( (gnss.valid&0x04) == 0x04 ) {
        // only send the possition if data is fully resolved.
        MessageHeader messageHeader(129029L, 3, getAddress(), SNMEA2000::broadcastAddress);
        startFastPacket(&messageHeader, 43);
        outputByte(gnss.sid);
        output2ByteUInt(gnss.daysSince1970);
        outputBytes((uint8_t *)(&gnss.secondsSinceMidnight),4);
        OpBuf op;
        op.i64 = (int64_t)1E9*(int64_t)gnss.lat;  // measurement in 1E-7 deg required in 1E-16
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E9*(int64_t)gnss.lon; // measurement in 1E-7 deg required in 1E-16
        outputBytes(&op.buf[0],8);  
        op.i64 = (int64_t)1E3*(int64_t)gnss.height;  // measurement in 1E-3 required in 1E-6 m
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

// {"timestamp":"2023-07-03-07:43:33.558","prio":6,"src":30,"dst":255,"pgn":127258,"description":"Magnetic Variation",
// "fields":{"SID":26,"Source":"WMM 2020","Reserved":"00","Age of service":"2023.07.03","Variation":0.0}}
// Ok
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

//{"timestamp":"2023-07-03-07:43:33.559","prio":6,"src":30,"dst":255,"pgn":126992,"description":"System Time","fields
//":{"SID":26,"Source":"GPS","Date":"2023.07.03","Time":"00:00:00"}}
// Ok
void GNSSReciever::sendTimeUTC() {
    if ( (gnss.valid&0x04) == 0x04 || (gnss.valid&0x02) == 0x02 ) {
        // date and time valid.
        MessageHeader messageHeader(126992L, 6, getAddress(), SNMEA2000::broadcastAddress);
        startPacket(&messageHeader);
        outputByte(gnss.sid);
        outputByte(0xf0); // 0=GPS source
        output2ByteUInt(gnss.daysSince1970);
        outputBytes((uint8_t *)(&gnss.secondsSinceMidnight),4);
        finishPacket();
    }
}

/*
{"timestamp":"2023-07-03-11:34:22.765","prio":3,"src":30,"dst":255,"pgn":129540,"description":"GNSS Sats in View","
fields":{"SID":64,"Range Residual Mode":"Range residuals were calculated after the position","Sats in View":31,"lis
t":[{"PRN":1,"Elevation":44.0,"Azimuth":187.6,"SNR":36.00,"Range residuals":-25000000,"Status":"Used"},{"PRN":2,"El
evation":63.0,"Azimuth":187.6,"SNR":36.00,"Range residuals":13000000,"Status":"Used"},{"PRN":3,"Elevation":7.0,"Azi
muth":187.6,"SNR":21.00,"Range residuals":0,"Status":"Not tracked"},{"PRN":8,"Elevation":68.0,"Azimuth":164.0,"SNR"
:22.00,"Range residuals":-33000000,"Status":"Used"},{"PRN":10,"Elevation":39.0,"Azimuth":55.0,"SNR":19.00,"Range re
siduals":-13000000,"Status":"Used"},{"PRN":14,"Elevation":25.0,"Azimuth":187.6,"SNR":30.00,"Range residuals":360000
00,"Status":"Used"},{"PRN":21,"Elevation":74.0,"Azimuth":187.6,"SNR":35.00,"Range residuals":18000000,"Status":"Use
d"},{"PRN":23,"Elevation":5.0,"Azimuth":44.0,"SNR":10.00,"Range residuals":315000000,"Status":"Not tracked"},{"PRN"
:27,"Elevation":34.0,"Azimuth":139.0,"SNR":20.00,"Range residuals":-101000000,"Status":"Used"},{"PRN":32,"Elevation
":27.0,"Azimuth":104.0,"SNR":20.00}]}}
*/

//
void GNSSReciever::sendSatelitesInView(NavSat *sat) {
    gnss.numSvu = 0;
    for (int batch = 0; batch < sat->numSvs; batch = batch+17) {
        MessageHeader messageHeader(129540L, 6, getAddress(), SNMEA2000::broadcastAddress);
        uint8_t nsat = min(17, sat->numSvs - batch);
        startFastPacket(&messageHeader, 3+12*nsat);
        outputByte(toSID(sat->iTOW));
        // range residuals are after
        outputByte(0xfc | 0x01);
        outputByte(nsat);
        for (int i = batch; i < batch+nsat; i++) {
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
            outputByte(sat->satelites[i].svId);
            output2ByteDouble(((float)M_PI/180)*sat->satelites[i].elev, 0.0001);
            output2ByteDouble(((float)M_PI/180)*sat->satelites[i].azim, 0.0001);
            output2ByteDouble(sat->satelites[i].cno, 0.01);
            output4ByteDouble(10.0*sat->satelites[i].prRes, 0.00001);
            outputByte(0xf0 | usage);
        }
        finishFastPacket();
    }

}

//{"timestamp":"2023-07-03-06:38:54.838","prio":6,"src":30,"dst":255,"pgn":129539,"description":"GNSS DOPs","fields":
// {"SID":4,"Desired Mode":4,"Actual Mode":"2D","Reserved":"40","HDOP":0.63,"VDOP":0.85,"TDOP":0.53}}
// Desired mode and actual mode ???
void GNSSReciever::sendDOP() {
    //129539L
    MessageHeader messageHeader(129539L, 6, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(gnss.sid);
    outputByte(((0x02)) | ((gnss.actualMode & 0x07) << 3));
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
        geomag::Vector mag_field = geomag::GeoMag(dyear,position,geomag::WMM2020);
        geomag::Elements out = geomag::magField2Elements(mag_field, lat, lon);
        gnss.variation = out.declination;
    }
}

// Date incorrect.
// 8584  should be 19540
// date is being sent correctly, but calculated incorrectly

uint16_t GNSSReciever::getDaysSince1970(NavPVT *pvt) {
    TimeElements posTime;
    posTime.Year = pvt->year - 1970;
    posTime.Month = pvt->month;
    posTime.Day = pvt->day;
    posTime.Hour = pvt->hour;
    posTime.Minute = pvt->min;
    posTime.Second = pvt->sec; 
    time_t t = makeTime(posTime);
    return elapsedDays(t);
}

float GNSSReciever::decimalYear(NavPVT *pvt) {
    TimeElements posTime;
    posTime.Year = pvt->year - 1970;
    posTime.Month = 1;
    posTime.Day = 1;
    posTime.Hour = 0;
    posTime.Minute = 0;
    posTime.Second = 0; 
    time_t startOfYear = makeTime(posTime);
    posTime.Year = pvt->year - 1970;
    posTime.Month = pvt->month;
    posTime.Day = pvt->day;
    posTime.Hour = pvt->hour;
    posTime.Minute = pvt->min;
    posTime.Second = pvt->sec; 
    time_t pvtTime = makeTime(posTime);
    return 1.0*pvt->year+(pvtTime - startOfYear)/(365*86400);
}



uint32_t GNSSReciever::getSecondsSinceMidnight(NavPVT *pvt) {
    // must use unint32 to avoid truncation
    return  (uint32_t)(pvt->hour)*(uint32_t)36000000 
        + (uint32_t)(pvt->min) * (uint32_t)600000 
        + (uint32_t)(pvt->sec)*(uint32_t)10000 
        + (uint32_t)(pvt->nano)/(uint32_t)1E5;
}

void GNSSReciever::outputBytes(uint8_t *p, uint8_t len) {
    for (int i = 0; i < len; i++) {
        outputByte(p[i]);
    }    
}

