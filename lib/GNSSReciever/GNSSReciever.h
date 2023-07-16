#pragma once

#include <Arduino.h>
#include "UBXMessages.h"
#include "SmallNMEA2000.h"

/*
enum eUsageStatus {
  usage_NotTracked=0,
  usage_TrackedNotUsed=1,
  usage_UsedWithoutDifferentialCorrections=2,
  usage_DifferentialCorrectionsAvailable=3,
  usage_TrackedWithDifferentialCorrections=4,
  usage_UsedWithDifferentialCorrections=5,
  usage_Error=14,
  usage_Unavailable=15,
}


enum eVariationSource {
    var_WMM2000=4,
    var_WMM2005=5,
    var_WMM2010=6,
    var_WMM2015=7,
    var_WMM2020=8,
};

enum eGNSSMode {
    gmssm_1D=0,
    gmssm_2D=1,
    gmssm_3D=2,
    gmssm_Auto=3,
    gmssm_Reserved=4,
    gmssm_Reserved2=5,
    gmssm_Error=6,
    gmssm_Unavailable=7
};


enum eGnssType {
    gnsst_GPS=0,
    gnsst_GLONASS=1,
    gnsst_GPSGLONASS=2,
    gnsst_GPSSBASWAAS=3,
    gnsst_GPSSBASWAASGLONASS=4,
    gnsst_Galileo=8
};
enum eGnssMethod {
    gnssm_noGNSS=0,
    gnssm_GNSSfix=1,
    gnssm_DGNSS=2,
    gnssm_Unavailable=15
};

typedef struct _GNSSStationReferences {
    eGnssType referenceStationType,
    uint16_t referenceStationID,
    double ageOfCorrection // in seconds.
} GNSSStationReferences;

typedef struct _GNSSSatelites {
  byte prn;
  double elevation;
  double azimuth;
  double snr;
  double rangeResiduals;
  eUsageStatus UsageStatus;
} GNSSSatelites;
*/


typedef struct _GNSSFix {
    uint8_t sid;
    uint8_t fixType;
    uint8_t methodType;
    uint8_t actualMode;
    uint8_t valid;
    uint8_t flags;
    uint8_t numSV;
    uint8_t numSvu;
    int16_t pdop;
    int16_t hdop;
    int16_t vdop;
    int16_t tdop;
    uint16_t daysSince1970;
    uint32_t secondsSinceMidnight;
    int32_t lat;
    int32_t lon;
    int32_t height;
    int32_t latitude_scaled;
    int32_t longitude_scaled;
    int32_t heading_scaled;
    uint32_t ground_speed;
    uint32_t heading_accuracy;
    uint32_t speed_accuracy;
    float variation;
} GNSSFix;

typedef struct _GNSSMetrics {
    uint16_t posllh = 0;
    uint16_t velned = 0;
    uint16_t pvt = 0;
    uint16_t dop = 0;
    uint16_t sat = 0;
    uint16_t unknown = 0;
} GNSSMetrics;


class GNSSReciever : public SNMEA2000 {
    public:
      GNSSReciever(byte addr,
        SNMEA2000DeviceInfo * devInfo, 
        const SNMEA2000ProductInfo * pinfo, 
        const SNMEA2000ConfigInfo * cinfo,
        const unsigned long *tx,
        const unsigned long *rx,
        const uint8_t csPin,
        Print *console
        ): SNMEA2000{addr, devInfo, pinfo, cinfo, tx, rx, csPin, console} {};

    void update(UbloxHeader *message);
    GNSSFix * getFix() {
        return &gnss;
    };
    GNSSMetrics * getMetrics() {
        return &metrics;
    }
private:
    GNSSFix gnss;
    GNSSMetrics metrics;
    bool cogSent = false;
    unsigned long lastVariationCalc = 0;
    void updateGnssFromPVT(NavPVT * pvt);
    void sendRapidPossitionUpdate();
    void sendCOGSOG();
    void sendPossition();
    void sendMagneticVariation();
    void sendTimeUTC();
    void sendSatelitesInView(NavSat *sat);
    void sendDOP();
    void calculateVariationDegrees(NavPVT *pvt);
    uint16_t getDaysSince1970(NavPVT *pvt);
    float decimalYear(NavPVT *pvt);
    uint32_t getSecondsSinceMidnight(NavPVT *pvt);
    void outputBytes(uint8_t *p, uint8_t len);

};



