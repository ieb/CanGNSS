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
private:
    uint16_t hdop; // 0.01 units
    uint16_t pdop; // 0.01 units
    byte actualMode;

    void sendRapidPossitionUpdate(NavPosLLH *possition);
    void sendCOGSOG(NavVelNED *velned);
    void sendPossition(NavPVT *pvt);
    void sendMagneticVariation(NavPVT *pvt);
    void sendTimeUTC(NavPVT *pvt);
    void sendSatelitesInView(NavSat *sat);
    void sendDOP(NavDOP *dop);
    float calculateVariationRadians(NavPVT *pvt);
    uint16_t getDaysSince1970(NavPVT *pvt);
    float decimalYear(NavPVT *pvt);
    uint32_t getSecondsSinceMidnight(NavPVT *pvt);
    void outputBytes(uint8_t *p, uint8_t len);

};



