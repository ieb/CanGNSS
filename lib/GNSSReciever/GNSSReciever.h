#pragma once



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



class SNMEA2000Device : public SNMEA2000 {
    public:
      SNMEA2000Device(byte addr,
        SNMEA2000DeviceInfo * devInfo, 
        const SNMEA2000ProductInfo * pinfo, 
        const SNMEA2000ConfigInfo * cinfo,
        const unsigned long *tx,
        const unsigned long *rx,
        const uint8_t csPin
        ): SNMEA2000{addr, devInfo, pinfo, cinfo, tx, rx, csPin} {};

    /**
     * 126992, // System Time, 1Hz
     */
    void sendSystemTime(byte sid, 
        uint16_t systemDate, 
        double systemTime);

    /**
     * 129025, // Position Rapid update 5Hz
     */
    void sendPositionRapidUpdate(double latitude, double longitude);
    /**
      * 129026, // COG/SOG Rapid Update 4Hz
     */
    void sendCOGSOGRapidUpdate(byte sid, 
        double cog, 
        double sog);
    /**
      * 129029, // Position data 1Hz
     */
    void sendPosstionData(byte sid, 
        uint16_t systemDate, 
        double systemTime, // seconds since midnight
        double latitude, 
        double longitude, 
        double altitude,
        eGnssMethod gnssMethod, 
        byte nSatelites, 
        double hdop, 
        double pdop, 
        eGnssType gnssType = gnsst_GPSSBASWAASGLONASS,
        byte nreference = 0, 
        GNSSStationReferences *refrences = NULL );
    /**
      * 129539, // GNSS DOPs 1Hz
     */
    void sendGNSSDOP(byte sid, 
        double hdop, 
        double pdop, 
        double tdop,
        eGNSSMode desiredMode, 
        eGNSSMode actualMode);
    /**
      * 129540, // GNSS Satellites in View
     */
    void sendGNSSSatelites(byte sid, 
        eRangeResidualMode rangeResidualMode, 
        byte nSatelitesInView, 
        GNSSSatelites *satelites);
    /**
      * 127258, // Magnetic Variation
     */
    void sendMagneticVariation(byte sid, 
        uint16_t age, 
        double variation,
        eVariationSource source);

};



