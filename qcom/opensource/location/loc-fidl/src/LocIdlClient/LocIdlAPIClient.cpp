/*
Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <string>

#ifndef _WIN32
#include <unistd.h>
#endif
#include <chrono>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>
#include <CommonAPI/CommonAPI.hpp>
#include <v0/com/qualcomm/qti/location/LocIdlAPIProxy.hpp>
#include <time.h>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <signal.h>
#include <gptp_helper.h>
#include "log_util.h"

using namespace v0::com::qualcomm::qti::location;
using namespace std;
#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */

std::shared_ptr<LocIdlAPIProxy<>> myProxy;
bool verbose = false;

CommonAPI::CallInfo info(1000);
bool     sessionStarted;
uint32_t mask;
uint32_t pvtSubscription;
uint32_t svSubscription;
uint32_t nmeaSubscription;
uint32_t measSubscription;
uint32_t nHzmeasSubscription;
uint32_t dataSubscription;
static bool mIsGptpInitialized = false;

void ToolUsage()
{
    cout << " Usage : " << endl;
    cout << " LocIdlAPIClient -m <interested reports in decimal> -d "
                                            "<test duration in seconds> -v" << endl;
    cout << " -v represents verbose output " << endl;
    cout << " ===========================================" << endl<<endl;
    cout << " Example 1: No argument - deafult values will be used(-m 1 -d 60)" << endl;
    cout << " LocIdlAPIClient" <<endl;
    cout << " ===========================================" << endl<<endl;
    cout << " Example 2: For all the reports and test duration 300sec with verbose output" << endl;
    cout << " LocIdlAPIClient -m 31 -d 300 -v" <<endl<<endl;
    cout << " Example 3: For Position, SV reports and test duration 300sec " << endl;
    cout << " LocIdlAPIClient -m 3 -d 300" <<endl;
    cout << " Bit mask definition" << endl ;
    cout << " REPORT_NHZ_PVT    0x01   1" <<endl;
    cout << " REPORT_SV         0x02   2" <<endl;
    cout << " REPORT_NMEA       0x04   4" <<endl;
    cout << " REPORT_GNSSDATA   0x08   8" <<endl;
    cout << " REPORT_1HZ_MEAS   0x10   16" <<endl;
    cout << " ===========================================" << endl<<endl;
    return;
}

void printMeasurement(const LocIdlAPI::IDLGnssMeasurements& gnssMeasurements)
{
    static unsigned int measCount;
    uint64_t gptp_time_ns = 0;
    const LocIdlAPI::IDLGnssMeasurementsClock &clk = gnssMeasurements.getClock();
    static bool printMeasHeader = true;

    if (printMeasHeader) {
        cout << "Type, LeapSecond, TimeNs, TimeUncNs, No.Of SV" << endl;
        printMeasHeader = false;
    }

    measCount += 1;

    cout << "MEAS, " << clk.getLeapSecond()<<", "<< clk.getTimeNs()<< ""
        ", "<< clk.getTimeUncertaintyNs()<<", "
        "" << (gnssMeasurements.getMeasurements()).size()<< endl;

    if (verbose) {
        cout << "-------" << endl;
        cout << "Clk Flags     " << clk.getFlags() << endl;
        cout << "LeapSecond    " << clk.getLeapSecond() << endl;
        cout << "TimeNs        " << clk.getTimeNs() << endl;
        cout << "TimeUncNs     " << clk.getTimeUncertaintyNs() << endl;
        cout << "FullBiasNs    " << clk.getFullBiasNs() << endl;
        cout << "BiasNs        " << clk.getBiasNs() << endl;
        cout << "BiasUncNs     " << clk.getBiasUncertaintyNs() << endl;
        cout << "DriftNsps     " << clk.getDriftNsps() << endl;
        cout << "DriftUncNsps  " << clk.getDriftUncertaintyNsps() << endl;
        cout << "HwClockCount  " << clk.getHwClockDiscontinuityCount() << endl;

        const vector<LocIdlAPI::IDLGnssMeasurementsData > &measData =
                                        gnssMeasurements.getMeasurements();
        for (uint16_t idx = 0; idx < measData.size(); idx++) {
            cout <<"Idx  "<< idx << endl;

            cout <<"MeasFlags "<< measData[idx].getFlags() << endl;
            cout <<"svId "<< measData[idx].getSvType() << endl;

            cout <<"svType "<< measData[idx].getSvType() << endl;
            cout <<"timeOffsetNs"<< measData[idx].getTimeOffsetNs() << endl;
            cout <<"stateMask "<< measData[idx].getStateMask() << endl;
            cout <<"receivedSvTimeNs "<< measData[idx].getReceivedSvTimeNs() << endl;
            cout <<"receivedSvTimeSubNs "<< measData[idx].getReceivedSvTimeSubNs() << endl;
            cout <<"receivedSvTimeUncertaintyNs "
                    "" << measData[idx].getReceivedSvTimeUncertaintyNs() << endl;

            cout <<"carrierToNoiseDbHz "<< measData[idx].getCarrierToNoiseDbHz() << endl;
            cout <<"pseudorangeRateMps "<< measData[idx].getPseudorangeRateMps() << endl;
            cout <<"pseudorangeRateUncertaintyMps "
                    "" << measData[idx].getPseudorangeRateUncertaintyMps() << endl;
            cout <<"adrStateMask "<< measData[idx].getAdrStateMask() << endl;
            cout <<"adrMeters "<< measData[idx].getAdrMeters() << endl;
            cout <<"adrUncertaintyMeters "<< measData[idx].getAdrUncertaintyMeters() << endl;
            cout <<"carrierFrequencyHz "<< measData[idx].getCarrierFrequencyHz()  << endl;

            cout <<"carrierCycles "<< measData[idx].getCarrierCycles() << endl;
            cout <<"carrierPhase "<< measData[idx].getCarrierPhase() << endl;
            cout <<"carrierPhaseUncertainty "<< measData[idx].getCarrierPhaseUncertainty() << endl;
            cout <<"multipathIndicator "<< measData[idx].getMultipathIndicator() << endl;
            cout <<"signalToNoiseRatioDb "<< measData[idx].getSignalToNoiseRatioDb() << endl;
            cout <<"agcLevelDb "<< measData[idx].getAgcLevelDb() << endl;

            cout <<"basebandCarrierToNoiseDbHz "
                    "" << measData[idx].getBasebandCarrierToNoiseDbHz() << endl;
            cout <<"gnssSignalType "<< measData[idx].getGnssSignalType() << endl;
            cout <<"fullInterSignalBiasNs "<< measData[idx].getFullInterSignalBiasNs() << endl;
            cout <<"fullInterSignalBiasUncertaintyNs "
                    ""<< measData[idx].getFullInterSignalBiasUncertaintyNs() << endl;
            cout <<"cycleSlipCount "<< static_cast<int>(measData[idx].getCycleSlipCount()) << endl;
        }
        cout << "-------" << endl;
    }
}

void printPosResport(const LocIdlAPI::IDLLocationReport &_locationReport)
{
    const LocIdlAPI::IDLLocation &location = _locationReport.getLocInfo();

    static unsigned int posCount;
    uint64_t gptp_time_ns = 0;
    bool retPtp = false;
    static bool printPvtHeader = true;

    if (printPvtHeader) {
        cout << "Type, UTCTimestamp(ms), Latitude, Longitude, "
                        "RxTimeStampPTP(ns), TxTimestampPTP(ns), Latency(ms)" << endl;
        printPvtHeader = false;
    }

    posCount += 1;

    uint64_t lFlags = _locationReport.getLocationInfoFlags();
    if (lFlags &  LocIdlAPI::IDLLCALocationInfoFlagMask::IDL_LOC_INFO_GPTP_TIME_BIT) {
        retPtp = gptpGetCurPtpTime(&gptp_time_ns);
        if (retPtp) {
            cout <<"PVT, "
                "" << location.getTimestamp()<< ", "
                "" <<location.getLatitude() << ", " << location.getLongitude() << ", "
                "" << gptp_time_ns << ", "
                "" << _locationReport.getElapsedgPTPTime() <<", "
                "" << fixed << setprecision(3) << ""
                "" <<(float)(gptp_time_ns -
                            _locationReport.getElapsedgPTPTime()) / (float)1000000 <<""
                ""  << endl;
        } else {
            cout <<"PVT, "
                "" << location.getTimestamp()<< ", "
                "" <<location.getLatitude() << ", " << location.getLongitude() << ", "
                "" << "NA" << ", "
                "" << _locationReport.getElapsedgPTPTime() <<", "
                "" << fixed << setprecision(3) << ""
                "" <<"NA" <<""
                ""  << endl;
        }
    } else {
            cout <<"PVT, "
                "" << location.getTimestamp()<< ", "
                "" <<location.getLatitude() << ", " << location.getLongitude() << ", "
                "" << "NA" << ", "
                "" << "NA" <<", "
                "" << fixed << setprecision(3) << ""
                "" <<"NA" <<""
                ""  << endl;
    }

    if (verbose) {
        cout << "-------" << fixed << setprecision(8) << endl;
        cout << "TimeStamp      " << location.getTimestamp() << endl;
        cout << "Latitude       " << location.getLatitude() << endl;
        cout << "Longitude      " << location.getLongitude() << endl;
        cout << "Altitude       " << location.getAltitude() << endl;
        cout << "Speed          " << location.getSpeed() << endl;

        cout << "Bearing        " << location.getBearing() << endl;
        cout << "HorizontalAccuracy" << location.getHorizontalAccuracy() << endl;
        cout << "VerticalAccuracy  " << location.getVerticalAccuracy() << endl;
        cout << "SpeedAccuracy     " << location.getSpeedAccuracy() << endl;
        cout << "BearingAccuracy   " << location.getBearingAccuracy() << endl;

        cout << "TechMask          " << location.getTechMask() << endl;
        cout << "ElapsedRealTimeNs " << location.getElapsedRealTimeNs() << endl;
        cout << "ElapsedRealTimeUncNs " << location.getElapsedRealTimeUncNs() << endl;
        cout << "TimeUncMs         " << location.getTimeUncMs() << endl;

        cout << "Flags             " << location.getFlags() << endl;
        cout << "AltitudeMeanSeaLevel  "<< _locationReport.getAltitudeMeanSeaLevel() << endl;
        cout << "pDop             " << _locationReport.getPdop() << endl;
        cout << "HDop  " << _locationReport.getHdop() << endl;
        cout << "VDop  " << _locationReport.getVdop() << endl;
        cout << "GDop  " << _locationReport.getGdop() << endl;
        cout << "TDop  " << _locationReport.getTdop() << endl;

        cout << "MagneticDeviation       "<< _locationReport.getMagneticDeviation() << endl;
        cout << "HorReliability          "<< _locationReport.getHorReliability() << endl;
        cout << "VerReliability          "<< _locationReport.getVerReliability() << endl;
        cout << "HorUncEllipseSemiMajor  "<< _locationReport.getHorUncEllipseSemiMajor() << endl;
        cout << "HorUncEllipseSemiMinor  " << _locationReport.getHorUncEllipseSemiMinor() << endl;
        cout << "HorUncEllipseOrientAzimuth  "
                    "" << _locationReport.getHorUncEllipseOrientAzimuth() << endl;

        cout << "NorthStdDeviation       "<< _locationReport.getNorthStdDeviation() << endl;
        cout << "EastStdDeviation        "<< _locationReport.getEastStdDeviation() << endl;
        cout << "NorthVelocity           "<< _locationReport.getNorthVelocity() << endl;
        cout << "EastVelocity            "<< _locationReport.getEastVelocity() << endl;
        cout << "UpVelocity              "<< _locationReport.getUpVelocity() << endl;

        cout << "NorthVelocityStdDeviation"
            "" << _locationReport.getNorthVelocityStdDeviation() << endl;
        cout << "EastVelocityStdDeviation "
            "" << _locationReport.getEastVelocityStdDeviation() << endl;
        cout << "UpVelocityStdDeviatio    "<< _locationReport.getUpVelocityStdDeviation() << endl;
        cout << "NumSvUsedInPosition      " << _locationReport.getNumSvUsedInPosition() << endl;

        const LocIdlAPI::IDLLocationReportSvUsedInPosition &svUsed =
                                _locationReport.getSvUsedInPosition();
        cout << "GpsSvUsedIdsMask     "<< svUsed.getGpsSvUsedIdsMask() << endl;
        cout << "GalSvUsedIdsMask     "<< svUsed.getGalSvUsedIdsMask() << endl;
        cout << "BdsSvUsedIdsMask     "<< svUsed.getBdsSvUsedIdsMask() << endl;
        cout << "QzssSvUsedIdsMask    "<< svUsed.getQzssSvUsedIdsMask() << endl;
        cout << "NavicSvUsedIdsMask   "<< svUsed.getNavicSvUsedIdsMask() << endl;
        cout << "GloSvUsedIdsMask     "<< svUsed.getGloSvUsedIdsMask() << endl;


        cout << "NavSolutionMask      "<< _locationReport.getNavSolutionMask() << endl;
        cout << "PosTechMask          "<< _locationReport.getPosTechMask() << endl;

        const LocIdlAPI::IDLLocationReportPositionDynamics &posDynamics =
                                    _locationReport.getBodyFrameData();
        cout << "BodyFrameDataMask    "<< posDynamics.getBodyFrameDataMask() << endl;
        cout << "LongAccel            "<< posDynamics.getLongAccel() << endl;
        cout << "LatAccel             "<< posDynamics.getLatAccel() << endl;
        cout << "VertAccel            "<< posDynamics.getVertAccel() << endl;
        cout << "LongAccelUnc         "<< posDynamics.getLongAccelUnc() << endl;
        cout << "LatAccelUnc          "<< posDynamics.getLatAccelUnc() << endl;
        cout << "VertAccelUnc         "<< posDynamics.getVertAccelUnc() << endl;
        cout << "Pitch                "<< posDynamics.getPitch() << endl;
        cout << "PitchUnc             "<< posDynamics.getPitchUnc() << endl;
        cout << "PitchRateUnc         "<< posDynamics.getPitchRateUnc() << endl;
        cout << "PitchRate            "<< posDynamics.getPitchRate() << endl;
        cout << "Roll                 "<< posDynamics.getRoll() << endl;
        cout << "Roll Unc             "<< posDynamics.getRollUnc() << endl;
        cout << "Roll Rate            "<< posDynamics.getRollRate() << endl;
        cout << "Roll Rate Unc        "<< posDynamics.getRollRateUnc() << endl;
        cout << "Yaw                  "<< posDynamics.getYaw() << endl;
        cout << "YawUnc               "<< posDynamics.getYawUnc() << endl;
        cout << "YawRate              "<< posDynamics.getYawRate() << endl;
        cout << "YawRateUnc           "<< posDynamics.getYawRateUnc() << endl;

        const LocIdlAPI::IDLGnssSystemTime &gnssTime = _locationReport.getGnssSystemTime();
        cout << "getLocationCbEvent GnssSystemTimeSrc " << gnssTime.getGnssSystemTimeSrc() << endl;
        const LocIdlAPI::IDLSystemTimeStructUnion &time = gnssTime.getTimeUnion();
        if (time.isType<LocIdlAPI::IDLGnssSystemTimeStructType>()) {
              const LocIdlAPI::IDLGnssSystemTimeStructType &systemTime =
                                time.get<LocIdlAPI::IDLGnssSystemTimeStructType>();
              cout <<"SystemWeek      " <<systemTime.getSystemWeek() <<endl ;
              cout <<"SystemWeekMs    " <<systemTime.getSystemMsec() <<endl ;
              cout <<"SysClkTimeBias  " <<systemTime.getSystemClkTimeBias() <<endl ;
              cout <<"SysClkTimeUncMs " <<systemTime.getSystemClkTimeUncMs() <<endl ;
              cout <<"RefFCount       " <<systemTime.getRefFCount() <<endl ;
              cout <<"NumClockResets  " <<systemTime.getNumClockResets() <<endl ;
        }

        const vector<LocIdlAPI::IDLGnssMeasUsageInfo> &meas = _locationReport.getMeasUsageInfo();
        for (uint8_t idx = 0; idx < meas.size() && idx < 176; idx++) {
            cout << "GnssConstellation    "<< meas[idx].getGnssConstellation()<<endl ;
            cout << "GnssSignalType       "<< meas[idx].getGnssSignalType()<<endl ;
            cout << "GnssSvId             "<< meas[idx].getGnssSvId()<<endl ;
        }

        cout << "LeapSeconds      " << static_cast<int>(_locationReport.getLeapSeconds()) << endl;
        cout << "CalibrationConfidence "
            "" << static_cast<int>(_locationReport.getCalibrationConfidencePercent()) << endl;
        cout << "CalibrationStatus    " << _locationReport.getCalibrationStatus() << endl;
        cout << "LocOutputEngType     " << _locationReport.getLocOutputEngType() << endl;
        cout << "LocOutputEngMask     " << _locationReport.getLocOutputEngMask() << endl;
        cout << "ConformityIndex      " << _locationReport.getConformityIndex() << endl;

        const LocIdlAPI::IDLLLAInfo &lla = _locationReport.getLlaVRPBased();
        cout << "Latitude             " << lla.getLatitude() << endl;
        cout << "Longitude            " << lla.getLongitude() << endl;
        cout << "Altitude             " << lla.getAltitude() << endl;

        const vector<float> &emu = _locationReport.getEnuVelocityVRPBased();
        for (int k = 0; k < emu.size(); k++) {
            cout << "Emu                  " << emu[k] << endl;
        }

        cout << "DrSolutionStatusMask " << _locationReport.getDrSolutionStatusMask() << endl;
        cout << "AltitudeAssumed      " << _locationReport.getAltitudeAssumed() << endl;
        cout << "SessionStatus        " << _locationReport.getSessionStatus() << endl;
        cout << "IntegrityRiskUsed    " << _locationReport.getIntegrityRiskUsed() << endl;
        cout << "ProtectAlongTrack    " << _locationReport.getProtectAlongTrack() << endl;
        cout << "ProtectCrossTrack    " << _locationReport.getProtectCrossTrack() << endl;
        cout << "ProtectVertical      " << _locationReport.getProtectVertical() << endl;
        cout << "LocationInfoFlags    " << _locationReport.getLocationInfoFlags() << endl;

        const vector<uint16_t> &dgnss = _locationReport.getDgnssStationId();
        for (int idx = 0; idx < dgnss.size(); idx++) {
               cout << "DgnssStationId        " << dgnss[idx] << endl;
        }
        cout << "ElapsedPTPTimeNs  " << _locationReport.getElapsedgPTPTime() << endl;
        cout << "-------" << endl;
    }
}

void printGnssData(const LocIdlAPI::IDLGnssData& gnssData)
{
   vector<uint32_t> dataMask = gnssData.getGnssDataMask();
   vector<double> jammerInd = gnssData.getJammerInd();
   vector<double> agc = gnssData.getAgc();

   if (verbose) {
       cout << "Type, SignalType, Mask, JammeInd, Agc" << endl;
       cout << "-------" << endl;
       for (int i = 0; i < (dataMask.size() - 1); i++) {
           cout << "GNSSDATA, " << i << " , "<< dataMask[i] << " , "
                "" << jammerInd[i] << " , " << agc[i] << endl;
       }
       cout << "-------" << endl;
   }
}

void printSVInfo(const vector<LocIdlAPI::IDLGnssSv> &gnssSv)
{
    static unsigned int svCount;
    static bool printSVHeader = true;

    if (printSVHeader) {
        cout << "Type, No.of SV" << endl;
        printSVHeader = false;
    }
    svCount += 1;
    cout << "SV, " << gnssSv.size() << endl;


    if (verbose) {
        cout << "-------" << endl;
        for (uint16_t idx = 0; idx < gnssSv.size(); idx++) {
            cout << "svId  " << gnssSv[idx].getSvId() << endl;
            cout << "Type    " << gnssSv[idx].getType() << endl;
            cout << "CN0Dbhz  " << gnssSv[idx].getCN0Dbhz() << endl;
            cout << "setElevation    " << gnssSv[idx].getElevation() << endl;
            cout << "Azimuth  " << gnssSv[idx].getAzimuth() << endl;
            cout << "CarrierFrequencyHz    " << gnssSv[idx].getCarrierFrequencyHz() << endl;
            cout << "GnssSignalTypeMask" << gnssSv[idx].getGnssSignalTypeMask() << endl;
            cout << "BasebandCarrierToNoiseDbHz    "
                    "" << gnssSv[idx].getBasebandCarrierToNoiseDbHz() << endl;
            cout << "GloFrequency" << gnssSv[idx].getGloFrequency() << endl;
        }
        cout << "-------" << endl;
    }
}

void printNmea(const uint64_t timestamp, const string &nmea)
{
    static unsigned int nmeaCount;
    static bool printNmeaHeader = true;
    string segment;
    nmeaCount += 1;

    if (verbose) {
        cout << "NMEA, " << timestamp <<", "<< nmea<< endl;
    } else {
        if (printNmeaHeader) {
            cout << "Type, Timestamp, ID" << endl;
            printNmeaHeader = false;
        }
        stringstream stnmea(nmea);
        getline(stnmea, segment, ',');
        cout << "NMEA, "<< timestamp <<", "<< segment<< endl;
    }
}

void DeInitHandles()
{
    CommonAPI::CallStatus callStatus;

    if (sessionStarted) {
        if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_DATA_CB_INFO_BIT) {
            myProxy->getGnssDataEvent().unsubscribe(dataSubscription);
        }
        if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_LOC_CB_INFO_BIT) {
            myProxy->getLocationReportEvent().unsubscribe(pvtSubscription);
        }
        if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_1HZ_MEAS_CB_INFO_BIT) {
            myProxy->getGnssMeasurementsEvent().unsubscribe(measSubscription);
        }
        if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_SV_CB_INFO_BIT) {
            myProxy->getGnssSvEvent().unsubscribe(svSubscription);
        }
        if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_NMEA_CB_INFO_BIT) {
            myProxy->getGnssNmeaEvent().unsubscribe(nmeaSubscription);
        }

        myProxy->stopPositionSession(callStatus, &info);
    }
    if (mIsGptpInitialized) {
        mIsGptpInitialized = false;
        gptpDeinit();
    }
    usleep(5000);

}

void signalHandler(int signal) {
    cout << "signalHandler " <<endl;
    DeInitHandles();
    exit(0);
    return;
}

bool parseCommandLine(int argc, char* argv[], int &delay)
{
    extern char *optarg;
    int opt;
    bool flag = false;

    /*
    Valid mask values:
    REPORT_NHZ_PVT    0x01
    REPORT_SV         0x02
    REPORT_NMEA       0x04
    REPORT_GNSSDATA   0x08
    REPORT_1HZ_MEAS   0x10
    */
    /*PVT enabled by default */
    mask = 0x01;

    /*60sec / 1 min */
    delay = 60;

    if (argc > 1) {
        while ((opt = getopt(argc, argv,
                  "m:d:hv")) != -1) {
             switch (opt) {
                 case 'm':
                    mask = atoi(optarg);
                    flag = true;
                    break;
                 case 'd':
                    delay = atoi(optarg);
                    flag = true;
                    break;
                 case 'v':
                    verbose = true;
                    break;
                 case 'h':
                 default:
                     ToolUsage();
                     return false;
             }
        }
        if (!flag) {
             ToolUsage();
             return false;
        }
    }
    return true;
}

void regSigHandler()
{
    struct sigaction mySigAction = {};

    mySigAction.sa_handler = signalHandler;
    sigemptyset(&mySigAction.sa_mask);
    sigaction(SIGHUP, &mySigAction, NULL);
    sigaction(SIGTERM, &mySigAction, NULL);
    sigaction(SIGINT, &mySigAction, NULL);
    sigaction(SIGPIPE, &mySigAction, NULL);
}

void subscribeGnssResports()
{

    myProxy->getProxyStatusEvent().subscribe([&] (const CommonAPI::AvailabilityStatus status) {
        switch (status) {
        case CommonAPI::AvailabilityStatus::UNKNOWN:
            std::cout << "Unkown" << endl;
            break;
        case CommonAPI::AvailabilityStatus::NOT_AVAILABLE:
            std::cout << "NOT_AVAILABLE" << endl;
            break;
        case CommonAPI::AvailabilityStatus::AVAILABLE:
            std::cout << "AVAILABLE" << endl;
            break;
        }
    });
    // Subscribe for receiving values
    myProxy->getGnssCapabilitiesMaskAttribute().getChangedEvent().subscribe(
        [&](const uint32_t &val) {
                cout << "Received caps change event: " << val << endl;
            });

    if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_DATA_CB_INFO_BIT) {
        dataSubscription = myProxy->getGnssDataEvent().subscribe(
        [&](const LocIdlAPI::IDLGnssData& gnssData){
            printGnssData(gnssData);
        });
    }

    if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_LOC_CB_INFO_BIT) {
        pvtSubscription = myProxy->getLocationReportEvent().subscribe(
        [&](const LocIdlAPI::IDLLocationReport &_locationReport) {
            printPosResport(_locationReport);
        });
    }

    if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_1HZ_MEAS_CB_INFO_BIT) {
        measSubscription = myProxy->getGnssMeasurementsEvent().subscribe(
        [&](const LocIdlAPI::IDLGnssMeasurements& gnssMeasurements) {
            printMeasurement(gnssMeasurements);
        });
    }

    if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_SV_CB_INFO_BIT) {
        svSubscription = myProxy->getGnssSvEvent().subscribe(
        [&](const vector<LocIdlAPI::IDLGnssSv> &gnssSv) {
            printSVInfo(gnssSv);
        });
    }

    if (mask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_NMEA_CB_INFO_BIT) {
        nmeaSubscription = myProxy->getGnssNmeaEvent().subscribe(
        [&](const uint64_t timestamp, const string nmea){
            printNmea(timestamp, nmea);
        });
    }
}

void sessionStart()
{
    uint32_t _intervalInMs = 100;
    LocIdlAPI::IDLLocationResponse resp;
    CommonAPI::CallStatus callStatus;
    info.sender_ = 1234;

    sleep(1);
    myProxy->startPositionSession(_intervalInMs, mask, callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
        cout << "startPositionSession() Remote call failed! callStatus "
        "" << (int)callStatus << endl;
        sessionStarted = false;
    } else {
        sessionStarted = true;
    }
}

int main(int argc, char* argv[])
{
    int delay;

    /* Command Line parsing*/
    if (!parseCommandLine(argc, argv, delay))
        return -1;
    /* signal Handler */
    regSigHandler();

    /* GPTP */
    if (false == mIsGptpInitialized) {
        if (gptpInit()) {
            mIsGptpInitialized = true;
            LOC_LOGd(" GPTP initialization success ");
        } else {
            LOC_LOGe(" GPTP initialization failed ");
        }
    }

    CommonAPI::Runtime::setProperty("LogContext", "LocIdlAPI");
    CommonAPI::Runtime::setProperty("LogApplication", "LocIdlAPI");
    CommonAPI::Runtime::setProperty("LibraryBase", "LocIdlAPI");

    shared_ptr < CommonAPI::Runtime > runtime = CommonAPI::Runtime::get();

    string domain = "local";
    string instance = "com.qualcomm.qti.location.LocIdlAPI";
    string connection = "client-sample";

    myProxy = runtime->buildProxy<LocIdlAPIProxy>(domain,
            instance, connection);

    cout << "Checking availability!" << endl;
    while (!myProxy->isAvailable())
        usleep(10);
    cout << "Available..." << endl;

    subscribeGnssResports();
    sessionStart();
    if (sessionStarted)
        sleep(delay);
    DeInitHandles();
    return 0;
}
