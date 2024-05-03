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

#define LOG_TAG "LOC_IDL_SERVICE"
#include <iostream>
#include <thread>
#include <stdio.h>
#include <functional>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>

#include <CommonAPI/CommonAPI.hpp>
#include "LocIdlAPIStubImpl.hpp"
#include "LocIdlAPIService.h"
#include "gptp_helper.h"

using namespace std;
using namespace v0::com::qualcomm::qti::location;
using namespace location_client;

static bool capabilitiesReceived = false;
static uint32_t numGnssLocationCb = 0;
static uint32_t numGnssDataCb = 0;
static uint32_t posCount = 0;
static uint32_t latentPosCount = 0;
/** Latency threshold for Position reports in msec */
#define MAX_POSITION_LATENCY   20
#define IDL_MAX_RETRY 5

/** Keep cont of number of start session requests, this variable
 *  shall be incremented on each startSession and decremented on
 *  every stop Session, Stop session request shall be sent to LCA
 *  only if this variable is 0 */
static uint32_t numControlRequests = 0;

static void onConfigResponseCb(location_integration::LocConfigTypeEnum      requestType,
                               location_integration::LocIntegrationResponse response) {
     LOC_LOGd("<<< onConfigResponseCb, type %d, err %d\n", requestType, response);
}

class LocationTrackingSessCbHandler {
    public:
        LocationTrackingSessCbHandler(const LocIdlAPIService *pClientApiService,
                const uint32_t &reportCbMask) {
            if (NULL != pClientApiService) {
                LOC_LOGd(" ==== LocationTrackingSessCbHandler --> ");
                memset(&mCallbackOptions, 0, sizeof(GnssReportCbs));
                if (reportCbMask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_LOC_CB_INFO_BIT) {
                    mCallbackOptions.gnssLocationCallback =
                            [pClientApiService] (const ::GnssLocation n) {
                        //Convert Location report from LCA to FIDL format
                        LocIdlAPI::IDLLocationReport idlLocRpt =
                                pClientApiService->parseLocationReport(n);
                        posCount++;
                        struct timespec curBootTime = {};
                        clock_gettime(CLOCK_BOOTTIME, &curBootTime);
                        int64_t curBootTimeNs = ((int64_t)curBootTime.tv_sec * 1000000000) +
                                (int64_t)curBootTime.tv_nsec;
                        int64_t latencyMs = (curBootTimeNs - n.elapsedRealTimeNs)/1000000;
                        if (latencyMs > MAX_POSITION_LATENCY) {
                              latentPosCount++;
                        }
                        if (posCount % 600 == 0) {
                            LOC_LOGe("%"PRId64" out of %"PRId64" Position samples are"
                                     " latent by 20 msec",
                                    latentPosCount, posCount);
                        }
                        pClientApiService->mService->fireLocationReportEvent(idlLocRpt);
                    };
                }
                if (reportCbMask &
                        LocIdlAPI::IDLGnssReportCbInfoMask::IDL_SV_CB_INFO_BIT) {
                    mCallbackOptions.gnssSvCallback =
                            [pClientApiService](const std::vector<::GnssSv>& gnssSvs) {
                        std::vector<LocIdlAPI::IDLGnssSv> idlSVReportVector;
                        LOC_LOGd("Number of SV's recevived -- %d", gnssSvs.size());
                        for (auto GnssSv : gnssSvs) {
                            //Convert Location report from LCA to FIDL format
                            LocIdlAPI::IDLGnssSv idlSVRpt =
                                pClientApiService->parseGnssSvReport(GnssSv);
                            idlSVReportVector.push_back(idlSVRpt);
                        }
                        pClientApiService->mService->fireGnssSvEvent(idlSVReportVector);
                    };
                }
                if (reportCbMask &
                        LocIdlAPI::IDLGnssReportCbInfoMask::IDL_NMEA_CB_INFO_BIT) {
                    mCallbackOptions.gnssNmeaCallback =
                            [pClientApiService](uint64_t timestamp, const std::string nmea) {
                        pClientApiService->mService->fireGnssNmeaEvent(timestamp, nmea);
                    };
                }
                if (reportCbMask &
                        LocIdlAPI::IDLGnssReportCbInfoMask::IDL_1HZ_MEAS_CB_INFO_BIT) {
                    mCallbackOptions.gnssMeasurementsCallback =
                            [pClientApiService](const ::GnssMeasurements gnssMeasurements) {
                        LocIdlAPI::IDLGnssMeasurements idlGnssMeasurement =
                                pClientApiService->parseGnssMeasurements(gnssMeasurements);
                        pClientApiService->mService->fireGnssMeasurementsEvent(idlGnssMeasurement);
                    };
                }
                 // GnssDataCb
                 if (reportCbMask & LocIdlAPI::IDLGnssReportCbInfoMask::IDL_DATA_CB_INFO_BIT) {
                     mCallbackOptions.gnssDataCallback =
                             [pClientApiService] (const ::GnssData n) {
                         numGnssDataCb++;
                         //Convert GnssData report from LCA to FIDL format
                         LocIdlAPI::IDLGnssData idlGnssDataRpt =
                                 pClientApiService->parseGnssDataReport(n);
                         pClientApiService->mService->fireGnssDataEvent(idlGnssDataRpt);
                     };
                 }
            }
        }

        LocationTrackingSessCbHandler(const LocIdlAPIService *pClientApiService,
                const uint32_t &locReqEngMask, const uint32_t &engReportCallbackMask) {
            if (NULL != pClientApiService) {
                memset(&mEngineCallbackOptions, 0, sizeof(EngineReportCbs));

                if (engReportCallbackMask &
                    LocIdlAPI::IDLEngineReportCbMask::IDL_ENGINE_LOCATION_CB_INFO_BIT) {
                    mEngineCallbackOptions.engLocationsCallback =
                            [pClientApiService] (const std::vector<::GnssLocation> &engLocations) {

                        std::vector<LocIdlAPI::IDLLocationReport> idlEngLocVector;
                        for (auto gnssLocation : engLocations) {
                            LocIdlAPI::IDLLocationReport idlLocRpt =
                                    pClientApiService->parseLocationReport(gnssLocation);
                            idlEngLocVector.push_back(idlLocRpt);
                        }
                        pClientApiService->mService->fireEngineLocationsEvent(idlEngLocVector);
                    };
                }

                //Fill LocReqEngMask
                memset(&mLcaLocReqEngMask, 0, sizeof(location_client::LocReqEngineTypeMask));
                uint32_t lcaLocReqEngMask = 0;
                if (locReqEngMask &
                        LocIdlAPI::IDLLocReqEngineTypeMask::IDL_LOC_REQ_ENG_FUSED) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_FUSED_BIT;
                }
                if (locReqEngMask & LocIdlAPI::IDLLocReqEngineTypeMask::IDL_LOC_REQ_ENG_SPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_SPE_BIT;
                }
                if (locReqEngMask & LocIdlAPI::IDLLocReqEngineTypeMask::IDL_LOC_REQ_ENG_PPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_PPE_BIT;
                }
                if (locReqEngMask & LocIdlAPI::IDLLocReqEngineTypeMask::IDL_LOC_REQ_ENG_VPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_VPE_BIT;
                }

                mLcaLocReqEngMask = (location_client::LocReqEngineTypeMask)lcaLocReqEngMask;
            }
        }

        GnssReportCbs& getLocationCbs() { return mCallbackOptions; }
        EngineReportCbs& getEngineLocationCbs() { return mEngineCallbackOptions; }
        location_client::LocReqEngineTypeMask& getLcaLocReqEngMask() { return mLcaLocReqEngMask; }

    private:
        GnssReportCbs mCallbackOptions;
        EngineReportCbs mEngineCallbackOptions;
        location_client::LocReqEngineTypeMask mLcaLocReqEngMask;

};

LocIdlAPI::IDLLocationReport LocIdlAPIService::parseLocationReport
(
    const location_client::GnssLocation &lcaLoc

) const {

    return (mLcaIdlConverter->parseLocReport(lcaLoc));

}

LocIdlAPI::IDLGnssSv LocIdlAPIService::parseGnssSvReport
(
    const location_client::GnssSv& gnssSvs
) const {
    return (mLcaIdlConverter->parseSvReport(gnssSvs));
}

LocIdlAPI::IDLGnssMeasurements LocIdlAPIService::parseGnssMeasurements
(
    const location_client::GnssMeasurements& gnssMeasurements
) const {

    return (mLcaIdlConverter->parseMeasurements(gnssMeasurements));

}

LocIdlAPI::IDLGnssData LocIdlAPIService::parseGnssDataReport
(
    const location_client::GnssData& gnssData
) const {
    return (mLcaIdlConverter->parseGnssData(gnssData));
}

LocIdlAPIService* LocIdlAPIService::mInstance = nullptr;
LocIdlAPIService* LocIdlAPIService::getInstance()
{
    if (nullptr == mInstance) {
        mInstance = new LocIdlAPIService();
    }
    return mInstance;
}

LocIdlAPIService::LocIdlAPIService():
        mLcaInstance(nullptr),
        mMsgTask(new MsgTask("LocIDLService")),
        mLIAInstance(nullptr),
#ifdef POWER_DAEMON_MGR_ENABLED
        mPowerEventObserver(nullptr),
#endif
        mLcaIdlConverter(new LocLcaIdlConverter()),
        mGnssReportMask(0)
{

}

LocIdlAPIService::~LocIdlAPIService()
{

}

void LocIdlAPIService::onPowerEvent(IDLPowerStateType powerEvent) {
    LOC_LOGi("Recieved Power Event %d", powerEvent);
    struct PowerEventMsg : public LocMsg {

         LocIdlAPIService* mIDLService;
        IDLPowerStateType mPowerEvent;
        inline PowerEventMsg(LocIdlAPIService* IDLService,
                IDLPowerStateType event) :
            LocMsg(),
            mIDLService(IDLService),
            mPowerEvent(event){};
        inline virtual void proc() const {
            switch (mPowerEvent) {
                 case IDL_POWER_STATE_SUSPEND:
                 case IDL_POWER_STATE_SHUTDOWN:
                     mIDLService->unRegisterWithFIDLService();
                     break;
                 case IDL_POWER_STATE_RESUME:
                     mIDLService->registerWithFIDLService();
                     break;
             }
        }
    };

    mMsgTask->sendMsg(new PowerEventMsg(this, powerEvent));


}
bool LocIdlAPIService::init()
{

    struct InitMsg : public LocMsg {
        LocIdlAPIService* mLCAService;
        inline InitMsg(LocIdlAPIService* LCAService) :
            LocMsg(),
            mLCAService(LCAService){};

        inline virtual void proc() const {
            mLCAService->createLocIdlService();
        }
    };

    mMsgTask->sendMsg(new InitMsg(this));
    return true;
}

bool LocIdlAPIService::createLocIdlService()
{
     LOC_LOGe("Initializing IDL Service ");
    // Create LCA object
    if (!mLcaInstance) {
        //Create capability callback
        CapabilitiesCb capabilitiesCb = [pClientApiService=this] (::LocationCapabilitiesMask mask) {
            LOC_LOGe("<<< onCapabilitiesCb mask=0x%" PRIx64 "", mask);
            LOC_LOGd("<<< onCapabilitiesCb mask string=%s ",
                    LocationClientApi::capabilitiesToString(mask).c_str());
            pClientApiService->processCapabilities(mask);
        };
        // Create LCA instance
        mLcaInstance = new LocationClientApi(capabilitiesCb);
        LOC_LOGi (" LCA instance created Successfully ");
    }

    if (nullptr == mLIAInstance) {
        LocConfigPriorityMap priorityMap;
        LocIntegrationCbs intCbs;
        intCbs.configCb = LocConfigCb(onConfigResponseCb);
        mLIAInstance = new LocationIntegrationApi(priorityMap, intCbs);
        LOC_LOGi (" LIA instance created Successfully ");
    }
#ifdef POWER_DAEMON_MGR_ENABLED
    if (nullptr == mPowerEventObserver && mInstance != NULL) {
        mPowerEventObserver = LocIdlPowerEvtHandler::getPwrEvtHandler(mInstance);
        if (nullptr == mPowerEventObserver) {
            LOC_LOGe(" mPowerEventObserver null !! ");
        }
    }
#endif
    registerWithFIDLService();

    return true;
}

bool LocIdlAPIService::processCapabilities(::LocationCapabilitiesMask mask)
{
    struct ProcessCapsMsg : public LocMsg {

        LocIdlAPIService* mLCAService;
        ::LocationCapabilitiesMask mMask;
        inline ProcessCapsMsg(LocIdlAPIService* LCAService,
                ::LocationCapabilitiesMask mask) :
            LocMsg(),
            mLCAService(LCAService),
            mMask(mask){};
        inline virtual void proc() const {
            //convert capabilities from LCA to FIDL format
            uint32_t idlCapsMask = 0;
            if (mMask & LOCATION_CAPS_TIME_BASED_TRACKING_BIT) {
                idlCapsMask |= LocIdlAPI::IDLLocationCapabilitiesMask::\
                        IDL_CAPS_TIME_BASED_TRACKING_BIT;
            }
            if (mMask & LOCATION_CAPS_GNSS_MEASUREMENTS_BIT) {
                idlCapsMask |= LocIdlAPI::IDLLocationCapabilitiesMask::IDL_CAPS_GNSS_MEAS_BIT;
            }
            //Update Capabilities to clients
            if (mLCAService->mService) {
                mLCAService->mService->fireGnssCapabilitiesMaskAttributeChanged(idlCapsMask);
            } else {
                LOC_LOGe("mLCAService->mService == NULL !! \n");
            }
        }
    };

    mMsgTask->sendMsg(new ProcessCapsMsg(this, mask));
    return true;
}

bool LocIdlAPIService::registerWithFIDLService()
{
    LOC_LOGe("Registering IDL Service ");

    CommonAPI::Runtime::setProperty("LogContext", "LOCIDL");
    CommonAPI::Runtime::setProperty("LogApplication", "LOCIDL");
    CommonAPI::Runtime::setProperty("LibraryBase", "LocIdlAPI");

    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    std::string domain = "local";
    std::string instance = "com.qualcomm.qti.location.LocIdlAPI";
    std::string connection = "location-fidl-service";

    mService = std::make_shared<LocIdlAPIStubImpl>(this);

    bool successfullyRegistered = runtime->registerService(domain, instance, mService, connection);

    while (!successfullyRegistered) {
        LOC_LOGe("Register IDL Service failed, trying again in 100 milliseconds !!");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        successfullyRegistered = runtime->registerService(domain, instance, mService, connection);
    }

    LOC_LOGd("Successfully Registered Service!");

    return true;
}

bool LocIdlAPIService::unRegisterWithFIDLService()
{
    LOC_LOGi("UnRegistering IDL Service ");

    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    std::string domain = "local";
    std::string instance = "com.qualcomm.qti.location.LocIdlAPI";
    std::string connection = "location-fidl-service";

    bool successfullyUnRegistered = runtime->unregisterService(domain,
            v0::com::qualcomm::qti::location::LocIdlAPI::getInterface(), instance);
    uint8_t count =  1;
    while (!successfullyUnRegistered && count <= IDL_MAX_RETRY) {
        LOC_LOGi("UnRegister IDL Service failed, No. of retires: %d !!", count);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        successfullyUnRegistered = runtime->unregisterService(domain,
                v0::com::qualcomm::qti::location::LocIdlAPI::getInterface(), instance);
        count++;
    }
    LOC_LOGi("Successfully UnRegistered Service!");
    return true;
}

LocIdlAPI::IDLLocationResponse LocIdlAPIService::parseIDLResponse(
        const location_client::LocationResponse lcaResponse) const {

    LocIdlAPI::IDLLocationResponse resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_UNKNOWN;
    switch (lcaResponse) {
        case LOCATION_RESPONSE_SUCCESS:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_SUCCESS;
            break;
        case LOCATION_RESPONSE_UNKOWN_FAILURE:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_UNKOWN_FAILURE;
            break;
        case LOCATION_RESPONSE_NOT_SUPPORTED:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_NOT_SUPPORTED;
            break;
        case LOCATION_RESPONSE_PARAM_INVALID:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_PARAM_INVALID;
            break;
        case LOCATION_RESPONSE_TIMEOUT:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_TIMEOUT;
            break;
        case LOCATION_RESPONSE_REQUEST_ALREADY_IN_PROGRESS:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_REQUEST_ALREADY_IN_PROGRESS;
            break;
        case LOCATION_RESPONSE_SYSTEM_NOT_READY:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_SYSTEM_NOT_READY;
            break;
        case LOCATION_RESPONSE_EXCLUSIVE_SESSION_IN_PROGRESS:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_EXCLUSIVE_SESSION_IN_PROGRESS;
            break;
        default:
            resp = LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_UNKNOWN;
    }
    return resp;
}

/* Process Fused/Detailed Position request */
void LocIdlAPIService::startPositionSession
(
        const std::shared_ptr<CommonAPI::ClientId> client,
        uint32_t intervalInMs, uint32_t gnssReportCallbackMask,
        LocIdlAPIStub::startPositionSessionReply_t reply
) const
{
    struct StartFusedPosMsg : public LocMsg {
        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        uint32_t mIntervalInMs;
        uint32_t mGnssReportCbMask;
        LocIdlAPIStub::startPositionSession1Reply_t mReply;
        inline StartFusedPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                uint32_t intervalInMs,
                uint32_t gnssReportCallbackMask,
                LocIdlAPIStub::startPositionSession1Reply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mIntervalInMs(intervalInMs),
            mGnssReportCbMask(gnssReportCallbackMask),
            mReply(reply){};
        inline virtual void proc() const {
            numControlRequests++;
            mLCAService->mGnssReportMask |= mGnssReportCbMask;

            LOC_LOGi("==== startPositionSession intervalMs %u GnssReportCbMask 0X%X"
                     " LCAReportMask 0X%X numControlRequests %u", mIntervalInMs, mGnssReportCbMask,
                     mLCAService->mGnssReportMask, numControlRequests);
            LocationTrackingSessCbHandler cbHandler(mLCAService, mLCAService->mGnssReportMask);
            ResponseCb rspCb = [client=mClient, reply=mReply] (::LocationResponse response) {
                LOC_LOGd("==== responseCb %d", response);
                //convert response from LCA to FIDL format
                LocIdlAPI::IDLLocationResponse resp = mInstance->parseIDLResponse(response);
                reply(resp);
            };

            mLCAService->mLcaInstance->startPositionSession(mIntervalInMs,
                    cbHandler.getLocationCbs(), rspCb);
        }
    };
    mMsgTask->sendMsg(new StartFusedPosMsg(this, client, intervalInMs,
            gnssReportCallbackMask, reply));
}

/* Process Engine specific Position request */
void LocIdlAPIService::startPositionSession
(
       const std::shared_ptr<CommonAPI::ClientId> client,
       uint32_t intervalInMs, uint32_t locReqEngMask, uint32_t engReportCallbackMask,
       LocIdlAPIStub::startPositionSession1Reply_t reply
) const
{
    struct StartPosMsg : public LocMsg {

        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        uint32_t mIntervalInMs;
        uint32_t mLocReqEngMask;
        uint32_t mEngReportCallbackMask;
        LocIdlAPIStub::startPositionSession1Reply_t mReply;
        inline StartPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                uint32_t intervalInMs,
                uint32_t locReqEngMask,
                uint32_t engReportCallbackMask,
                LocIdlAPIStub::startPositionSession1Reply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mIntervalInMs(intervalInMs),
            mLocReqEngMask(locReqEngMask),
            mEngReportCallbackMask(engReportCallbackMask),
            mReply(reply){};
        inline virtual void proc() const {
            numControlRequests++;
            LOC_LOGi("==== startPositionSession Engine Specific %u 0X%X 0X%X ",
                    mIntervalInMs, mLocReqEngMask, mEngReportCallbackMask);
            LocationTrackingSessCbHandler cbHandler(mLCAService, mLocReqEngMask,
                    mEngReportCallbackMask);
            ResponseCb rspCb = [client=mClient, reply=mReply] (::LocationResponse response) {
                LOC_LOGd("==== responseCb %d", response);
                //convert response from LCA to FIDL format
                LocIdlAPI::IDLLocationResponse resp = mInstance->parseIDLResponse(response);
                reply(resp);
            };

            mLCAService->mLcaInstance->startPositionSession(mIntervalInMs,
                    cbHandler.getLcaLocReqEngMask(),
                    cbHandler.getEngineLocationCbs(),
                    rspCb);

        }
    };

    mMsgTask->sendMsg(new StartPosMsg(this, client, intervalInMs,
            locReqEngMask, engReportCallbackMask, reply));
}

void LocIdlAPIService::stopPositionSession
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    LocIdlAPIStub::stopPositionSessionReply_t reply
) const
{
    struct StopPosMsg : public LocMsg {
        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        LocIdlAPIStub::stopPositionSessionReply_t mReply;
        inline StopPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                LocIdlAPIStub::stopPositionSessionReply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mReply(reply){};
        inline virtual void proc() const {
            numControlRequests--;
            if (!numControlRequests) {
                LOC_LOGd(" Sending STOP Session request !!");
                mLCAService->mLcaInstance->stopPositionSession();
                posCount = 0;
                latentPosCount = 0;
                mLCAService->mGnssReportMask = 0;
            }
            mReply();
       }
    };
    mMsgTask->sendMsg(new StopPosMsg(this, client, reply));
}

void LocIdlAPIService::LIAdeleteAidingData
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    uint32_t aidingDataMask,
    LocIdlAPIStub::deleteAidingDataReply_t reply
) const
{

    uint16_t mask = aidingDataMask;
    LOC_LOGd(" DeleteAssistance Mask recieved: %x ", mask);
    if (mLIAInstance) {
        bool ret = mLIAInstance->deleteAidingData((location_integration::\
                   AidingDataDeletionMask)mask);
        if (ret) {
            reply(LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_SUCCESS);
        } else {
            reply(LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_UNKOWN_FAILURE);
        }
    }
}

void LocIdlAPIService::LIAconfigConstellations
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    std::vector< LocIdlAPI::IDLGnssSvIdInfo > svListSrc,
    LocIdlAPIStub::configConstellationsReply_t reply
) const
{

    LOC_LOGd(" ");
    location_integration::LocConfigBlacklistedSvIdList svList;
    for (int i = 0; i < svListSrc.size(); i++) {
        location_integration::GnssSvIdInfo svIdConstellation = {};

        switch (svListSrc[i].getConstellation()) {
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_GLONASS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GLONASS;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_QZSS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_QZSS;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_BEIDOU:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_BEIDOU;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_GALILEO:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GALILEO;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_SBAS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_SBAS;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_NAVIC:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_NAVIC;
                break;
            case LocIdlAPI::IDLGnssConstellationType::IDL_GNSS_CONSTELLATION_TYPE_GPS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GPS;
                break;
            default:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_MAX;
       }
       svIdConstellation.svId = svListSrc[i].getSvId();
       svList.push_back(svIdConstellation);
    }
    if (mLIAInstance) {
        bool retVal = mLIAInstance->configConstellations(&svList);
        if (retVal) {
            reply(LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_SUCCESS);
        } else {
            reply(LocIdlAPI::IDLLocationResponse::IDL_LOC_RESP_UNKOWN_FAILURE);
        }
    }
}

int main() {
    LocIdlAPIService *pLCAService = LocIdlAPIService::getInstance();
    if (pLCAService) {
        pLCAService->init();
    }

    if (gptpInit()) {
        LOC_LOGd(" GPTP init success ");
    } else {
        LOC_LOGe(" GPTP init failed ");
    }

    // Waiting for calls
    int fd[2], n = 0;
    char buffer[10];
    if (pipe(fd) != -1) {
        n = read(fd[0], buffer, 10);
        if (n > 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return 0;
}
