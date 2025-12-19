/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "caBasicService.h"
#include "ns3/ItsPduHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/BitString.hpp"
#include "ns3/vdp.h"
#include "asn_utils.h"
#include <cmath>
#include "ns3/snr-tag.h"
#include "ns3/sinr-tag.h"
#include "ns3/rssi-tag.h"
#include "ns3/timestamp-tag.h"
#include "ns3/rsrp-tag.h"
#include "ns3/size-tag.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("CABasicService");

  CABasicService::~CABasicService() {
    NS_LOG_INFO("CABasicService object destroyed.");
  }

  CABasicService::CABasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket_tx=NULL;
    m_btp = NULL;
    m_real_time=false;

    // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
    m_T_CheckCamGen_ms=T_GenCamMin_ms;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCam_ms=T_GenCamMax_ms;

    lastCamGen=-1;
    lastCamGenLowFrequency=-1;
    lastCamGenSpecialVehicle=-1;

    // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
    m_N_GenCamMax=3;
    m_N_GenCam=0;

    m_vehicle=true;

    // CAM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenCam_ms=1000;

    m_cam_sent=0;

    // All the optional containers are disabled by default
    m_lowFreqContainerEnabled = false;
    m_specialVehContainerEnabled = false;

    m_CAReceiveCallback = nullptr;
    m_CAReceiveCallbackExtended = nullptr;

    m_refPositions.clear ();
  }

  CABasicService::CABasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp, bool real_time, bool is_vehicle)
  {
    m_socket_tx=NULL;
    m_btp = NULL;

    // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
    m_T_CheckCamGen_ms=T_GenCamMin_ms;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCam_ms=T_GenCamMax_ms;

    lastCamGen=-1;
    lastCamGenLowFrequency=-1;
    lastCamGenSpecialVehicle=-1;

    // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
    m_N_GenCamMax=3;
    m_N_GenCam=0;

    // CAM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenCam_ms=1000;

    m_cam_sent=0;

    // All the optional containers are disabled by default
    m_lowFreqContainerEnabled = false;
    m_specialVehContainerEnabled = false;

    m_CAReceiveCallback = nullptr;
    m_CAReceiveCallbackExtended = nullptr;

    m_refPositions.clear ();

    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;

    m_vdp=vdp;
    m_real_time=real_time;

    m_vehicle=is_vehicle;
  }

  CABasicService::CABasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp, bool real_time, bool is_vehicle, Ptr<Socket> socket_tx)
  {
    CABasicService(fixed_stationid,fixed_stationtype,vdp,real_time,is_vehicle);

    m_socket_tx=socket_tx;
  }

  void
  CABasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
  }

  void
  CABasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
  {
    m_vehicle = false;
    m_RSUlon = longitude_deg;
    m_RSUlat = latitude_deg;
    //High frequency RSU container
    m_protectedCommunicationsZonesRSU = asn1cpp::makeSeq(RSUContainerHighFrequency);
    auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
    asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
    asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
    asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);
    asn1cpp::sequenceof::pushList(m_protectedCommunicationsZonesRSU->protectedCommunicationZonesRSU,protectedComm);
    m_btp->setFixedPositionRSU(latitude_deg,longitude_deg);
  }

  void
  CABasicService::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
    m_btp->setStationID(fixed_stationid);
  }

  void
  CABasicService::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
  }

  void
  CABasicService::setSocketRx (Ptr<Socket> socket_rx)
  {
    m_btp->setSocketRx(socket_rx);
    m_btp->addCAMRxCallback (std::bind(&CABasicService::receiveCam,this,std::placeholders::_1,std::placeholders::_2));
  }

  void
  CABasicService::startCamDissemination()
  {
    if(m_vehicle)
      {
        m_event_camDisseminationStart = Simulator::Schedule (Seconds(0), &CABasicService::initDissemination, this);
      }
    else
      {
        m_event_camDisseminationStart = Simulator::Schedule (Seconds (0), &CABasicService::RSUDissemination, this);
      }
  }

  void
  CABasicService::startCamDissemination(double desync_s)
  {
    if(m_vehicle)
      {
        m_event_camDisseminationStart = Simulator::Schedule (Seconds (desync_s), &CABasicService::initDissemination, this);
      }
    else
      {
        m_event_camDisseminationStart = Simulator::Schedule (Seconds (desync_s), &CABasicService::RSUDissemination, this);
      }
  }

  void
  CABasicService::receiveCam (BTPDataIndication_t dataIndication, Address from)
  {
    Ptr<Packet> packet;
    asn1cpp::Seq<CAM> decoded_cam;

    uint8_t *buffer; //= new uint8_t[packet->GetSize ()];
    buffer=(uint8_t *)malloc((dataIndication.data->GetSize ())*sizeof(uint8_t));
    dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
    std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());

    RssiTag rssi;
    bool rssi_result = dataIndication.data->PeekPacketTag(rssi);

    SnrTag snr;
    bool snr_result = dataIndication.data->PeekPacketTag(snr);

    RsrpTag rsrp;
    bool rsrp_result = dataIndication.data->PeekPacketTag(rsrp);

    SinrTag sinr;
    bool sinr_result = dataIndication.data->PeekPacketTag(sinr);

    SizeTag size;
    bool size_result = dataIndication.data->PeekPacketTag(size);

    TimestampTag timestamp;
    dataIndication.data->PeekPacketTag(timestamp);

    if(!snr_result)
      {
        snr.Set(SENTINEL_VALUE);
      }
    if (!rssi_result)
      {
        rssi.Set(SENTINEL_VALUE);
      }
    if (!rsrp_result)
      {
        rsrp.Set(SENTINEL_VALUE);
      }
    if (!sinr_result)
      {
        sinr.Set(SENTINEL_VALUE);
      }
    if (!size_result)
      {
        size.Set(SENTINEL_VALUE);
      }

    SetSignalInfo(timestamp.Get(), size.Get(), rssi.Get(), snr.Get(), sinr.Get(), rsrp.Get());

    /* Try to check if the received packet is really a CAM */
    if (buffer[1]!=FIX_CAMID)
      {
        NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '2' was expected.");
        free(buffer);
        return;
      }

    free(buffer);

    /** Decoding **/
    decoded_cam = asn1cpp::uper::decodeASN(packetContent, CAM);

    if(bool(decoded_cam)==false) {
        NS_LOG_ERROR("Warning: unable to decode a received CAM.");
        return;
      }

    if(m_LDM != NULL){
      //Update LDM
      vLDM_handler(decoded_cam);
    }

    if(m_CAReceiveCallback!=nullptr) {
      m_CAReceiveCallback(decoded_cam,from);
    } else if(m_CAReceiveCallbackExtended!=nullptr) {
      m_CAReceiveCallbackExtended(decoded_cam,from,m_station_id,m_stationtype,GetSignalInfo());
    }
  }

  void
  CABasicService::vLDM_handler(asn1cpp::Seq<CAM> decodedCAM)
  {
      vehicleData_t vehdata;
      LDM::LDM_error_t db_retval;
      bool lowFreq_ok;
      vehdata.detected = false;
      vehdata.itsType = itsType_vehicle;
      vehdata.stationType = asn1cpp::getField(decodedCAM->cam.camParameters.basicContainer.stationType,long);
      vehdata.stationID = asn1cpp::getField(decodedCAM->header.stationId,uint64_t);
      vehdata.lat = asn1cpp::getField(decodedCAM->cam.camParameters.basicContainer.referencePosition.latitude,double)/(double)DOT_ONE_MICRO;
      vehdata.lon = asn1cpp::getField(decodedCAM->cam.camParameters.basicContainer.referencePosition.longitude,double)/(double)DOT_ONE_MICRO;
      vehdata.elevation = asn1cpp::getField(decodedCAM->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue,double)/(double)CENTI;
      vehdata.heading = asn1cpp::getField(decodedCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,double)/(double)DECI;
      vehdata.speed_ms = asn1cpp::getField(decodedCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,double)/(double)CENTI;
      vehdata.camTimestamp = asn1cpp::getField(decodedCAM->cam.generationDeltaTime,long);
      vehdata.timestamp_us = Simulator::Now ().GetMicroSeconds ();
      vehdata.VAM = false;

      vehdata.vehicleWidth = OptionalDataItem<long>(asn1cpp::getField(decodedCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth,long));
      if(vehdata.vehicleWidth.getData() == VehicleWidth_unavailable)
          vehdata.vehicleWidth = OptionalDataItem<long>((long) 10);
      vehdata.vehicleLength = OptionalDataItem<long>(asn1cpp::getField(decodedCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue,long));


      auto lowFreqContainer = asn1cpp::getSeqOpt(decodedCAM->cam.camParameters.lowFrequencyContainer,LowFrequencyContainer,&lowFreq_ok);
      if(lowFreq_ok)
      {
          vehdata.exteriorLights = OptionalDataItem<uint8_t>(asn1cpp::bitstring::getterByteMask(lowFreqContainer->choice.basicVehicleContainerLowFrequency.exteriorLights,0));
      }
      else
      {
          LDM::returnedVehicleData_t retveh;

         if(m_LDM->lookup(vehdata.stationID,retveh) == LDM::LDM_OK){
             vehdata.exteriorLights = retveh.vehData.exteriorLights;
         }
         else{
             vehdata.exteriorLights = OptionalDataItem<uint8_t>(false);
         }
      }

      db_retval=m_LDM->insert(vehdata);
      if(db_retval!=LDM::LDM_OK && db_retval!=LDM::LDM_UPDATED) {
          std::cerr << "Warning! Insert on the database for vehicle " << asn1cpp::getField(decodedCAM->header.stationId,int) << "failed!" << std::endl;
      }
  }

  void
  CABasicService::initDissemination()
  {
    generateAndEncodeCam();
    m_event_camCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions, this);
  }

  void
  CABasicService::RSUDissemination()
  {
    generateAndEncodeCam();
    m_event_camRsuDissemination = Simulator::Schedule (MilliSeconds(m_RSU_GenCam_ms), &CABasicService::RSUDissemination, this);
  }

  void
  CABasicService::checkCamConditions()
  {
    int64_t now=computeTimestampUInt64 ()/NANO_TO_MILLI;
    CABasicService_error_t cam_error;
    bool condition_verified=false;
    static bool dyn_cond_verified=false;

    // If no initial CAM has been triggered before checkCamConditions() has been called, throw an error
    if(m_prev_heading==-1 || m_prev_speed==-1 || m_prev_distance==-1)
      {
        NS_FATAL_ERROR("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
      }
    /*
     * ETSI EN 302 637-2 V1.3.1 chap. 6.1.3 condition 1) (no DCC)
     * One of the following ITS-S dynamics related conditions is given:
    */

    /* 1a)
     * The absolute difference between the current heading of the originating
     * ITS-S and the heading included in the CAM previously transmitted by the
     * originating ITS-S exceeds 4°;
    */
    double head_diff = m_vdp->getHeadingValue () - m_prev_heading;
    head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;
    if (head_diff > 4.0 || head_diff < -4.0)
      {
        cam_error=generateAndEncodeCam ();
        if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
          } else {
            NS_LOG_ERROR("Cannot generate CAM. Error code: "<<cam_error);
          }
      }

    /* 1b)
     * the distance between the current position of the originating ITS-S and
     * the position included in the CAM previously transmitted by the originating
     * ITS-S exceeds 4 m;
    */
    double pos_diff = m_vdp->getTravelledDistance () - m_prev_distance;
    if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
      {
        cam_error=generateAndEncodeCam ();
        if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
          } else {
            NS_LOG_ERROR("Cannot generate CAM. Error code: "<<cam_error);
          }
      }

    /* 1c)
     * he absolute difference between the current speed of the originating ITS-S
     * and the speed included in the CAM previously transmitted by the originating
     * ITS-S exceeds 0,5 m/s.
    */
    double speed_diff = m_vdp->getSpeedValue () - m_prev_speed;
    if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
      {
        cam_error=generateAndEncodeCam ();
        if(cam_error==CAM_NO_ERROR)
          {
            m_N_GenCam=0;
            condition_verified=true;
            dyn_cond_verified=true;
          } else {
            NS_LOG_ERROR("Cannot generate CAM. Error code: "<<cam_error);
          }
      }

    /* 2)
     * The time elapsed since the last CAM generation is equal to or greater than T_GenCam
    */
    if(!condition_verified && (now-lastCamGen>=m_T_GenCam_ms))
      {
         cam_error=generateAndEncodeCam ();
         if(cam_error==CAM_NO_ERROR)
           {

             if(dyn_cond_verified==true)
               {
                 m_N_GenCam++;
                 if(m_N_GenCam>=m_N_GenCamMax)
                   {
                     m_N_GenCam=0;
                     m_T_GenCam_ms=T_GenCamMax_ms;
                     dyn_cond_verified=false;
                   }
               }
           } else {
             NS_LOG_ERROR("Cannot generate CAM. Error code: "<<cam_error);
           }
      }

    m_event_camCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions, this);
  }

  CABasicService_error_t
  CABasicService::generateAndEncodeCam()
  {
    VDP::CAM_mandatory_data_t cam_mandatory_data;
    CABasicService_error_t errval=CAM_NO_ERROR;

    Ptr<Packet> packet;

    BTPDataRequest_t dataRequest = {};

    int64_t now,now_centi;

    /* Collect data for mandatory containers */
    auto cam = asn1cpp::makeSeq(CAM);

    if(bool(cam)==false)
      {
        return CAM_ALLOC_ERROR;
      }

    /* Fill the header */
    asn1cpp::setField(cam->header.messageId, FIX_CAMID);
    asn1cpp::setField(cam->header.protocolVersion , 2);
    asn1cpp::setField(cam->header.stationId, m_station_id);

    /*
     * Compute the generationDeltaTime, "computed as the time corresponding to the
     * time of the reference position in the CAM, considered as time of the CAM generation.
     * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
     * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
     * generationDeltaTime = TimestampIts mod 65 536"
    */
    asn1cpp::setField(cam->cam.generationDeltaTime, compute_timestampIts (m_real_time) % 65536);

    /* Fill the basicContainer's station type */
    asn1cpp::setField(cam->cam.camParameters.basicContainer.stationType, m_stationtype);
    if(m_vehicle==true)
      {

        cam_mandatory_data=m_vdp->getCAMMandatoryData();

        /* Fill the basicContainer */
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue, cam_mandatory_data.altitude.getValue ());
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence, cam_mandatory_data.altitude.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.latitude, cam_mandatory_data.latitude);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.longitude, cam_mandatory_data.longitude);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength, cam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength, cam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation, cam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);

        /* Fill the highFrequencyContainer */

        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present, HighFrequencyContainer_PR_basicVehicleContainerHighFrequency);
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue, cam_mandatory_data.heading.getValue ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence, cam_mandatory_data.heading.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue, cam_mandatory_data.speed.getValue ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence, cam_mandatory_data.speed.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection, cam_mandatory_data.driveDirection);
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue, cam_mandatory_data.VehicleLength.getValue());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication, cam_mandatory_data.VehicleLength.getConfidence());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth, cam_mandatory_data.VehicleWidth);
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value, cam_mandatory_data.longAcceleration.getValue ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.confidence, cam_mandatory_data.longAcceleration.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue, cam_mandatory_data.curvature.getValue ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence, cam_mandatory_data.curvature.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode, cam_mandatory_data.curvature_calculation_mode);
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue, cam_mandatory_data.yawRate.getValue ());
        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence, cam_mandatory_data.yawRate.getConfidence ());

        // Manage optional data
        auto accControl = m_vdp->getAccelerationControl ();
        if(accControl.isAvailable ()) {
            asn1cpp::bitstring::setBit(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl,setByteMask(accControl.getData ()),0);
        }

        auto lanePosition = m_vdp->getLanePosition ();
        if(lanePosition.isAvailable ()) {
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition,lanePosition.getData ());
        }

        auto swa = m_vdp->getSteeringWheelAngle();
        if(swa.isAvailable()) {
            auto swa_seq = asn1cpp::makeSeq(SteeringWheelAngle);
            asn1cpp::setField(swa_seq->steeringWheelAngleValue,swa.getData ().getValue ());
            asn1cpp::setField(swa_seq->steeringWheelAngleConfidence,swa.getData ().getConfidence ());
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle,swa_seq);
        }

        auto latacc = m_vdp->getLateralAcceleration();
        if(latacc.isAvailable()) {
            auto latacc_seq = asn1cpp::makeSeq(AccelerationComponent);
            asn1cpp::setField(latacc_seq->value,latacc.getData ().getValue ());
            asn1cpp::setField(latacc_seq->confidence,latacc.getData ().getConfidence ());
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration,latacc_seq);
        }


        auto vertacc = m_vdp->getVerticalAcceleration();
        if(vertacc.isAvailable()) {
            auto vertacc_seq = asn1cpp::makeSeq(AccelerationComponent);
            asn1cpp::setField(vertacc_seq->value,vertacc.getData ().getValue ());
            asn1cpp::setField(vertacc_seq->confidence,vertacc.getData ().getConfidence ());
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration,vertacc_seq);
        }

        auto perfClass = m_vdp->getPerformanceClass();
        if(perfClass.isAvailable ()) {
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass,perfClass.getData ());
        }

        auto tollZone = m_vdp->getCenDsrcTollingZone ();
        if(tollZone.isAvailable ()) {
            auto tollZone_seq = asn1cpp::makeSeq(CenDsrcTollingZone);
            asn1cpp::setField(tollZone_seq->cenDsrcTollingZoneId,tollZone.getData ().cenDsrcTollingZoneID);
            asn1cpp::setField(tollZone_seq->protectedZoneLatitude,tollZone.getData ().latitude);
            asn1cpp::setField(tollZone_seq->protectedZoneLongitude,tollZone.getData ().longitude);
            asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone,tollZone_seq);
        }

        // Store all the "previous" values used in checkCamConditions()
        m_prev_distance=m_vdp->getTravelledDistance ();
        m_prev_speed=m_vdp->getSpeedValue ();
        m_prev_heading=m_vdp->getHeadingValue ();

      }
   else
      {
        /* Fill the basicContainer */
        cam_mandatory_data = m_vdp->getCAMMandatoryData();
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence,cam_mandatory_data.altitude.getConfidence ());
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue,cam_mandatory_data.altitude.getValue ());
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.latitude,cam_mandatory_data.latitude);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.longitude,cam_mandatory_data.longitude);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength,cam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength,cam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
        asn1cpp::setField(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation,cam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
        /* Fill the highFrequencyContainer */
        //auto RSUContainerHighFreq = asn1cpp::makeSeq(RSUContainerHighFrequency);

        //High frequency RSU container

        asn1cpp::setField(cam->cam.camParameters.highFrequencyContainer.present,HighFrequencyContainer_PR_rsuContainerHighFrequency);

        //auto zones = asn1cpp::makeSeq(ProtectedCommunicationZonesRSU);
        //auto highfreq = asn1cpp::makeSeq(RSUContainerHighFrequency);

        auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
        asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
        asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
        asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);

        asn1cpp::sequenceof::pushList(cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU,protectedComm);

      }

    if(m_lowFreqContainerEnabled == true)
    {
      // Send a low frequency container only if at least 500 ms have passed since the last CAM with a low frequency container
      if(lastCamGenLowFrequency==-1 || (computeTimestampUInt64 ()-lastCamGenLowFrequency)/1e6>=500.0)
      {
        auto vehicleRole = m_vdp->getVehicleRole ();
        auto exteriorLights = m_vdp->getExteriorLights ();

        auto lowFreqContainer = asn1cpp::makeSeq(LowFrequencyContainer);

        // It is important to set also the "present" field when an ASN.1 CHOICE is involved, otherwise the setField of the whole container
        // (i.e. cam->cam.camParameters.lowFrequencyContainer in this case) will fail with a fatal error ("cannot swap null pointers")
        asn1cpp::setField(lowFreqContainer->present,LowFrequencyContainer_PR_basicVehicleContainerLowFrequency);

        if(vehicleRole.isAvailable ()) {
          asn1cpp::setField(lowFreqContainer->choice.basicVehicleContainerLowFrequency.vehicleRole,vehicleRole.getData ());
        }

        if(exteriorLights.isAvailable ()) {
          asn1cpp::bitstring::setBit(lowFreqContainer->choice.basicVehicleContainerLowFrequency.exteriorLights,setByteMask(exteriorLights.getData ()),0);
        }

        //Path History management
        if(!m_refPositions.empty ())
        {
          long pathCoverage = 0; // To compute path range which cannot be grater than 500m

          auto curr_ph = asn1cpp::makeSeq(PathPoint);
          now=computeTimestampUInt64 ()/NANO_TO_CENTI;

          //For first pathpoint
          asn1cpp::setField(curr_ph->pathPosition.deltaAltitude,cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue - m_refPositions[0].first.altitude.altitudeValue);
          asn1cpp::setField(curr_ph->pathPosition.deltaLatitude,cam->cam.camParameters.basicContainer.referencePosition.latitude - m_refPositions[0].first.latitude);
          asn1cpp::setField(curr_ph->pathPosition.deltaLongitude,cam->cam.camParameters.basicContainer.referencePosition.longitude - m_refPositions[0].first.longitude);
          asn1cpp::setField(curr_ph->pathDeltaTime,m_refPositions[0].second.deltaTime);
          asn1cpp::sequenceof::pushList(lowFreqContainer->choice.basicVehicleContainerLowFrequency.pathHistory,curr_ph);

          pathCoverage += m_refPositions[0].second.deltaCoverage;

          for(size_t i=1;i<m_refPositions.size ();i++)
            {
              auto curr_ph = asn1cpp::makeSeq(PathPoint);

              asn1cpp::setField(curr_ph->pathPosition.deltaAltitude,m_refPositions[i].first.altitude.altitudeValue - m_refPositions[i-1].first.altitude.altitudeValue);
              asn1cpp::setField(curr_ph->pathPosition.deltaLatitude,m_refPositions[i].first.latitude - m_refPositions[i-1].first.latitude);
              asn1cpp::setField(curr_ph->pathPosition.deltaLongitude,m_refPositions[i].first.longitude - m_refPositions[i-1].first.longitude);
              asn1cpp::setField(curr_ph->pathDeltaTime,m_refPositions[i].second.deltaTime);

              asn1cpp::sequenceof::pushList(lowFreqContainer->choice.basicVehicleContainerLowFrequency.pathHistory,curr_ph);

              if((pathCoverage + m_refPositions[i].second.deltaCoverage) >= 500)
                break; // If path history exceeds 500m of range, stop adding pathpoints

              pathCoverage += m_refPositions[i].second.deltaCoverage;

            }
          }

        if(vehicleRole.isAvailable () || exteriorLights.isAvailable () || m_refPositions.size()>0) {
          asn1cpp::setField(cam->cam.camParameters.lowFrequencyContainer,lowFreqContainer);
      }

        lastCamGenLowFrequency=computeTimestampUInt64 ();
      }
    }

//     Special vehicle containers

    if((m_specialVehContainerEnabled) && (m_vdp->getVehicleRole ().getData () != VehicleRole_default))
    {
      auto specialVehicleCont = asn1cpp::makeSeq(SpecialVehicleContainer);
        switch(m_vdp->getVehicleRole ().getData ()) {
        case VehicleRole_publicTransport:
          {

            auto publicTransportContainerData = m_vdp->getPublicTransportContainerData ();
            if(publicTransportContainerData.isAvailable ())
              {
                asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_publicTransportContainer);
                asn1cpp::setField(specialVehicleCont->choice.publicTransportContainer.embarkationStatus,publicTransportContainerData.getData ().embarkationStatus);

                if(publicTransportContainerData.getData ().ptActivationType.isAvailable ())
                  {
                    auto ptactivation_seq = asn1cpp::makeSeq(PtActivation);
                    asn1cpp::setField(ptactivation_seq->ptActivationType,publicTransportContainerData.getData ().ptActivationType.getData ());
                    asn1cpp::setField(ptactivation_seq->ptActivationData,publicTransportContainerData.getData ().ptActivationData.getData ());
                    asn1cpp::setField(specialVehicleCont->choice.publicTransportContainer.ptActivation,ptactivation_seq);
                  }
              }
            break;
          }
        case VehicleRole_specialTransport:
          {

            auto specialTransportContainerData = m_vdp->getSpecialTransportContainerData ();
            if(specialTransportContainerData.isAvailable ())
              {

                asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_specialTransportContainer);
                asn1cpp::bitstring::setBit(specialVehicleCont->choice.specialTransportContainer.specialTransportType,setByteMask(specialTransportContainerData.getData ().specialTransportType),0);
                asn1cpp::bitstring::setBit(specialVehicleCont->choice.specialTransportContainer.lightBarSirenInUse,setByteMask(specialTransportContainerData.getData ().lightBarSirenInUse),0);
              }

            break;
          }
        case VehicleRole_dangerousGoods:
          {
          auto dangerousGoodsBasic = m_vdp->getDangerousGoodsBasicType ();
          if(dangerousGoodsBasic.isAvailable ())
            {

              asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_dangerousGoodsContainer);
              asn1cpp::setField(specialVehicleCont->choice.dangerousGoodsContainer.dangerousGoodsBasic,dangerousGoodsBasic.getData ());

            }
          }
          break;
        case VehicleRole_roadWork:
          {

            auto roadWorksContainerBasicData = m_vdp->getRoadWorksContainerBasicData_t ();
            if(roadWorksContainerBasicData.isAvailable ())
              {

                asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_roadWorksContainerBasic);
                asn1cpp::bitstring::setBit(specialVehicleCont->choice.roadWorksContainerBasic.lightBarSirenInUse,setByteMask(roadWorksContainerBasicData.getData ().lightBarSirenInUse),0);

                if(roadWorksContainerBasicData.getData ().roadWorksSubCauseCode.isAvailable ())
                  {
                    asn1cpp::setField(specialVehicleCont->choice.roadWorksContainerBasic.roadworksSubCauseCode,roadWorksContainerBasicData.getData ().roadWorksSubCauseCode.getData ());
                  }

                if((roadWorksContainerBasicData.getData ().drivingLaneStatus.isAvailable ()) ||
                   (roadWorksContainerBasicData.getData ().innerhardShoulderStatus.isAvailable ()) ||
                   (roadWorksContainerBasicData.getData ().outerhardShoulderStatus.isAvailable ()))
                  {
                    auto closeLanes_seq = asn1cpp::makeSeq(ClosedLanes);
                    if(roadWorksContainerBasicData.getData ().drivingLaneStatus.isAvailable ())
                      {
                        asn1cpp::bitstring::setBit(closeLanes_seq->drivingLaneStatus,setByteMask(roadWorksContainerBasicData.getData ().drivingLaneStatus.getData (),0),0);
                        asn1cpp::bitstring::setBit(closeLanes_seq->drivingLaneStatus,setByteMask(roadWorksContainerBasicData.getData ().drivingLaneStatus.getData (),1),1);
                      }

                    if(roadWorksContainerBasicData.getData ().innerhardShoulderStatus.isAvailable ())
                        asn1cpp::setField(closeLanes_seq->innerhardShoulderStatus,roadWorksContainerBasicData.getData ().innerhardShoulderStatus.getData ());

                    if(roadWorksContainerBasicData.getData ().outerhardShoulderStatus.isAvailable ())
                        //asn1cpp::setField(closeLanes_seq->outerhardShoulderStatus,roadWorksContainerBasicData.getData ().outerhardShoulderStatus.getData ());

                    asn1cpp::setField(specialVehicleCont->choice.roadWorksContainerBasic.closedLanes,closeLanes_seq);
                  }

              }
            break;
          }
        case VehicleRole_rescue:
          {

            auto rescueContainerData = m_vdp->getRescueContainerLightBarSirenInUse ();
            if(rescueContainerData.isAvailable ())
              {

                asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_rescueContainer);
                asn1cpp::bitstring::setBit(specialVehicleCont->choice.rescueContainer.lightBarSirenInUse,setByteMask(rescueContainerData.getData ()),0);


              }
            break;
          }
        case VehicleRole_emergency:
          {
          auto emergencyContainerData = m_vdp->getEmergencyContainerData ();

          if(emergencyContainerData.isAvailable ())
            {

              asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_emergencyContainer);
              asn1cpp::bitstring::setBit(specialVehicleCont->choice.emergencyContainer.lightBarSirenInUse,setByteMask(emergencyContainerData.getData().lightBarSirenInUse),0);

//              if(emergencyContainerData.getData().causeCode.isAvailable ())
//                {
//                  auto cause_seq = asn1cpp::makeSeq(CauseCode);
//                  asn1cpp::setField(cause_seq->causeCode,emergencyContainerData.getData ().causeCode.getData ());
//                  asn1cpp::setField(cause_seq->subCauseCode,emergencyContainerData.getData ().subCauseCode.getData ());
//                  asn1cpp::setField(specialVehicleCont->choice.emergencyContainer.incidentIndication,cause_seq);
//                }

              if(emergencyContainerData.getData ().emergencyPriority.isAvailable ())
                {
                  asn1cpp::bitstring::setBit(specialVehicleCont->choice.emergencyContainer.emergencyPriority,setByteMask(emergencyContainerData.getData ().emergencyPriority.getData ()),0);
                }

            }
          break;
          }
        case VehicleRole_safetyCar:
          {

            auto safetyCarContainerData = m_vdp->getSafetyCarContainerData ();
            if(safetyCarContainerData.isAvailable ())
              {

                asn1cpp::setField(specialVehicleCont->present,SpecialVehicleContainer_PR_safetyCarContainer);
                asn1cpp::bitstring::setBit(specialVehicleCont->choice.safetyCarContainer.lightBarSirenInUse,setByteMask(safetyCarContainerData.getData ().lightBarSirenInUse),0);

                if(safetyCarContainerData.getData ().trafficRule.isAvailable ())
                  {
                    asn1cpp::setField(specialVehicleCont->choice.safetyCarContainer.trafficRule,safetyCarContainerData.getData ().trafficRule.getData ());
                  }
                if(safetyCarContainerData.getData ().speedLimit.isAvailable ())
                  {
                    asn1cpp::setField(specialVehicleCont->choice.safetyCarContainer.speedLimit,safetyCarContainerData.getData ().speedLimit.getData ());
                  }
//                if(safetyCarContainerData.getData ().incidentIndicationCauseCode.isAvailable ())
//                  {
//                    auto causeCode_seq = asn1cpp::makeSeq(CauseCode);
//                    asn1cpp::setField(causeCode_seq->causeCode,safetyCarContainerData.getData ().incidentIndicationCauseCode.getData ());
//                    asn1cpp::setField(causeCode_seq->subCauseCode,safetyCarContainerData.getData ().incidentIndicationSubCauseCode.getData ());
//                    asn1cpp::setField(specialVehicleCont->choice.safetyCarContainer.incidentIndication,causeCode_seq);
//                  }

              }
            break;
          }
        default:
          NS_FATAL_ERROR("CA Basic Service error. The user specified an invalid Special Vehicle Container type.");
      }

        asn1cpp::setField(cam->cam.camParameters.specialVehicleContainer,specialVehicleCont);
    }

    std::string encode_result = asn1cpp::uper::encode(cam);

    if(encode_result.size()<1)
    {
      return CAM_ASN1_UPER_ENC_ERROR;
    }

    packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());

    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = CA_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = TSB;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt =0;
    dataRequest.GNMaxRepInt=0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = packet->GetSize ();
    dataRequest.data = packet;
    m_btp->sendBTP(dataRequest);

    m_cam_sent++;

    // Estimation of the transmission time
    m_last_transmission = (double) Simulator::Now().GetMilliSeconds();
    uint32_t packetSize = packet->GetSize();
    m_Ton_pp = (double) (NanoSeconds((packetSize * 8) / 0.006) + MicroSeconds(68)).GetNanoSeconds();
    m_Ton_pp = m_Ton_pp / 1e6;

    toffUpdateAfterTransmission();

    // Store the time in which the last CAM (i.e. this one) has been generated and successfully sent
    now=computeTimestampUInt64 ()/NANO_TO_MILLI;
    now_centi = computeTimestampUInt64 ()/NANO_TO_CENTI; //Time in centiseconds(now[ms]/10->centiseconds) for Reference Position
    m_T_GenCam_ms=now-lastCamGen;
    lastCamGen = now;

    // Save this point in the list of PH points (if the low frequency container transmission is enabled)
    if(m_lowFreqContainerEnabled == true)
    {
      PathHistoryDeltas_t pathDeltas = {};

      if(m_refPositions.size() >= m_MaxPHLength) {
        m_refPositions.pop_back ();
      }

      if(!m_refPositions.empty ())
        {
          double delta;

          delta  = (long) haversineDist (asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO,
                                                           asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO,
                                                           (double) m_refPositions[0].first.latitude/DOT_ONE_MICRO, (double) m_refPositions[0].first.longitude/DOT_ONE_MICRO);

          pathDeltas.deltaCoverage = (double) delta;

          if(pathDeltas.deltaCoverage < 500)
            {
              /*
              The idea is to have an total coverage distance of (15[m]*m_MaxPHLength(23))=345m in the PathHistory, and since T_GenCamMin_ms=100ms
              then 15[m]/100[ms]=540[km/h] so we will hardly have more than 15[m] in between Reference Positions, neither more than 500[m] in the total coverage
              */
              // If coverage delta is less than 15 meters and the heading delta is less than 10 degrees.
              if((pathDeltas.deltaCoverage < 15) &&
                   (abs(m_refPositions[0].second.heading - cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue) < 100))
                  {
                    // Vehicle is not moving/is moving too slow, update deltaTime and timestamp of last pathPoint
                    m_refPositions[0].second.deltaTime += (now_centi) - m_refPositions[0].second.timestamp;
                    if(m_refPositions[0].second.deltaTime > 65535)
                      m_refPositions[0].second.deltaTime = 65535;
                    m_refPositions[0].second.timestamp = now_centi;
                    return errval;
                  }
                pathDeltas.deltaTime = (now_centi) - m_refPositions[0].second.timestamp;
            }
        }
      // Reference postiion as specified in ETSI TS 102 894-2
      pathDeltas.timestamp = now_centi;
      pathDeltas.heading =  asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,long);
      m_refPositions.insert(m_refPositions.begin(), std::make_pair(cam->cam.camParameters.basicContainer.referencePosition, pathDeltas));

    }

    return errval;
  }

  uint64_t
  CABasicService::terminateDissemination()
  {
    Simulator::Remove(m_event_camCheckConditions);
    Simulator::Remove(m_event_camDisseminationStart);
    Simulator::Remove(m_event_camRsuDissemination);
    return m_cam_sent;
  }

  int64_t
  CABasicService::computeTimestampUInt64()
  {
    int64_t int_tstamp=0;

    if (!m_real_time)
      {
        int_tstamp=Simulator::Now ().GetNanoSeconds ();
      }
    else
      {
        struct timespec tv;

        clock_gettime (CLOCK_MONOTONIC, &tv);

        int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;
      }
    return int_tstamp;
  }

  void
  CABasicService::toffUpdateAfterDeltaUpdate(double delta)
  {
    if (m_last_transmission == 0)
      return;
    double waiting = Simulator::Now().GetMilliSeconds() - m_last_transmission;
    double aux = m_Ton_pp / delta * (m_T_CheckCamGen_ms - waiting) / m_T_CheckCamGen_ms + waiting;
    aux = std::max (aux, 25.0);
    double new_gen_time = std::min (aux, 1000.0);
    setCheckCamGenMs ((long) new_gen_time);
    m_last_delta = delta;
  }

  void
  CABasicService::toffUpdateAfterTransmission()
  {
    if (m_last_delta == 0)
      return;
    double aux = m_Ton_pp / m_last_delta;
    double new_gen_time = std::max(aux, 25.0);
    new_gen_time = std::min(new_gen_time, 1000.0);
    setCheckCamGenMs ((long) new_gen_time);
  }
}
