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
 *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#include "mcBasicService.h"
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

#include "ns3/asn_SEQUENCE_OF.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("MCBasicService");

  MCSpecification::~MCSpecification()
  {
    for (auto &adv : m_maneuver_advice)
      {
        if (adv->currentStateAdvisedChange != nullptr)
          {
            ASN_STRUCT_FREE(asn_DEF_CurrentStateAdvisedChange, adv->currentStateAdvisedChange);
          }
        //ASN_STRUCT_FREE(asn_DEF_Submanoeuvres, &adv.submaneuvres);
        ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_Submanoeuvres, &adv->submaneuvres);

      }

    for (auto &subm : m_submaneuver_description)
      {
        if (subm->kinematicsCharacteristics != nullptr)
          {
            ASN_STRUCT_FREE(asn_DEF_KinematicsCharacteristics, subm->kinematicsCharacteristics);
          }
        if (subm->targetRoadResourceIContainer != nullptr)
          {
            ASN_STRUCT_FREE(asn_DEF_TrrDescription, subm->targetRoadResourceIContainer);
          }
        if (subm->referenceTrajectory != nullptr)
          {
            ASN_STRUCT_FREE(asn_DEF_Trajectory, subm->referenceTrajectory);
          }
        if (subm->submanoeuvreStrategy != nullptr)
          {
            ASN_STRUCT_FREE(asn_DEF_SubmanoeuvreStrategy, subm->submanoeuvreStrategy);
          }
      }
  }

  bool MCSpecification::checkContainers()
  {
    int count = m_vehicle_advise_container
                + m_vehicle_maneuver_container
                + m_vehicle_acknowledgement_container
                + m_vehicle_response_container
                + m_vehicle_terminator_container;
    return count == 1;
  }


  MCBasicService::~MCBasicService() {
    NS_LOG_INFO("MCBasicService object destroyed.");
  }

  MCBasicService::MCBasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket_tx=NULL;
    m_btp = NULL;
    m_real_time=false;

    // Setting a default value of m_T_CheckMCMGen_ms equal to 100 ms (i.e. T_GenMCMMin_ms)
    m_T_CheckMCMGen_ms=T_GenMCMMin_ms;

    m_T_GenMCM_ms=T_GenMCMMax_ms;

    lastMCMGen=-1;

    m_vehicle=true;

    // MCM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenMCM_ms=T_GenMCMMax_ms;

    m_MCM_sent=0;

    m_refPositions.clear ();
  }

  MCBasicService::MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp, bool real_time, bool is_vehicle)
  {
    m_socket_tx=NULL;
    m_btp = NULL;

    // Setting a default value of m_T_CheckMCMGen_ms equal to 100 ms (i.e. T_GenMCMMin_ms)
    m_T_CheckMCMGen_ms=T_GenMCMMin_ms;

    m_T_GenMCM_ms=T_GenMCMMax_ms;

    lastMCMGen=-1;

    // MCM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenMCM_ms=1000;

    m_MCM_sent=0;

    m_refPositions.clear ();

    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;

    m_vdp=vdp;
    m_real_time=real_time;

    m_vehicle=is_vehicle;
  }


  MCBasicService::MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp, bool real_time, bool is_vehicle, Ptr<Socket> socket_tx)
  {
    MCBasicService(fixed_stationid,fixed_stationtype,vdp,real_time,is_vehicle);

    m_socket_tx=socket_tx;
  }

  void
  MCBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
  }

  void
  MCBasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
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
  MCBasicService::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
    m_btp->setStationID(fixed_stationid);
  }

  void
  MCBasicService::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
  }

  void
  MCBasicService::setSocketRx (Ptr<Socket> socket_rx)
  {
    m_btp->setSocketRx(socket_rx);
    m_btp->addMCMRxCallback (std::bind(&MCBasicService::receiveMCM,this,std::placeholders::_1,std::placeholders::_2));
  }

  void
  MCBasicService::vLDM_handler (asn1cpp::Seq<MCM> decodedMCM)
  {
    return;
  }

  void
  MCBasicService::receiveMCM (BTPDataIndication_t dataIndication, Address from)
  {
    Ptr<Packet> packet;
    asn1cpp::Seq<MCM> decoded_MCM;

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

    /* Try to check if the received packet is really a MCM */
    if (buffer[1]!=FIX_MCMID)
      {
        NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '2' was expected.");
        free(buffer);
        return;
      }

    free(buffer);

    /** Decoding **/
    decoded_MCM = asn1cpp::uper::decodeASN(packetContent, MCM);

    if(bool(decoded_MCM)==false) {
        NS_LOG_ERROR("Warning: unable to decode a received MCM.");
        return;
      }

    if(m_LDM != NULL){
      //Update LDM
      vLDM_handler(decoded_MCM);
    }

    if(m_MCReceiveCallback!=nullptr) {
      m_MCReceiveCallback(decoded_MCM,from);
    }
    if(m_MCReceiveCallbackExtended!=nullptr) {
      m_MCReceiveCallbackExtended(decoded_MCM,from,m_station_id,m_stationtype,GetSignalInfo());
    }
  }


  void
  MCBasicService::checkMCMConditions()
  {
    int64_t now=computeTimestampUInt64 ()/NANO_TO_MILLI;
    MCBasicService_error_t MCM_error;
    bool condition_verified=false;
    static bool dyn_cond_verified=false;

    // TODO

    m_event_MCMCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckMCMGen_ms), &MCBasicService::checkMCMConditions, this);
  }

  MCBasicService_error_t
  MCBasicService::generateAndEncodeMCM(MCSpecification *specification)
  {
    // Only one container must be activated for one message
    NS_ASSERT (specification->checkContainers());
    VDP::MCM_mandatory_data_t MCM_mandatory_data;
    MCBasicService_error_t errval=MCM_NO_ERROR;

    Ptr<Packet> packet;

    BTPDataRequest_t dataRequest = {};

    int64_t now,now_centi;

    /* Collect data for mandatory containers */
    auto MCM_message = asn1cpp::makeSeq(MCM);

    if(bool(MCM_message)==false)
      {
        return MCM_ALLOC_ERROR;
      }

    /* Fill the header */
    asn1cpp::setField(MCM_message->header.messageId, FIX_MCMID);
    asn1cpp::setField(MCM_message->header.protocolVersion, 1);
    asn1cpp::setField(MCM_message->header.stationId, m_station_id);

    /* Fill the basicContainer */

    /*
     * Compute the generationDeltaTime, "computed as the time corresponding to the
     * time of the reference position in the MCM, considered as time of the MCM generation.
     * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
     * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
     * generationDeltaTime = TimestampIts mod 65 536"
    */
    asn1cpp::setField(MCM_message->payload.basicContainer.generationDeltaTime, compute_timestampIts (m_real_time) % 65536);

    asn1cpp::setField(MCM_message->payload.basicContainer.stationID, m_station_id);
    asn1cpp::setField(MCM_message->payload.basicContainer.itssRole, specification->getMCMItsRole());
    if(m_vehicle==true)
      {
        asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_vehicle);
        MCM_mandatory_data = m_vdp->getMCMMandatoryData();
        asn1cpp::setField(MCM_message->payload.basicContainer.position.latitude, MCM_mandatory_data.latitude);
        asn1cpp::setField(MCM_message->payload.basicContainer.position.longitude, MCM_mandatory_data.longitude);
        asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeValue, MCM_mandatory_data.altitude.getValue());
        asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeConfidence, MCM_mandatory_data.altitude.getConfidence());
        asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
        asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMinorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
        asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisOrientation, MCM_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
        asn1cpp::setField(MCM_message->payload.basicContainer.mcmType, specification->getMCMType());
        asn1cpp::setField(MCM_message->payload.basicContainer.manoeuvreId, specification->getManeuverID());
        asn1cpp::setField(MCM_message->payload.basicContainer.concept, specification->getMCMConcept());
        auto *rational = (ManoeuvreCoordinationRational_t*)CALLOC(1, sizeof(ManoeuvreCoordinationRational_t));
        if (specification->getMCMConcept() == 0) {
            asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationGoal);
            asn1cpp::setField(rational->choice.manoeuvreCooperationGoal, specification->getMCMGoal());
          } else {
            asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationCost);
            asn1cpp::setField(rational->choice.manoeuvreCooperationCost, specification->getMCMCost());
          }
        MCM_message->payload.basicContainer.rational = rational;
        if (specification->getMCMType() == 4 || specification->getMCMType() == 7)
          {
            asn1cpp::setField (MCM_message->payload.basicContainer.executionStatus, specification->getMCMStatus());
          }
      }
   else
     {
       /* Fill the basicContainer for RSU*/
       asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_roadsideUnit);
       // TODO
     }

   if (specification->getManeuverContainer())
     {
       // Select this choice
       asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_vehicleManoeuvreContainer);

       // Allocate + fill VehicleManoeuvreContainer
       auto &man = MCM_message->payload.mcmContainer.choice.vehicleManoeuvreContainer;
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleHeading.value, MCM_mandatory_data.heading.getValue());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleHeading.confidence, MCM_mandatory_data.heading.getConfidence());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSpeed.speedValue, MCM_mandatory_data.speed.getValue());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSpeed.speedConfidence, MCM_mandatory_data.speed.getConfidence());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleWidth, MCM_mandatory_data.VehicleWidth);
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthValue, MCM_mandatory_data.VehicleLength.getValue());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthConfidenceIndication, MCM_mandatory_data.VehicleLength.getConfidence());
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleHeight, VehicleHeight_unavailable);
       asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleType, specification->getVehicleType());

       for (auto it = specification->getSubmaneuverDescription().begin(); it != specification->getSubmaneuverDescription().end(); ++it)
         {
           auto *subd = (SubmanoeuvreDescription *)CALLOC(1, sizeof(SubmanoeuvreDescription));
           if (asn_copy(&asn_DEF_SubmanoeuvreDescription, (void**)&subd, &(*it)) == 0)
             {
               if (ASN_SEQUENCE_ADD (&man.submaneuvres, subd) != 0)
                 {
                   ASN_STRUCT_FREE (asn_DEF_SubmanoeuvreDescription, subd);
                 }
             }
           else
             {
               ASN_STRUCT_FREE(asn_DEF_SubmanoeuvreDescription, subd);
             }
         }

       for (auto it = specification->getManeuverAdvice().begin(); it != specification->getManeuverAdvice().end(); ++it)
         {
           auto *advice = (ManoeuvreAdvice *)CALLOC(1, sizeof(ManoeuvreAdvice));
           if (asn_copy(&asn_DEF_ManoeuvreAdvice, (void**)&advice, &(*it)) == 0)
             {
               if (ASN_SEQUENCE_ADD (&man.manoeuvreAdvice, advice) != 0)
                 {
                   ASN_STRUCT_FREE (asn_DEF_ManoeuvreAdvice, advice);
                 }
             }
           else
             {
               ASN_STRUCT_FREE(asn_DEF_ManoeuvreAdvice, advice);
             }
         }
     }
   else if (specification->getAdviseContainer())
     {
       asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_advisedManoeuvreContainer);
       auto &adv = MCM_message->payload.mcmContainer.choice.advisedManoeuvreContainer;
       for (auto it = specification->getManeuverAdvice().begin(); it != specification->getManeuverAdvice().end(); ++it)
         {
           auto *advice = (ManoeuvreAdvice *)CALLOC(1, sizeof(ManoeuvreAdvice));
           if (asn_copy(&asn_DEF_ManoeuvreAdvice, (void**)&advice, &(*it)) == 0)
             {
               if (ASN_SEQUENCE_ADD (&adv, advice) != 0)
                 {
                   ASN_STRUCT_FREE (asn_DEF_ManoeuvreAdvice, advice);
                 }
             }
           else
             {
               ASN_STRUCT_FREE(asn_DEF_ManoeuvreAdvice, advice);
             }
         }
     }
   else if (specification->getAcknowledgmentContainer())
     {
       asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_acknowledgmentContainer);
       asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.acknowledgedType, McmType_acknowledgment);
       asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.generationDeltaTime, compute_timestampIts (m_real_time) % 65536);
     }
   else if (specification->getResponseContainer())
     {
       asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_responseContainer);
       if (specification->getMCMResponse() == 0)
         {
           asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_accept);
         }
       else
         {
           asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_decline);
         }
       }
   else if (specification->getTerminatorContainer())
     {
       asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_terminationContainer);
     }
   else
     {
       NS_FATAL_ERROR ("No container specified.");
     }

   std::string encode_result = asn1cpp::uper::encode(MCM_message);

   /*
   for (auto ptr_it = free_ptr.begin(); ptr_it != free_ptr.end(); ++ptr_it)
     {
       void* ptr = std::get<0>(*ptr_it);
       asn_TYPE_descriptor_s type = std::get<1>(*ptr_it);
       if (ptr != nullptr)
         {
           ASN_STRUCT_FREE(type, ptr);
         }
     }
     */

    if(encode_result.size()<1)
    {
      return MCM_ASN1_UPER_ENC_ERROR;
    }

    packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());

    // dataRequest.socket = specification->socket;
    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = MC_PORT;
    dataRequest.destPInfo = 0;
    dataRequest.GNType = TSB;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNRepInt = 0;
    dataRequest.GNMaxRepInt = 0;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = packet->GetSize ();
    dataRequest.data = packet;
    std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, 0, MessageId_mcm);
    GNDataConfirm_t dataConfirm = std::get<0>(status);
    MessageId_t message_id = std::get<1>(status);
    /* Update the MCM statistics */
    if(dataConfirm == ACCEPTED) {
        if (message_id == MessageId_mcm) m_MCM_sent++;
      }

    // Estimation of the transmission time
    m_last_transmission = (double) Simulator::Now().GetMilliSeconds();

    // Store the time in which the last MCM (i.e. this one) has been generated and successfully sent
    now=computeTimestampUInt64 ()/NANO_TO_MILLI;
    now_centi = computeTimestampUInt64 ()/NANO_TO_CENTI; //Time in centiseconds(now[ms]/10->centiseconds) for Reference Position
    m_T_GenMCM_ms = now - lastMCMGen;
    lastMCMGen = now;

    return errval;
  }

  uint64_t
  MCBasicService::terminateDissemination()
  {
    Simulator::Remove(m_event_MCMCheckConditions);
    Simulator::Remove(m_event_MCMDisseminationStart);
    Simulator::Remove(m_event_MCMRsuDissemination);
    return m_MCM_sent;
  }

  int64_t
  MCBasicService::computeTimestampUInt64()
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

  }
