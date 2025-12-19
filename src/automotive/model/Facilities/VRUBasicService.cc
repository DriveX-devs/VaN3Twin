//
//  VRUBasicService.cc
//

#include "VRUBasicService.h"
#include "ns3/ItsPduHeaderVam.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/BitString.hpp"
#include "ns3/asn_utils.h"
#include <cmath>

namespace ns3
{
    // Function to convert enum to string
    const char* triggerCondToString(triggcond_t triggerCond) {
        switch (triggerCond) {
            case NOT_VALID:          return "NOT_VALID";
            case DISSEMINATION_START:return "DISSEMINATION_START";
            case MAX_TIME_ELAPSED:   return "MAX_TIME_ELAPSED";
            case HEADING_CHANGE:     return "HEADING_CHANGE";
            case POSITION_CHANGE:    return "POSITION_CHANGE";
            case SPEED_CHANGE:       return "SPEED_CHANGE";
            case SAFE_DISTANCES:     return "SAFE_DISTANCES";
            default:                 return "UNKNOWN";
        }
    }

NS_LOG_COMPONENT_DEFINE("VRUBasicService");

VRUBasicService::~VRUBasicService(){
    //NS_LOG_INFO("VRUBasicService object destroyed.");
}

VRUBasicService::VRUBasicService(){
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_socket_tx = NULL;
  m_LDM = NULL;
  m_btp = NULL;
  m_VRUdpOCDA = NULL;
  m_real_time=false;

  m_T_GenVam_ms = T_GenVamMax_ms;
  m_T_CheckVamGen_ms = T_GenVamMin_ms;

  m_N_GenVam_red = 0;
  m_N_GenVam_max_red = 2;

  m_prev_heading = -1;
  m_prev_speed = -1;
  m_prev_position.x = -1;
  m_prev_position.y = -1;

  m_long_safe_d = -1;
  m_lat_safe_d = 2;
  m_vert_safe_d = 5;

  m_long_acceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

  lastVamGen = -1;

  m_vam_sent = 0;

  m_computationT_acc_ms = 0;

  m_VAM_metrics = true;
  m_csv_file_name = "Ped";

  m_VRU_role = VRU_ROLE_ON;
  m_VRU_clust_state = VRU_IDLE;

  m_trigg_cond = NOT_VALID;

  // All the optional containers are disabled by default
  m_lowFreqContainerEnabled = false;

  m_VAMReceiveCallback = nullptr;

}

VRUBasicService::VRUBasicService(unsigned long fixed_stationid,long fixed_stationtype,VRUdpOpenCDA* VRUdp,bool real_time){
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_socket_tx = NULL;
  m_LDM = NULL;
  m_btp = NULL;
    m_VRUdpOCDA = NULL;
  m_real_time = false;

  m_T_GenVam_ms = T_GenVamMax_ms;
  m_T_CheckVamGen_ms = T_GenVamMin_ms;

  m_N_GenVam_red = 0;
  m_N_GenVam_max_red = 2;

  m_prev_heading = -1;
  m_prev_speed = -1;
  m_prev_position.x = -1;
  m_prev_position.y = -1;

  m_long_safe_d = -1;
  m_lat_safe_d = 2;
  m_vert_safe_d = 5;

  m_long_acceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

  lastVamGen = -1;

  m_vam_sent = 0;

  m_computationT_acc_ms = 0;

    m_VAM_metrics = true;
    m_csv_file_name = "Ped";

  m_VRU_role = VRU_ROLE_ON;
  m_VRU_clust_state = VRU_IDLE;

  m_trigg_cond = NOT_VALID;

  // All the optional containers are disabled by default
  m_lowFreqContainerEnabled = false;

  m_VAMReceiveCallback = nullptr;

  m_station_id = (StationID_t) fixed_stationid;
  m_stationtype = (StationType_t) fixed_stationtype;
  m_real_time = real_time;

  m_VRUdpOCDA = VRUdp;
}

void VRUBasicService::setStationProperties(unsigned long fixed_stationid, long fixed_stationtype){
  m_station_id = fixed_stationid;
  m_stationtype = fixed_stationtype;
  m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
}

void VRUBasicService::setStationID(unsigned long fixed_stationid){
  m_station_id = fixed_stationid;
  m_btp->setStationID(fixed_stationid);
}

void VRUBasicService::setStationType(long fixed_stationtype){
  m_stationtype = fixed_stationtype;
  m_btp->setStationType(fixed_stationtype);
}

void VRUBasicService::setSocketRx(Ptr<Socket> socket_rx){
  m_btp->setSocketRx(socket_rx);
  m_btp->addVAMRxCallback (std::bind(&VRUBasicService::receiveVam,this,std::placeholders::_1,std::placeholders::_2));
}

void VRUBasicService::setVAMmetricsfile(std::string file_name, bool collect_metrics){
  m_VAM_metrics = collect_metrics;
  m_csv_file_name = file_name;
}

void VRUBasicService::setRoleAndClustState(VRURole_t VRU_role, VRUClusteringstate_t ClustState){
    m_VRU_role = VRU_role;
    m_VRU_clust_state = ClustState;
}

void VRUBasicService::setRoleAndClustState(){
    m_VRU_role = VRU_ROLE_ON;
    m_VRU_clust_state = VRU_IDLE;
}

void VRUBasicService::startVamDissemination(){
  if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
      if(m_VAM_metrics){
          csv_VAM_ofstream.open(m_csv_file_name+"-"+std::to_string(m_station_id)+"-VAM.csv",std::ofstream::trunc);
          csv_VAM_ofstream << "ID,Timestamp,Time from last VAM,Trigger Cond,Closest objID,Latitude,Longitude,Speed,X,Y " << std::endl;
      }
    m_event_vamDisseminationStart = Simulator::Schedule (Seconds(0), &VRUBasicService::initDissemination, this);
    m_VRU_clust_state = VRU_ACTIVE_STANDALONE;
    }
}

void VRUBasicService::startVamDissemination(double desync_s){
  if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
      if(m_VAM_metrics){
          csv_VAM_ofstream.open(m_csv_file_name+"-"+std::to_string(m_station_id)+"-VAM.csv",std::ofstream::trunc);
          csv_VAM_ofstream << "ID,Timestamp,TimefromlastVAM,TriggerCond,ClosestobjID,Latitude,Longitude,Speed,Heading,X,Y" << std::endl;
      }
    m_event_vamDisseminationStart = Simulator::Schedule (Seconds(desync_s), &VRUBasicService::initDissemination, this);
    m_VRU_clust_state = VRU_ACTIVE_STANDALONE;
    }
}

void VRUBasicService::startAccelerationComputation(long computationT_acc){
  if(computationT_acc > 0){
      if(computationT_acc < 5)
        m_computationT_acc_ms = computationT_acc;
      else{
          NS_LOG_ERROR("Warning: the time interval set for the computation of the longitudinal acceleration is too large."
                       "Set to 5s by default.");
          m_computationT_acc_ms = 5000;
        }

      m_long_acceleration = m_VRUdpOCDA->getLongAcceleration ();

      m_event_startLongAccelerationComputation = Simulator::Schedule (Seconds(m_computationT_acc_ms), &VRUBasicService::computeLongAcceleration, this);
    }
}

void VRUBasicService::receiveVam(BTPDataIndication_t dataIndication, Address from){
  Ptr<Packet> packet;
  asn1cpp::Seq<VAM> decoded_vam;
  uint8_t *buffer;

  if(m_VRU_role != VRU_ROLE_OFF){
      buffer=(uint8_t *)malloc((dataIndication.data->GetSize ())*sizeof(uint8_t));
      dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
      std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());

      /* Try to check if the received packet is really a VAM */
      if (buffer[1]!=FIX_VAMID)
        {
          NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '16' was expected.");
          free(buffer);
          return;
        }

      free(buffer);

      /** Decoding **/
      decoded_vam = asn1cpp::uper::decodeASN(packetContent, VAM);

      if(bool(decoded_vam)==false) {
          NS_LOG_ERROR("Warning: unable to decode a received VAM.");
          return;
        }

      if(m_LDM != NULL){
        //Update LDM
        //vLDM_handler(decoded_vam);
      }

      if(m_VAMReceiveCallback!=nullptr) {
        m_VAMReceiveCallback(decoded_vam,from);
      }
    }
}

void VRUBasicService::vLDM_handler(asn1cpp::Seq<VAM> decodedVAM){
  vehicleData_t vehdata;
  LDM::LDM_error_t db_retval;
  bool lowFreq_ok;

  vehdata.detected = false;
  vehdata.stationType = asn1cpp::getField(decodedVAM->vam.vamParameters.basicContainer.stationType,long);
  vehdata.stationID = asn1cpp::getField(decodedVAM->header.stationId,uint64_t);
  vehdata.lat = asn1cpp::getField(decodedVAM->vam.vamParameters.basicContainer.referencePosition.latitude,double)/(double)DOT_ONE_MICRO;
  vehdata.lon = asn1cpp::getField(decodedVAM->vam.vamParameters.basicContainer.referencePosition.longitude,double)/(double)DOT_ONE_MICRO;
  vehdata.elevation = asn1cpp::getField(decodedVAM->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue,double)/(double)CENTI;
  vehdata.heading = asn1cpp::getField(decodedVAM->vam.vamParameters.vruHighFrequencyContainer.heading.value,double)/(double)DECI;
  vehdata.speed_ms = asn1cpp::getField(decodedVAM->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue,double)/(double)CENTI;
  vehdata.vamTimestamp = asn1cpp::getField(decodedVAM->vam.generationDeltaTime,long);
  vehdata.timestamp_us = Simulator::Now ().GetMicroSeconds ();

  db_retval = m_LDM->insert(vehdata);
  if(db_retval!=LDM::LDM_OK && db_retval!=LDM::LDM_UPDATED) {
      std::cerr << "Warning! Insert on the database for pedestrian " << asn1cpp::getField(decodedVAM->header.stationId,int) << "failed!" << std::endl;
  }
}

void VRUBasicService::initDissemination(){
  m_trigg_cond = DISSEMINATION_START;
  VRUBasicService_error_t vam_error = generateAndEncodeVam();

  if((m_VRU_clust_state==VRU_ACTIVE_STANDALONE || m_VRU_clust_state==VRU_ACTIVE_CLUSTER_LEADER) && m_VRU_role==VRU_ROLE_ON)
    m_event_vamCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckVamGen_ms), &VRUBasicService::checkVamConditions, this);
}

void VRUBasicService::checkVamConditions(){
  int64_t now = computeTimestampUInt64 ()/NANO_TO_MILLI;
  VRUBasicService_error_t vam_error;
  bool condition_verified = false;
  bool redundancy_mitigation = false;

  // If no initial VAM has been triggered before checkCamConditions() has been called, throw an error
  if(m_prev_heading==-1 || m_prev_speed==-1 || (m_prev_position.x==-1 && m_prev_position.y==-1))
    {
      NS_FATAL_ERROR("Error. checkVamConditions() was called before sending any VAM and this is not allowed.");
    }

  // Check if redundancy mitigation has to be applied
  if(m_N_GenVam_red == 0)
    redundancy_mitigation = checkVamRedundancyMitigation ();

  /*
   * ETSI TS 103 300-3 V2.2.1 chap. 8 table 17 (no DCC)
   * One of the following ITS-S dynamics related conditions is given:
  */

  /* 1a)
   * The absolute difference between the current heading of the originating
   * ITS-S and the heading included in the VAM previously transmitted by the
   * originating ITS-S exceeds 4°;
  */
  double head_diff = m_VRUdpOCDA->getPedHeadingValue () - m_prev_heading;
  head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;
  if (head_diff > 4.0 || head_diff < -4.0)
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = HEADING_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        } else{
          m_N_GenVam_red++;
          condition_verified = true;
        }
    }

  /* 1b)
   * the distance between the current position of the originating ITS-S and
   * the position included in the VAM previously transmitted by the originating
   * ITS-S exceeds 4 m;
  */
  //libsumo::TraCIPosition new_pos = m_VRUdpOCDA->getPedPositionValue ();
  VDP::VDP_position_cartesian_t new_pos = m_VRUdpOCDA->getPedPositionValue ();
  double pos_diff = sqrt((new_pos.x-m_prev_position.x)*(new_pos.x-m_prev_position.x)+(new_pos.y-m_prev_position.y)*(new_pos.y-m_prev_position.y));
  if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = POSITION_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        } else{
          m_N_GenVam_red++;
          condition_verified = true;
        }
    }

  /* 1c)
   * The absolute difference between the current speed of the originating ITS-S
   * and the speed included in the VAM previously transmitted by the originating
   * ITS-S exceeds 0,5 m/s.
  */
  double speed_diff = m_VRUdpOCDA->getPedSpeedValue () - m_prev_speed;
  if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = SPEED_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        } else{
          m_N_GenVam_red++;
          condition_verified = true;
        }
    }

  // Computation of the longitudinal safe distance
  m_long_safe_d = abs(m_VRUdpOCDA->getPedSpeedValue ()*(T_GenVamMax_ms/1000));

  // Get the minimum distance of other vehicles/pedestrians from the current pedestrian
  m_min_dist = m_VRUdpOCDA->get_min_distance (m_LDM);
  //todo:check if get_min_distance is working properly

  /* 1d)
   * If the longitudinal distance between the originating ITS-S and the nearest pedestrian/vehicle
   * is smaller than v/t, where v is the current longitudinal speed of the originating ITS-S
   * and t is the maximum time interval between the generation of two consecutive VAMs, the lateral
   * distance smaller than 2 m and the vertical distance smaller than 5 m, a VAM must be transmitted
  */
  if (!condition_verified && m_min_dist[1].longitudinal < m_long_safe_d && m_min_dist[1].lateral < m_lat_safe_d && m_min_dist[1].vertical < m_vert_safe_d)
    {
      m_N_GenVam_red = 0;

      m_trigg_cond = SAFE_DISTANCES;
      m_min_dist[1].safe_dist = true;
      vam_error = generateAndEncodeVam ();
      if(vam_error==VAM_NO_ERROR)
        {
          condition_verified = true;
        } else {
          NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
        }
    } else{
      if(!condition_verified && m_min_dist[0].longitudinal < m_long_safe_d && m_min_dist[0].lateral < m_lat_safe_d && m_min_dist[0].vertical < m_vert_safe_d)
        {
          if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
              m_N_GenVam_red = 0;

              m_trigg_cond = SAFE_DISTANCES;
              m_min_dist[0].safe_dist = true;
              vam_error = generateAndEncodeVam ();
              if(vam_error==VAM_NO_ERROR)
                {
                  condition_verified = true;
                } else {
                  NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
                }
            } else{
              m_N_GenVam_red++;
              condition_verified = true;
            }
        }
    }

  /* 2)
   * The time elapsed since the last VAM generation is equal to or greater than T_GenVam
  */
  if(!condition_verified && (now-lastVamGen>=m_T_GenVam_ms))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = MAX_TIME_ELAPSED;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        } else{
          m_N_GenVam_red++;
          condition_verified = true;
        }
    }

  if((m_VRU_clust_state==VRU_IDLE || m_VRU_clust_state==VRU_ACTIVE_STANDALONE || m_VRU_clust_state==VRU_ACTIVE_CLUSTER_LEADER) && m_VRU_role==VRU_ROLE_ON)
    m_event_vamCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckVamGen_ms), &VRUBasicService::checkVamConditions, this);
}

void VRUBasicService::computeLongAcceleration(){
  m_long_acceleration = m_VRUdpOCDA->getLongAcceleration ();

  m_event_computeLongAcceleration = Simulator::Schedule (Seconds(m_computationT_acc_ms), &VRUBasicService::computeLongAcceleration, this);
}

bool VRUBasicService::checkVamRedundancyMitigation(){
  bool redundancy_mitigation = false;
  int64_t now = computeTimestampUInt64 ()/NANO_TO_MILLI;
  std::vector<LDM::returnedVehicleData_t> selectedStations;

  VRUdp_position_latlon_t ped_pos = m_VRUdpOCDA->getPedPosition ();
  double ped_speed = m_VRUdpOCDA->getPedSpeedValue ();
  double ped_heading = m_VRUdpOCDA->getPedHeadingValue ();
  ped_heading += (ped_heading>180.0) ? -360.0 : (ped_heading<-180.0) ? 360.0 : 0.0;

  if(now-lastVamGen < m_N_GenVam_max_red*5000){
      m_LDM->rangeSelect (4,ped_pos.lat,ped_pos.lon,selectedStations);

      for(std::vector<LDM::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end () && !redundancy_mitigation; ++it){
          if(it->vehData.stationType == StationType_pedestrian){
              double speed_diff = it->vehData.speed_ms - ped_speed;
              double near_VRU_heading = it->vehData.heading;
              near_VRU_heading += (near_VRU_heading>180.0) ? -360.0 : (near_VRU_heading<-180.0) ? 360.0 : 0.0;
              double heading_diff = near_VRU_heading - ped_heading;

              if((speed_diff < 0.5 && speed_diff > -0.5) && (heading_diff < 4 && heading_diff > -4)){
                  redundancy_mitigation = true;
                  m_N_GenVam_max_red = (int16_t)std::round(((double)std::rand()/RAND_MAX)*8) + 2;
                }
            }
        }
    }

  return redundancy_mitigation;
}

VRUBasicService_error_t VRUBasicService::generateAndEncodeVam(){
  VRUBasicService_error_t errval = VAM_NO_ERROR;
  VAM_mandatory_data_t vam_mandatory_data;

  Ptr<Packet> packet;

  BTPDataRequest_t dataRequest = {};

  int64_t now;

  /* Collect data for mandatory containers */
  auto vam = asn1cpp::makeSeq(VAM);

  if(bool(vam)==false)
    {
      return VAM_ALLOC_ERROR;
    }

  /* Fill the header */
  asn1cpp::setField(vam->header.messageId, FIX_VAMID);
  asn1cpp::setField(vam->header.protocolVersion , 2);
  asn1cpp::setField(vam->header.stationId, m_station_id);

  /*
   * Compute the generationDeltaTime, computed as the time corresponding to the
   * time of the reference position in the VAM, considered as time of the VAM generation.
   * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
   * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
   * generationDeltaTime = TimestampIts mod 65 536"
  */
  asn1cpp::setField(vam->vam.generationDeltaTime, compute_timestampIts (m_real_time) % 65536);

  /* Fill the basicContainer's station type */
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.stationType, m_stationtype);

  vam_mandatory_data = m_VRUdpOCDA->getVAMMandatoryData();

  /* Fill the basicContainer */
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue, vam_mandatory_data.altitude.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeConfidence, vam_mandatory_data.altitude.getConfidence ());
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.latitude, vam_mandatory_data.latitude);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.longitude, vam_mandatory_data.longitude);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength, vam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength, vam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation, vam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);

  /* Fill the highFrequencyContainer */
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.heading.value, vam_mandatory_data.heading.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.heading.confidence, vam_mandatory_data.heading.getConfidence ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue, vam_mandatory_data.speed.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedConfidence, vam_mandatory_data.speed.getConfidence ());
  if(m_computationT_acc_ms > 0){
      asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue,
                        m_long_acceleration.getValue ());
      asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationConfidence,
                        m_long_acceleration.getConfidence ());
    } else{
      asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue,
                        vam_mandatory_data.longAcceleration.getValue ());
      asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationConfidence,
                        vam_mandatory_data.longAcceleration.getConfidence ());
    }

  // Store all the "previous" values used in checkVamConditions()
  m_prev_position = m_VRUdpOCDA->getPedPositionValue ();
  m_prev_speed = m_VRUdpOCDA->getPedSpeedValue ();
  m_prev_heading = m_VRUdpOCDA->getPedHeadingValue ();

//    std::cout << "VAM_mandatory_data from VRUdpOCDA: " << std::endl;
//    std::cout << "ID: " << m_station_id << std::endl;
//    std::cout << "speed: " << vam_mandatory_data.speed.getValue() << std::endl;
//    std::cout << "heading: " << vam_mandatory_data.heading.getValue() << std::endl;
//    std::cout << "altitude: " << vam_mandatory_data.altitude.getValue() << std::endl;
//    std::cout << "longitude: " << vam_mandatory_data.longitude << std::endl;
//    std::cout << "latitude: " << vam_mandatory_data.latitude << std::endl;
//
//    std::cout << "VAM Object: " << vam << std::endl;
  /* VAM encoding */
  std::string encode_result = asn1cpp::uper::encode(vam);

  if(encode_result.size()<1)
  {
      std::cout << " ENCODING ERROR " << std::endl;
    return VAM_ASN1_UPER_ENC_ERROR;

  }

  packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());

  dataRequest.BTPType = BTP_B; //!< BTP-B
  dataRequest.destPort = VA_PORT;
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

  m_vam_sent++;

  // Estimation of the transmission time
  m_last_transmission = (double) Simulator::Now().GetMilliSeconds();
  uint32_t packetSize = packet->GetSize();
  m_Ton_pp = (double) (NanoSeconds((packetSize * 8) / 0.006) + MicroSeconds(68)).GetNanoSeconds();
  m_Ton_pp = m_Ton_pp / 1e6;

  toffUpdateAfterTransmission();

  // Compute the time in which the VAM has been sent
  now = computeTimestampUInt64 ()/NANO_TO_MILLI;

  // Store the time elapsed since last VAM generation
  long time_elapsed = -1;
  if(lastVamGen == -1)
    time_elapsed = now;
  else
    time_elapsed = now - lastVamGen;

  // Store the time in which the last VAM (i.e. this one) has been generated and successfully sent
  lastVamGen = now;

  // Get the position of the pedestrian who sent the VAM in terms of latitude and longitude
  VRUdp_position_latlon_t pos_ped = m_VRUdpOCDA->getPedPosition ();

  // Store the data of interest in the .csv file
  if(m_VAM_metrics){
      csv_VAM_ofstream << m_station_id << "," << now << "," << time_elapsed << "," << triggerCondToString(m_trigg_cond) << ",";
      if(m_trigg_cond == SAFE_DISTANCES){
          if(m_min_dist[0].safe_dist)
            csv_VAM_ofstream << m_min_dist[0].ID;
          else
            csv_VAM_ofstream << m_min_dist[1].ID;
      } else {
        csv_VAM_ofstream << -1;
      }

      csv_VAM_ofstream << "," << std::setprecision (12) << pos_ped.lat << "," << std::setprecision (12) << pos_ped.lon << "," <<
                       ( (double) vam_mandatory_data.speed.getValue())/CENTI << "," << vam_mandatory_data.heading.getValue() <<
                        "," << m_prev_position.x << "," <<  m_prev_position.y << std::endl;
  }

  return errval;
}

uint64_t VRUBasicService::terminateDissemination(){
    if (!m_csv_file_name.empty ())
    {
        csv_VAM_ofstream.close ();
    }
  Simulator::Remove(m_event_vamCheckConditions);
  Simulator::Remove(m_event_vamDisseminationStart);
  Simulator::Remove(m_event_computeLongAcceleration);

  return m_vam_sent;
}

int64_t VRUBasicService::computeTimestampUInt64()
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
VRUBasicService::toffUpdateAfterDeltaUpdate(double delta)
{
  if (m_last_transmission == 0)
    return;
  double waiting = Simulator::Now().GetMilliSeconds() - m_last_transmission;
  double aux = m_Ton_pp / delta * (m_T_CheckVamGen_ms - waiting) / m_T_CheckVamGen_ms + waiting;
  aux = std::max (aux, 25.0);
  double new_gen_time = std::min (aux, 1000.0);
  setCheckVamGenMs ((long) new_gen_time);
  m_last_delta = delta;
}

void
VRUBasicService::toffUpdateAfterTransmission()
{
  if (m_last_delta == 0)
    return;
  double aux = m_Ton_pp / m_last_delta;
  double new_gen_time = std::max(aux, 25.0);
  new_gen_time = std::min(new_gen_time, 1000.0);
  setCheckVamGenMs ((long) new_gen_time);
}

}
