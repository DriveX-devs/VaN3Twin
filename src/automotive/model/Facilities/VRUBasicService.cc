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
#include "ns3/VRUdp.h"
#include "ns3/asn_utils.h"
#include "ns3/ldm-utils.h"
#include "ns3/simulator.h"
#include <cmath>

// TODO make a metric to measure the Age of Information with and without the TIP

// TODO import the example and create a main file in which is possible to set the TIP parameters

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("VRUBasicService");

double tip_exponential(double t2c, double t2c_min, double k) {
    double exp_term = std::exp(-k * (t2c - t2c_min));
    return std::min(1.0, exp_term);
}

double tip_gaussian(double ttc, double sigma) {
    double numerator = -(ttc * ttc);
    double denominator = 2.0 * (sigma * sigma);
    return std::exp(numerator / denominator);
}

VRUBasicService::~VRUBasicService(){
    //NS_LOG_INFO("VRUBasicService object destroyed.");
}

VRUBasicService::VRUBasicService(){
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_socket_tx = NULL;
  m_LDM = NULL;
  m_btp = NULL;
  m_VRUdp = NULL;
  m_real_time=false;

  m_T_GenVam_ms = T_GenVamMax_ms;
  m_T_CheckVamGen_ms = T_GenVamMin_ms;

  m_N_GenVam_red = 0;
  m_N_GenVam_max_red = 2;

  m_prev_heading = -1;
  m_prev_speed = -1;
  m_prev_position.x = -1;
  m_prev_position.y = -1;
  m_prev_lat = -1;
  m_prev_lon = -1;

  m_long_safe_d = -1;
  m_lat_safe_d = 2;
  m_vert_safe_d = 5;

  m_long_acceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

  lastVamGen = -1;

  m_vam_sent = 0;

  m_computationT_acc_ms = 0;

  m_VAM_metrics = false;
  m_csv_file_name = "";

  m_VRU_role = VRU_ROLE_ON;
  m_VRU_clust_state = VRU_IDLE;

  m_trigg_cond = NOT_VALID;

  // All the optional containers are disabled by default
  m_lowFreqContainerEnabled = false;

  m_VAMReceiveCallback = nullptr;
  m_VAMReceiveCallbackExtended = nullptr;
}

VRUBasicService::VRUBasicService(unsigned long fixed_stationid,long fixed_stationtype,VRUdp* VRUdp,bool real_time){
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_socket_tx = NULL;
  m_LDM = NULL;
  m_btp = NULL;
  m_VRUdp = NULL;
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

  m_VAM_metrics = false;
  m_csv_file_name = "";

  m_VRU_role = VRU_ROLE_ON;
  m_VRU_clust_state = VRU_IDLE;

  m_trigg_cond = NOT_VALID;

  // All the optional containers are disabled by default
  m_lowFreqContainerEnabled = false;

  m_VAMReceiveCallback = nullptr;
  m_VAMReceiveCallbackExtended = nullptr;

  m_station_id = (StationID_t) fixed_stationid;
  m_stationtype = (StationType_t) fixed_stationtype;
  m_real_time = real_time;

  m_VRUdp = VRUdp;
}

void VRUBasicService::write_log_triggering(bool condition_verified, bool vamredmit_verified, float head_diff, float pos_diff, float speed_diff, long time_difference, std::string data_head, std::string data_pos, std::string data_speed, std::string data_safed, std::string data_tip, std::string data_time, std::string data_vamredmit, std::string data_dcc) {
  if (m_log_triggering && m_log_filename != "") {
      std::string data="";
      std::string sent="false";

      std::string motivation;
      std::string joint;
      int numConditions=0;
      std::string num_VAMs_sent="";

      long time = Simulator::Now().GetMilliSeconds();

      // Check the motivation of the VAM sent
      if (!condition_verified && !vamredmit_verified) {
          motivation="none";
          num_VAMs_sent="unavailable";
        } else if(vamredmit_verified){
          motivation="VAM Redundancy Mitigation";
          num_VAMs_sent="unavailable";
        } else {
          data="[VAM] VAM sent\n";
          sent="true";

          if(head_diff > 10.0 || head_diff < -10.0)
          // if(head_diff > 4.0 || head_diff < -4.0)
            {
              motivation="heading";
              joint=joint+"H";
              num_VAMs_sent=std::to_string(m_head_sent);
              numConditions++;
            }

          if((pos_diff > 4.0 || pos_diff < -4.0)) {
              motivation="position";
              joint=joint+"P";
              num_VAMs_sent=std::to_string(m_pos_sent);
              numConditions++;
            }

          if(speed_diff > 0.5 || speed_diff < -0.5) {
              motivation="speed";
              joint=joint+"S";
              num_VAMs_sent=std::to_string(m_speed_sent);
              numConditions++;
            }

          if (m_min_dist.size() != 0) {
              if((m_min_dist[1].longitudinal < m_long_safe_d && m_min_dist[1].lateral < m_lat_safe_d && m_min_dist[1].vertical < m_vert_safe_d) || (m_min_dist[0].longitudinal < m_long_safe_d && m_min_dist[0].lateral < m_lat_safe_d && m_min_dist[0].vertical < m_vert_safe_d)){
                  motivation="safe_distances";
                  joint=joint+"D";
                  num_VAMs_sent=std::to_string(m_safedist_sent);
                  numConditions++;
                }
            }

          if (m_tip_array.size() > 0) {
              motivation="TIP";
              joint=joint+"TI";
              num_VAMs_sent=std::to_string(m_tip_sent);
              numConditions++;
            }

          if(abs(time_difference - m_T_GenVam_ms) <= 10 || (m_T_GenVam_ms - time_difference) <= 0 ) {
              motivation="time";
              joint=joint+"T";
              num_VAMs_sent=std::to_string(m_time_sent);
              numConditions++;
            }

          // When joint with a single other motivation, the joint motivation should not be considered
          if(numConditions>1) {
              motivation="joint("+joint+")";
              if(joint=="HT") {
                  motivation="heading";
              }
              if(joint=="PT") {
                  motivation="position";
              }
              if(joint=="ST") {
                  motivation="speed";
              }
              if(joint=="DT") {
                  motivation="safe_distances";
              }
              if(joint=="TI") {
                motivation="TIP";
              }
            }

          if(condition_verified && strlen(motivation.c_str())==0) {
              motivation="numPkt";
            }
        }

      // Create the data for the log print
      data+="[LOG] Timestamp="+std::to_string(time)+" VAMSent="+sent+" Motivation="+motivation+" NUMVAMsSent="+num_VAMs_sent+" HeadDiff="+std::to_string(head_diff)+" PosDiff="+std::to_string(pos_diff)+" SpeedDiff="+std::to_string(speed_diff)+" TIP="+std::to_string(m_tip_map.size())+" TimeDiff="+std::to_string(time_difference)+"\n";
      data=data+data_head+data_pos+data_speed+data_safed+data_tip+data_time+data_vamredmit+data_dcc+"\n";

      std::ofstream file (m_log_filename, std::ios::app);
      file << data;
    }
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
  m_btp->addVRURxCallback(std::bind(&VRUBasicService::VRUreceivedNewMessageForTIP,this,std::placeholders::_1,std::placeholders::_2));
}

void VRUBasicService::setVAMmetricsfile(std::string file_name, bool collect_metrics){
  m_VAM_metrics = collect_metrics;
  m_csv_file_name = file_name;
}

void VRUBasicService::startVamDissemination(){
  if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
    m_event_vamDisseminationStart = Simulator::Schedule (Seconds(0), &VRUBasicService::initDissemination, this);
    m_VRU_clust_state = VRU_ACTIVE_STANDALONE;
    }
}

void VRUBasicService::startVamDissemination(double desync_s){
  if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
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

      m_long_acceleration = m_VRUdp->getLongAcceleration ();

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
        vLDM_handler(decoded_vam);
      }

      if(m_VAMReceiveCallback!=nullptr) {
        m_VAMReceiveCallback(decoded_vam,from);
      } else if(m_VAMReceiveCallbackExtended!=nullptr){
        m_VAMReceiveCallbackExtended(decoded_vam,from,m_station_id,m_stationtype);
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
  bool vamredmit_verified = false;
  bool redundancy_mitigation = false;
  std::string data_head="";
  std::string data_pos="";
  std::string data_speed="";
  std::string data_safed="";
  std::string data_vamredmit="";
  std::string data_time="";
  std::string data_dcc="";
  std::string data_tip="";

  // If no initial VAM has been triggered before checkCamConditions() has been called, throw an error
  if(m_prev_heading==-1 || m_prev_speed==-1 || (m_prev_position.x==-1 && m_prev_position.y==-1))
    {
      NS_FATAL_ERROR("Error. checkVamConditions() was called before sending any VAM and this is not allowed.");
    }

  // Check if redundancy mitigation has to be applied
  if(m_N_GenVam_red == 0)
    {
      redundancy_mitigation = checkVamRedundancyMitigation ();
    }

  /*
   * ETSI TS 103 300-3 V2.2.1 chap. 8 table 17 (no DCC)
   * One of the following ITS-S dynamics related conditions is given:
  */

  /* 1a)
   * The absolute difference between the current heading of the originating
   * ITS-S and the heading included in the VAM previously transmitted by the
   * originating ITS-S exceeds 4°;
  */

  double head_diff = m_VRUdp->getPedHeadingValue () - m_prev_heading;
  head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;
  data_head="[HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable/10)+" PrevHead="+std::to_string(m_prev_heading)+" CurrHead="+std::to_string(m_VRUdp->getPedHeadingValue ())+" HeadDiff="+std::to_string(head_diff)+"\n";
  if (head_diff > 10.0 || head_diff < -10.0)
  //if (head_diff > 4.0 || head_diff < -4.0)
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
        if (m_T_next_dcc == -1 || now - lastVamGen >= m_T_next_dcc)
        {
          m_N_GenVam_red = 0;

          m_trigg_cond = HEADING_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_head_sent++;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        }
        else
        {
          data_dcc += "[HEADING] verified but [DCC] doesn't allow sending\n";
        }
      }
      else{
          m_N_GenVam_red++;
          vamredmit_verified = true;
          // condition_verified = true;
        }
      
    }

  /* 1b)
   * the distance between the current position of the originating ITS-S and
   * the position included in the VAM previously transmitted by the originating
   * ITS-S exceeds 4 m;
  */
  double ped_lat = m_VRUdp->getPedPosition().lat;
  double ped_lon = m_VRUdp->getPedPosition().lon;
  libsumo::TraCIPosition new_pos = m_VRUdp->getPedPositionValue();
  double pos_diff = sqrt((new_pos.x-m_prev_position.x)*(new_pos.x-m_prev_position.x)+(new_pos.y-m_prev_position.y)*(new_pos.y-m_prev_position.y));
  data_pos="[DISTANCE] PrevLat="+std::to_string(m_prev_lat)+" PrevLon="+std::to_string(m_prev_lon)+" CurrLat="+std::to_string(ped_lat)+" CurrLon="+std::to_string(ped_lon)+" PosDiff="+std::to_string(pos_diff)+"\n";
  if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
        if (m_T_next_dcc == -1 || now - lastVamGen >= m_T_next_dcc)
        {
          m_N_GenVam_red = 0;

          m_trigg_cond = POSITION_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_pos_sent++;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        }
        else
        {
          data_dcc += "[DISTANCE] verified but [DCC] doesn't allow sending\n";
        } 
      }
      else{
          m_N_GenVam_red++;
          vamredmit_verified = true;
          // condition_verified = true;
        }
      
    }

  /* 1c)
   * The absolute difference between the current speed of the originating ITS-S
   * and the speed included in the VAM previously transmitted by the originating
   * ITS-S exceeds 0,5 m/s.
  */
  double speed_diff = m_VRUdp->getPedSpeedValue () - m_prev_speed;
  data_speed="[SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)+" PrevSpeed="+std::to_string(m_prev_speed)+" CurrSpeed="+std::to_string(m_VRUdp->getPedSpeedValue ())+" SpeedDiff="+std::to_string(speed_diff)+"\n";
  if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
        if (m_T_next_dcc == -1 || now - lastVamGen >= m_T_next_dcc)
        {
          m_N_GenVam_red = 0;

          m_trigg_cond = SPEED_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_speed_sent++;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        }
        else
        {
          data_dcc += "[SPEED] verified but [DCC] doesn't allow sending\n";
        } 
      }
      else{
        m_N_GenVam_red++;
        vamredmit_verified = true;
        // condition_verified = true;
      }
    }

  // Computation of the longitudinal safe distance
  m_long_safe_d = abs(m_VRUdp->getPedSpeedValue ()*(T_GenVamMax_ms/1000));

  // Get the minimum distance of other vehicles/pedestrians from the current pedestrian
  m_min_dist = m_VRUdp->get_min_distance (m_LDM);

  /* 1d)
   * If the longitudinal distance between the originating ITS-S and the nearest pedestrian/vehicle
   * is smaller than v/t, where v is the current longitudinal speed of the originating ITS-S
   * and t is the maximum time interval between the generation of two consecutive VAMs, the lateral
   * distance smaller than 2 m and the vertical distance smaller than 5 m, a VAM must be transmitted
  */

  if (m_min_dist.size() != 0)
    {
      data_safed = "[SAFE DISTANCES] LongSafeDist="+std::to_string(m_long_safe_d)+" LatSafeDist="+std::to_string(m_lat_safe_d)+" VertSafeDist="+std::to_string(m_vert_safe_d)+" MinLongDistVeh="+printMinDist(m_min_dist[1].longitudinal)+" MinLatDistVeh="+printMinDist(m_min_dist[1].lateral)+" MinVertDistVeh="+printMinDist(m_min_dist[1].vertical)+" MinLongDistPed="+printMinDist(m_min_dist[0].longitudinal)+" MinLatDistPed="+printMinDist(m_min_dist[0].lateral)+" MinVertDistPed="+printMinDist(m_min_dist[0].vertical)+"\n";
    }
  else
    {
      data_safed = "[SAFE DISTANCES] LongSafeDist="+std::to_string(m_long_safe_d)+" LatSafeDist="+std::to_string(m_lat_safe_d)+" VertSafeDist="+std::to_string(m_vert_safe_d)+" MinLongDistVeh="+printMinDist(0.0)+" MinLatDistVeh="+printMinDist(0.0)+" MinVertDistVeh="+printMinDist(0.0)+" MinLongDistPed="+printMinDist(0.0)+" MinLatDistPed="+printMinDist(0.0)+" MinVertDistPed="+printMinDist(0.0)+"\n";
    }
  if (!m_min_dist.empty() && !condition_verified && m_min_dist[1].longitudinal < m_long_safe_d && m_min_dist[1].lateral < m_lat_safe_d && m_min_dist[1].vertical < m_vert_safe_d)
    {
      m_N_GenVam_red = 0;

      m_trigg_cond = SAFE_DISTANCES;
      m_min_dist[1].safe_dist = true;
      vam_error = generateAndEncodeVam ();
      if(vam_error==VAM_NO_ERROR)
        {
          condition_verified = true;
          m_safedist_sent++;
        } else {
          NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
        }
    } else{
      if(!m_min_dist.empty() && !condition_verified && m_min_dist[0].longitudinal < m_long_safe_d && m_min_dist[0].lateral < m_lat_safe_d && m_min_dist[0].vertical < m_vert_safe_d)
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
              vamredmit_verified = true;
              // condition_verified = true;
            }
        }
    }

    // Clean the TIP map from old data
    double now_ms = Simulator::Now().GetMilliSeconds();
    cleanTIPMap(now_ms);
    searchFirstEightTIPs();
    if (m_tip_array.size() > 0) {
      data_tip += "[TIP] TIPs found in the TIP map that require a VAM to be sent:\n";
      if (!condition_verified && !vamredmit_verified) {
        if (!redundancy_mitigation && (m_N_GenVam_red == 0 || m_N_GenVam_red == m_N_GenVam_max_red)) {
          if (m_T_next_dcc == -1 || now - lastVamGen >= m_T_next_dcc) {
            m_N_GenVam_red = 0;

            m_trigg_cond = TIP_TRIGGER;
            vam_error = generateAndEncodeVam();
            if (vam_error == VAM_NO_ERROR) {
              condition_verified = true;
              m_tip_sent++;
            } else {
              std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
            }
          }
        } else {
          m_N_GenVam_red++;
          vamredmit_verified = true;
        }
      }
    } else {
      // Create the data for the log print
      data_tip += "[TIP] No TIPs found in the TIP map that require a VAM to be sent\n";
    }

  /* 2)
   * The time elapsed since the last VAM generation is equal to or greater than T_GenVam
  */
  long time_difference = now - lastVamGen;
  data_time="[TIME] Timestamp="+std::to_string(now)+" LastVAMSent="+std::to_string(lastVamGen)+" TimeThreshold="+std::to_string(m_T_GenVam_ms)+" TimeDiff="+std::to_string(time_difference)+" TimeNextVAM="+std::to_string(m_T_GenVam_ms - time_difference)+"\n";
  if(!condition_verified && (now-lastVamGen>=m_T_GenVam_ms))
    {
      if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
        if (m_T_next_dcc == -1 || now - lastVamGen >= m_T_next_dcc)
        {
          m_N_GenVam_red = 0;
          m_trigg_cond = MAX_TIME_ELAPSED;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_time_sent++;
            } else {
              NS_LOG_ERROR("Cannot generate VAM. Error code: "<<vam_error);
            }
        }
        else
        {
          data_dcc += "[TIME] verified but [DCC] doesn't allow sending\n";
        }
          
      } else{
          m_N_GenVam_red++;
          vamredmit_verified = true;
          // condition_verified = true;
      }
    }

  data_vamredmit = "[REDUNDANCY MITIGATION] numSkipVAMsForRedMitMax="+std::to_string(m_N_GenVam_max_red)+" numSkipVAMsForRedMit="+std::to_string(m_N_GenVam_red)+" TimestampLastVAMGen="+std::to_string(lastVamGen)+" TimeIntervalSinceLastVAMGen="+std::to_string(time_difference)+"\n";

  int n=0;
  write_log_triggering (condition_verified, vamredmit_verified, head_diff, pos_diff, speed_diff, time_difference, data_head, data_pos, data_speed, data_safed, data_tip, data_time, data_vamredmit, data_dcc);

  if((m_VRU_clust_state==VRU_IDLE || m_VRU_clust_state==VRU_ACTIVE_STANDALONE || m_VRU_clust_state==VRU_ACTIVE_CLUSTER_LEADER) && m_VRU_role==VRU_ROLE_ON)
    m_event_vamCheckConditions = Simulator::Schedule (MilliSeconds(m_T_CheckVamGen_ms), &VRUBasicService::checkVamConditions, this);
}

void VRUBasicService::computeLongAcceleration(){
  m_long_acceleration = m_VRUdp->getLongAcceleration ();

  m_event_computeLongAcceleration = Simulator::Schedule (Seconds(m_computationT_acc_ms), &VRUBasicService::computeLongAcceleration, this);
}

bool VRUBasicService::checkVamRedundancyMitigation(){
  bool redundancy_mitigation = false;
  int64_t now = computeTimestampUInt64 ()/NANO_TO_MILLI;
  std::vector<LDM::returnedVehicleData_t> selectedStations;

  VRUdp_position_latlon_t ped_pos = m_VRUdp->getPedPosition ();
  double ped_speed = m_VRUdp->getPedSpeedValue ();
  double ped_heading = m_VRUdp->getPedHeadingValue ();
  ped_heading += (ped_heading>180.0) ? -360.0 : (ped_heading<-180.0) ? 360.0 : 0.0;

  if(now-lastVamGen < m_N_GenVam_max_red*5000){
      if (m_LDM != nullptr)
        {
          m_LDM->rangeSelect (4,ped_pos.lat,ped_pos.lon,selectedStations);
        }

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
  asn1cpp::setField(vam->header.protocolVersion , 3);
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
  vam_mandatory_data = m_VRUdp->getVAMMandatoryData();

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

  /* Fill the lowFrequencyContainer */
  now = computeTimestampUInt64 ()/NANO_TO_MILLI;
  if(m_last_vam_gen_LF == -1 || now - m_last_vam_gen_LF >= m_T_GenVamLFMin_ms) {
    if (m_stationtype == StationType_cyclist)
      asn1cpp::setField(vam->vam.vamParameters.vruLowFrequencyContainer->profileAndSubprofile.present, VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle);
    else if (m_stationtype == StationType_pedestrian)
      asn1cpp::setField(vam->vam.vamParameters.vruLowFrequencyContainer->profileAndSubprofile.present, VruProfileAndSubprofile_PR_pedestrian);
    asn1cpp::setField(vam->vam.vamParameters.vruLowFrequencyContainer->sizeClass, VruSizeClass_medium);
    // We do not set the exteriorLights field, as it is not used in the current implementation with pedestrians and cyclists.
    // If needed, it can be set here using the following line: asn1cpp::setField(vam->vam.vamParameters.vruLowFrequencyContainer->exteriorLights);

    // Store the time in which the last VAM LF has been generated and successfully sent
    m_last_vam_gen_LF = now;
  }

  long numberOfTIPs = 0;
  if (m_tip_array.size() > 0) {
    numberOfTIPs = m_tip_array.size();
    auto TIP_container = asn1cpp::makeSeq (VruMotionPredictionContainer);
    auto TIPs = asn1cpp::makeSeq (SequenceOfTrajectoryInterceptionIndication);
    for (auto it = m_tip_array.begin(); it != m_tip_array.end(); ++it) {
      uint64_t stationId = std::get<0>(*it);
      double tip = std::get<1>(*it);
      auto TIP = asn1cpp::makeSeq(TrajectoryInterceptionIndication);
      asn1cpp::setField(TIP->subjectStation, stationId);
      long tip_scaled = std::lround(tip * 50.0);
      asn1cpp::setField(TIP->trajectoryInterceptionProbability, tip_scaled);
      asn1cpp::setField(TIP->trajectoryInterceptionConfidence, TrajectoryInterceptionConfidence_above90Percent);
      asn1cpp::sequenceof::pushList(*TIPs, TIP);
    }
    // Set the TIPs in the motionPredictionContainer
    asn1cpp::setField(TIP_container->trajectoryInterceptionIndication, TIPs);
    // Set the motionPredictionContainer in the VAM
    asn1cpp::setField(vam->vam.vamParameters.vruMotionPredictionContainer, TIP_container);
  }

  // Store all the "previous" values used in checkVamConditions()
  m_prev_position = m_VRUdp->getPedPositionValue ();
  m_prev_lat = m_VRUdp->getPedPosition().lat;
  m_prev_lon = m_VRUdp->getPedPosition().lon;
  m_prev_speed = m_VRUdp->getPedSpeedValue ();
  m_prev_heading = m_VRUdp->getPedHeadingValue ();
  // Previous TIPs are updated in the map when a VAM is sent, so no need to update them here
  for (const auto& entry : m_tip_array) {
    uint64_t id = std::get<0>(entry);
    double tip = std::get<1>(entry);
    updatePreviousSentTIPOnMap(id, tip);
  }

  /* VAM encoding */
  std::string encode_result = asn1cpp::uper::encode(vam);

  if(encode_result.size()<1)
  {
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
  std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, 0, MessageId_vam);
  GNDataConfirm_t dataConfirm = std::get<0>(status);
  MessageId_t message_id = std::get<1>(status);
  /* Update the VAM statistics */
  if(dataConfirm == ACCEPTED) {
      if (message_id == MessageId_vam) m_vam_sent++;
    }

  // Estimation of the transmission time
  m_last_transmission = (double) Simulator::Now().GetMilliSeconds();

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
  VRUdp_position_latlon_t pos_ped = m_VRUdp->getPedPosition ();

  // Store the data of interest in the .csv file
  std::ofstream csv_VAM_ofstream;
  if(m_VAM_metrics && (m_station_id==1000)){
      csv_VAM_ofstream.open(m_csv_file_name,std::ofstream::out | std::ofstream::app);
      csv_VAM_ofstream << m_station_id << "," << now << "," << time_elapsed << "," << m_trigg_cond << ",";
      if(m_trigg_cond == SAFE_DISTANCES){
          if(m_min_dist[0].safe_dist)
            csv_VAM_ofstream << m_min_dist[0].ID;
          else
            csv_VAM_ofstream << m_min_dist[1].ID;
      } else {
        csv_VAM_ofstream << -1;
      }
      csv_VAM_ofstream << "," << std::setprecision (12) << pos_ped.lat << "," << std::setprecision (12) << pos_ped.lon << "," <<
                       ( (double) vam_mandatory_data.speed.getValue())/CENTI << std::endl;
      csv_VAM_ofstream.close ();
  }

  return errval;
}

uint64_t VRUBasicService::terminateDissemination(){
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

void VRUBasicService::addNewTIPToMap(uint64_t id, double time, double tip) {
  if (m_tip_map.find(id) == m_tip_map.end()) {
    // Set the first previous TIP to 0
    m_tip_map[id] = std::make_tuple(time, tip, 0);
  } else {
    // Preserve the previous sent tip and update with the new tip
    double previous_sent_tip = std::get<2>(m_tip_map[id]);
    m_tip_map[id] = std::make_tuple(time, tip, previous_sent_tip);
  }
}

void VRUBasicService::updatePreviousSentTIPOnMap(uint64_t id, double previous_sent_tip) {
  // Preserve the information about the last computed TIP
  double time = std::get<0>(m_tip_map[id]);
  double tip = std::get<1>(m_tip_map[id]);
  m_tip_map[id] = std::make_tuple(time, tip, previous_sent_tip);
}

void VRUBasicService::searchFirstEightTIPs() {
  // Find the first weight station IDs and the associated TIP
  // Focus on the difference between latest registered TIP and previous TIP (>= DELTA_TIP)
  m_tip_array.clear();
  for (const auto& entry : m_tip_map) {
    uint64_t id = entry.first;
    double time = std::get<0>(entry.second);
    double tip = std::get<1>(entry.second);
    double previous_sent_tip = std::get<2>(entry.second);
    if (tip - previous_sent_tip >= DELTA_TIP) {
      m_tip_array.push_back(std::make_tuple(id, tip));
    }
  }
  // Sort the vector based on the TIP values in descending order
  std::sort(m_tip_array.begin(), m_tip_array.end(), [](const std::tuple<uint64_t, double>& a, const std::tuple<uint64_t, double>& b) {
    return std::get<1>(a) > std::get<1>(b);
  });
  // Keep only the first 8 elements if there are more than 8
  if (m_tip_array.size() > 8) {
    m_tip_array.resize(8);
  }
}

void VRUBasicService::cleanTIPMap(double time) {
  for (auto it = m_tip_map.begin(); it != m_tip_map.end();) {
      double tmp_time = std::get<0>(it->second);
      
      if (time - tmp_time >= MAX_TIP_MAP_TIME) {
          it = m_tip_map.erase(it); 
      } else {
          ++it;
      }
  }
}

void VRUBasicService::VRUreceivedNewMessageForTIP(BTPDataIndication_t dataIndication, bool veh) {
  uint8_t *buffer;
  buffer=(uint8_t *)malloc((dataIndication.data->GetSize ())*sizeof(uint8_t));
  dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
  std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());
  vehicleData_t vehdata;
  if (veh) {
      asn1cpp::Seq<CAM> decoded_cam = asn1cpp::uper::decodeASN(packetContent, CAM);

      vehdata.stationID = decoded_cam->header.stationId;

      // ASN.1 latitude/longitude -> degrees
      vehdata.lat =
          static_cast<double>(decoded_cam->cam.camParameters.basicContainer.referencePosition.latitude) / static_cast<double>(DOT_ONE_MICRO);

      vehdata.lon =
          static_cast<double>(decoded_cam->cam.camParameters.basicContainer.referencePosition.longitude) / static_cast<double>(DOT_ONE_MICRO);

      // ASN.1 HeadingValue: 0.1 degree -> degrees
      vehdata.heading =
          static_cast<double>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue) / static_cast<double>(DECI);

      // ASN.1 SpeedValue: 0.01 m/s -> m/s
      vehdata.speed_ms =
          static_cast<double>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue) / static_cast<double>(CENTI);
  } else {
      asn1cpp::Seq<VAM> decoded_vam = asn1cpp::uper::decodeASN(packetContent, VAM);

      vehdata.stationID = decoded_vam->header.stationId;

      // ASN.1 latitude/longitude -> degrees
      vehdata.lat =
          static_cast<double>(decoded_vam->vam.vamParameters.basicContainer.referencePosition.latitude) / static_cast<double>(DOT_ONE_MICRO);

      vehdata.lon =
          static_cast<double>(decoded_vam->vam.vamParameters.basicContainer.referencePosition.longitude) / static_cast<double>(DOT_ONE_MICRO);

      // ASN.1 HeadingValue: 0.1 degree -> degrees
      vehdata.heading =
          static_cast<double>(decoded_vam->vam.vamParameters.vruHighFrequencyContainer.heading.value) / static_cast<double>(DECI);

      // ASN.1 SpeedValue: 0.01 m/s -> m/s
      vehdata.speed_ms =
          static_cast<double>(decoded_vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue) / static_cast<double>(CENTI);
  }
 auto [ttc, stc] = compute_ttc_stc(vehdata);
  if (ttc > 0 && stc > 0) {
    // If the TTC is below the configured threshold, and the STC is below the configured threshold, then we can trigger a VAM
    if (ttc < m_TTC_max && stc < m_STC_min) {
      std::string tip_modality = m_tip_modality;
      double tip = -1;
      if (tip_modality == "exp") {
        tip = tip_exponential(ttc, m_STC_min, m_TTC_k);
      } else if (tip_modality == "gaus") {
        tip = tip_gaussian(ttc, m_TTC_sigma);
      } else {
        std::cerr << "[ERROR] Unknown TIP modality: " << tip_modality << std::endl;
      }
      if (tip >= 0) {
        double now_ms = Simulator::Now().GetMilliSeconds(); // Convert to milliseconds
        addNewTIPToMap(vehdata.stationID, now_ms, tip);
      }
    }
  }
}

std::tuple<double, double> 
VRUBasicService::compute_ttc_stc(vehicleData_t vehdata) {
	double ttc = -1.0;
	double stc = -1.0;
	// Extract data from the VDP
	VRUdp_position_latlon_t ped_pos_lat_lon = m_VRUdp->getPedPosition();
  VRUdp::VRUDP_position_cartesian_t ped_pos_xy = m_VRUdp->getXY(ped_pos_lat_lon.lon, ped_pos_lat_lon.lat);
	double speed_value = m_VRUdp->getPedSpeedValue();
	double ped_heading = m_VRUdp->getPedHeadingValue();

	// Remote vehicle position in XYZ
	double node_lat = vehdata.lat;
	double node_lon = vehdata.lon;
	VRUdp::VRUDP_position_cartesian_t node_pos_xy = m_VRUdp->getXY(node_lon, node_lat);
  double node_heading = vehdata.heading;

	// Same "no heading available at all" bail-out as get_min_distance
	if(node_heading == HeadingValue_unavailable && ped_heading == HeadingValue_unavailable) {
	    return {ttc, stc};
	  } else {
		// Same fallback: borrow the ego heading if the remote one is missing and vice versa
		if(node_heading == HeadingValue_unavailable) {
			node_heading = ped_heading;
		} else if (ped_heading == HeadingValue_unavailable) {
			ped_heading = node_heading;
		}

		// Relative position vector, vehicle w.r.t. VRU
		double dx = node_pos_xy.x - ped_pos_xy.x;
		double dy = node_pos_xy.y - ped_pos_xy.y;
		double distance = sqrt(dx*dx + dy*dy);

		// Heading -> velocity components on the x=East, y=North axes
		double ped_heading_rad = ped_heading * (3.14159265358979323846 / 180.0);
    double node_heading_rad = node_heading * (3.14159265358979323846 / 180.0);

		double vx_veh = vehdata.speed_ms * sin(node_heading_rad);
		double vy_veh = vehdata.speed_ms * cos(node_heading_rad);

		double vx_ped = speed_value * sin(ped_heading_rad);
		double vy_ped = speed_value * cos(ped_heading_rad);

		// Relative velocity of the vehicle w.r.t. the VRU
		double dvx = vx_veh - vx_ped;
		double dvy = vy_veh - vy_ped;

		// Closing speed = -(d/dt) of the distance between the two nodes
		double closing_speed = (distance > 0) ? -(dx*dvx + dy*dvy)/distance : 0;

		if(distance > 0 && closing_speed > 0) {
			ttc = distance / closing_speed;
		} else {
			// Not approaching each other (or already colocated) -> no meaningful TTC
			ttc = -1.0;
		}

		if(ttc >= m_TTC_max || ttc < 0) {
			// No collision predicted (diverging, parallel courses, or already colocated)
			return {-1.0, -1.0};
		}

		// Linearly project the relative position (vehicle - VRU) forward to t = ttc.
		// This is the SAME linear-motion assumption already baked into the ttc formula,
		// so this is consistent with it rather than an extra approximation.
		double dx_at_ttc = dx + dvx * ttc;
		double dy_at_ttc = dy + dvy * ttc;

		stc = sqrt(dx_at_ttc*dx_at_ttc + dy_at_ttc*dy_at_ttc);

		if(stc > m_STC_min) {
			// The projected separation at the predicted collision time is below the minimum threshold -> treat this as a "no collision" case
			return {-1.0, -1.0};
		}

		return {ttc, stc};
	}
}

}
