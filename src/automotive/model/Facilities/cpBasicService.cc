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
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include <opencv2/core/types.hpp>
#include "cpBasicService.h"
#include "ns3/snr-tag.h"
#include "ns3/sinr-tag.h"
#include "ns3/rssi-tag.h"
#include "ns3/timestamp-tag.h"
#include "ns3/rsrp-tag.h"
#include "ns3/size-tag.h"
#include "ns3/dbscan.h"


namespace ns3 {

  NS_LOG_COMPONENT_DEFINE("CPBasicService");

  CPBasicService::CPBasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket_tx=NULL;
    m_btp = NULL;
    m_LDM = NULL;
    m_real_time=false;

    // Setting a default value of m_T_CheckCpmGen_ms equal to 100 ms (i.e. T_GenCpmMin_ms)
    m_T_CheckCpmGen_ms=T_GenCpmMin_ms;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCpm_ms=T_GenCpmMax_ms;

    lastCpmGen=-1;
    lastCpmGenLowFrequency=-1;
    lastCpmGenSpecialVehicle=-1;

    m_T_LastSensorInfoContainer = -1;

    m_N_GenCpmMax=1000;
    m_N_GenCpm=T_GenCpmMin_ms;

    m_vehicle=true;
    m_redundancy_mitigation = true;

    m_cpm_sent=0;
    m_csv_name = "Output";
//    m_csv_ofstream_cpm.open (m_csv_name+"-"+std::to_string(m_station_id)+"-CPM.csv",std::ofstream::trunc);
//    m_csv_ofstream_cpm << "timestamp,numberOfObjects,numberOfVRU,averageDistanceBetweenVRU,CPMsize" << std::endl;

  }

  void
  CPBasicService::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
    m_btp->setStationID(fixed_stationid);
  }

  void
  CPBasicService::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    m_btp->setStationType(fixed_stationtype);
  }

  void
  CPBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    m_btp->setStationProperties(fixed_stationid,fixed_stationtype);
  }

  void
  CPBasicService::setSocketRx (Ptr<Socket> socket_rx)
  {
    m_btp->setSocketRx(socket_rx);
    m_btp->addCPMRxCallback (std::bind(&CPBasicService::receiveCpm,this,std::placeholders::_1,std::placeholders::_2));
  }

  void
  CPBasicService::initDissemination()
  {
    std::srand(Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);
    m_event_cpmSend = Simulator::Schedule (Seconds(desync), &CPBasicService::generateAndEncodeCPM, this);
  }

  double
  CPBasicService::cartesian_dist(double lon1, double lat1, double lon2, double lat2)
  {
    libsumo::TraCIPosition pos1,pos2;
    pos1 = m_client->TraCIAPI::simulation.convertLonLattoXY(lon1,lat1);
    pos2 = m_client->TraCIAPI::simulation.convertLonLattoXY(lon2,lat2);
    return sqrt((pow((pos1.x-pos2.x),2)+pow((pos1.y-pos2.y),2)));
  }

  bool
  CPBasicService::checkCPMconditionsTypeA(std::vector<LDM::returnedVehicleData_t>::iterator PO_data) {
      /*Perceived Object Container Inclusion Management as mandated by TS 103 324 Section 6.1.2.3*/
      std::map<uint64_t, PHData_t> phPoints = PO_data->phData.getPHpoints();
      PHData_t previousCPM;

      if (!(PO_data->vehData.lastCPMincluded.isAvailable ())){
          std::cout << "Cond1-lasCPMincluded not available - typeA - ID: " << PO_data->vehData.stationID << std::endl;
          return true;
      }
      /* 1.a The object has first been detected by the perception system after the last CPM generation event.*/
      if ((PO_data->phData.getSize() == 1) && (PO_data->phData.getPHpoints().begin()->first > lastCpmGen*1000)) {
          std::cout << "First detected after last CPM event - ID: " << PO_data->vehData.stationID << std::endl;
          return true;
      }
      /* Get the last position of the reference point of this object lastly included in a CPM from the object pathHistory*/
      std::map<uint64_t, PHData_t>::reverse_iterator it = phPoints.rbegin();
      it++;
      for (auto fromPrev = it; fromPrev != phPoints.rend(); fromPrev++) {
          if (fromPrev->second.CPMincluded == true) {
              previousCPM = fromPrev->second;
          }
      }
      /* 1.b If the object list contains at least one object of Type-A which has not been included in a CPM for a time
       * equal or larger than T_GenCpmMax/2, all objects of Type-A should be included in the currently generated CPM */
//      if(PO_data->vehData.lastCPMincluded.isAvailable ()) {
//          if(PO_data->vehData.lastCPMincluded.getData() < ((computeTimestampUInt64 ()/NANO_TO_MILLI)-(m_N_GenCpmMax/2)))
//          {
//              std::cout << "New object to be included in the CPM after 500ms - ID: " << PO_data->vehData.stationID
//                        << std::endl;
//                /*TODO: need to create a flag to include all objects of Type-A in the CPM if we enter in this condition*/
//              m_send_all_typeA = false;
//              return true;
//          }
//      }
      return false;
  }

  bool
  CPBasicService::checkCPMconditionsTypeB(std::vector<LDM::returnedVehicleData_t>::iterator PO_data)
  {
    /*Perceived Object Container Inclusion Management as mandated by TS 103 324 Section 6.1.2.3*/
    std::map<uint64_t, PHData_t> phPoints = PO_data->phData.getPHpoints ();
    PHData_t previousCPM;

      if (!(PO_data->vehData.lastCPMincluded.isAvailable ())){
          std::cout << "Cond1-lasCPMincluded not available - typeB - ID: " << PO_data->vehData.stationID << std::endl;
          return true;
      }
    /* 1.a The object has first been detected by the perception system after the last CPM generation event.*/
    if((PO_data->phData.getSize ()==1) && (PO_data->phData.getPHpoints ().begin ()->first > lastCpmGen*1000)) {
        std::cout << "Cond1 - typeB - ID: " << PO_data->vehData.stationID << std::endl;
        return true;
    }
    /* Get the last position of the reference point of this object lastly included in a CPM from the object pathHistory*/
    std::map<uint64_t, PHData_t>::reverse_iterator it = phPoints.rbegin ();
//    it ++;
    for(auto fromPrev = it; fromPrev!=phPoints.rend(); fromPrev++)
      {
        if (fromPrev->second.CPMincluded == true)
          {
            previousCPM = fromPrev->second;
            break;
          }
      }
    /* 1.b The Euclidian absolute distance between the current estimated position of the reference point of the
     * object and the estimated position of the reference point of this object lastly included in a CPM exceeds
     * 4 m. */
    if(m_vdp->getCartesianDist (previousCPM.lon,previousCPM.lat,PO_data->vehData.lon,PO_data->vehData.lat) > 4.0) {
        std::cout << "Cond2 - typeB - ID: " << PO_data->vehData.stationID << std::endl;
        return true;
    }
    /* 1.c The difference between the current estimated absolute speed of the reference point of the object and the
     * estimated absolute speed of the reference point of this object lastly included in a CPM exceeds 0,5 m/s. */
    if(abs(previousCPM.speed_ms - PO_data->vehData.speed_ms) > 0.5) {
        std::cout << "Cond3 - typeB - ID: " << PO_data->vehData.stationID << std::endl;
        return true;
    }
    /* 1.d The difference between the orientation of the vector of the current estimated absolute velocity of the
     * reference point of the object and the estimated orientation of the vector of the absolute velocity of the
     * reference point of this object lastly included in a CPM exceeds 4 degrees. */
    if(abs(previousCPM.heading - PO_data->vehData.heading) > 4) {
        std::cout << "Cond4 - typeB - ID: " << PO_data->vehData.stationID << std::endl;
        return true;
    }
    /* 1.e The time elapsed since the last time the object was included in a CPM exceeds T_GenCpmMax. */
    if(PO_data->vehData.lastCPMincluded.isAvailable ())
      {
        if(PO_data->vehData.lastCPMincluded.getData() < ((computeTimestampUInt64 ()/NANO_TO_MILLI)-m_N_GenCpmMax)){
            std::cout << "Cond5 - typeB - ID: " << PO_data->vehData.stationID << std::endl;
          return true;
         }
      }
    return false;
  }

  void
  CPBasicService::generateAndEncodeCPM()
  {
    VDP::CPM_mandatory_data_t cpm_mandatory_data;
    Ptr<Packet> packet;

    BTPDataRequest_t dataRequest = {};

    int64_t now = computeTimestampUInt64 () / NANO_TO_MILLI;

    std::string encode_result;

    long numberOfPOs = 0;
    long container_counter = 1;
    long numberofVRUs = 0;
    long numberofVehicles = 0;
    /* Collect data for mandatory containers */
    auto cpm = asn1cpp::makeSeq (CollectivePerceptionMessage);

    if (bool (cpm) == false)
      {
        NS_LOG_ERROR ("Warning: unable to encode CPM.");
        return;
      }
      std::map<uint64_t,bool> clusterCPM_this_loop; //map to indicate if a cluster was already sent on a CPM
      std::map<size_t, ClusterInfo> cluster_map {};
    // retrieve number of clusters and initialize map to false
      if (m_clustering){
          if (m_LDM->getClusterMap(cluster_map)) {
              for (const auto &pair: cluster_map) {
                  clusterCPM_this_loop[pair.first] = false;
              }
          }
      }

    //Schedule new CPM
    m_event_cpmSend = Simulator::Schedule (MilliSeconds (m_N_GenCpm),
                                           &CPBasicService::generateAndEncodeCPM, this);

    auto CPMcontainers = asn1cpp::makeSeq (WrappedCpmContainers);
    auto POsContainer = asn1cpp::makeSeq (PerceivedObjectContainer);
    auto CPM_POs = asn1cpp::makeSeq (PerceivedObjects);
    //std::cout << "[CPM] Vehicle " << m_station_id << " position: " << m_vdp->getPositionXY().x << " " << m_vdp->getPositionXY().y << std::endl;
    std::vector<vehicleData_t> LDM_VRUs;
    std::vector<vehicleData_t> clusterLDM;
    std::vector<uint64_t> vruIDs;
    bool sendAll = false;


    if (m_LDM != nullptr)
      {
        std::vector<LDM::returnedVehicleData_t> LDM_POs;
        std::vector<LDM::returnedVehicleData_t> LDM_POs_TypeA;
        if (m_LDM->getAllPOs (LDM_POs)) // If there are any POs in the LDM
          {
//            std::cout << "Generating and encoding CPM...for " << m_station_id <<std::endl;
            /* Fill Perceived Object Container as detailed in ETSI TS 103 324, Section 7.1.8 */
            std::vector<LDM::returnedVehicleData_t>::iterator it;
            //go over all POs_TypeA
            /* ETSI TS 103 324 V2.1.1
            If the object list contains at least one object of Type-A which has not been included in a CPM
            for a time equal or larger than T_GenCpmMax/2, all objects of Type-A should be included in the
            currently generated CPM */

            std::map<uint64_t, uint64_t > id_lastIncluded_map;
            if (m_LDM->getAllPOs_TypeA(LDM_POs_TypeA)){
                for (it = LDM_POs_TypeA.begin (); it != LDM_POs_TypeA.end (); it++){
                    id_lastIncluded_map[it->vehData.stationID] = it->vehData.lastCPMincluded.getData();
                    if (it->vehData.lastCPMincluded.isAvailable ()) {
                        if (it->vehData.lastCPMincluded.getData() <=
                            ((computeTimestampUInt64() / NANO_TO_MILLI) - (m_N_GenCpmMax / 2))) {
                            sendAll = true;
                            std::cout << "New object to be included in the CPM after 500ms - ID: "
                                      << it->vehData.stationID << " - last time included: " << it->vehData.lastCPMincluded.getData()
                                      << std::endl;
                            break;
                        }
                    }
                }
            }

            for (it = LDM_POs.begin (); it != LDM_POs.end (); it++)
              {
                // --- Determine if PO is in a cluster ---
                unsigned long current_cluster_id = 0;
                ClusterInfo *current_cluster_info = nullptr;
                bool POinCluster {false};
                if (m_clustering){
                      for (auto &pair : cluster_map) {
                          if (std::find(pair.second.IDs.begin(), pair.second.IDs.end(), it->vehData.stationID) != pair.second.IDs.end()){
                              current_cluster_id = pair.first;
                              current_cluster_info = &pair.second;
                              POinCluster = true;
                          }
                      }
                }
                //ignore VAM objects
                if (it->vehData.VAM == false) {
                    if (it->vehData.perceivedBy.getData() != (long) m_station_id)
                        continue;
                }else{
                    std::cout << "PO " << it->vehData.stationID << " from a VAM" << std::endl;
                    continue;
                }
                // check type-B
                if (it->vehData.itsType == itsType_vehicle || it->vehData.itsType == itsType_motorcycle) {
                    if (!checkCPMconditionsTypeB(it) && m_redundancy_mitigation)
                        continue;
                }
                else if (it->vehData.itsType == itsType_pedestrian || it->vehData.itsType == itsType_bicycle) {
                    if (!checkCPMconditionsTypeA(it) && m_redundancy_mitigation && !sendAll) {
                        if (!POinCluster) {
                            continue;
                        }
                    }
                }

                unsigned long ClusterID {0};
                int cardinalitySize {0};
                float center_x {0};
                float center_y {0};
                float radius {0};
                float clusterHeading {0};
                long clusterXspeed {0};
                long clusterYspeed {0};
                bool sendClusterInfo {false};
                bool sendPedestrianInfo {true};
                //std::map<size_t, ClusterInfo> cluster_map {};
                if (m_clustering){
                    long current_timestamp_ms = computeTimestampUInt64() / NANO_TO_MILLI;
                    if (current_cluster_info != nullptr){
                        std::cout << "Pedestrian ID " << it->vehData.stationID << " is in cluster: " << current_cluster_id << std::endl;
                        if (clusterCPM_this_loop[current_cluster_id]==true){
                            std::cout << "\nCluster " << current_cluster_id << " already included on this loop\n" << std::endl;
                            sendPedestrianInfo = false;
                        } else {
                            // check timestamp or sendAll flag
                            if (sendAll || (current_cluster_info->lastCPMincluded <=
                                            ((current_timestamp_ms) - (m_N_GenCpmMax / 2)))) {
                                sendClusterInfo = true;
                                sendPedestrianInfo = false;

                                ClusterID = current_cluster_id;
                                cardinalitySize = current_cluster_info->IDs.size();
                                center_x = current_cluster_info->center.x * 10; //units in the CPM are in 0.1m
                                center_y = current_cluster_info->center.y * 10;
                                radius = current_cluster_info->radius * 10;
                                //assign clusters info to the pedestrian responsible for the generation of this CPM
                                //so we can retrieve it later
                                clusterHeading = current_cluster_info->heading * 10;
                                clusterXspeed = std::cos(current_cluster_info->heading*(M_PI/180)) * current_cluster_info->speed_ms*CENTI;
                                clusterYspeed = std::sin(current_cluster_info->heading*(M_PI/180)) * current_cluster_info->speed_ms*CENTI;
                                std::cout << "Sending Cluster on CPM\n  ID: " << ClusterID << "\n  Size: " << cardinalitySize << std::endl;
//                                    std::cout << "xspeedabs =  " << it->vehData.xSpeedAbs << ", yspeedabs = " << it->vehData.ySpeedAbs << std::endl;

                                for (const auto &member_id : current_cluster_info->IDs) {
                                    m_LDM->updateCPMincluded(member_id, current_timestamp_ms);
                                }

                                //update CPMincluded for pedestrian inside the cluster
                            } else {
                                // Cluster info was sent recently
                                sendPedestrianInfo = false;
                                std::cout << "Cluster info for " << current_cluster_id << " already sent <"
                                          << current_timestamp_ms - current_cluster_info->lastCPMincluded << "ms ago." << std::endl;
                            }
                        }
                    }
                }
                if (!sendPedestrianInfo && !sendClusterInfo){
                    continue;
                }

                auto PO = asn1cpp::makeSeq (PerceivedObject);
                asn1cpp::setField (PO->objectId, it->vehData.stationID);
                long timeOfMeasurement =
                    (Simulator::Now ().GetMicroSeconds () - it->vehData.timestamp_us) /
                    1000; // time of measuremente in ms
                if (timeOfMeasurement > 1500)
                  timeOfMeasurement = 1500;
                asn1cpp::setField (PO->measurementDeltaTime, timeOfMeasurement);
                if (it->vehData.xDistAbs.isAvailable() && it->vehData.xDistAbs.getData() < 131071 &&
                        it->vehData.xDistAbs.getData() > -131072) {
                    asn1cpp::setField (PO->position.xCoordinate.value,
                                       it->vehData.xDistAbs.getData());
                } else{
                    asn1cpp::setField (PO->position.xCoordinate.value,
                                       it->vehData.xDistAbs.getData());
                }
                asn1cpp::setField (PO->position.xCoordinate.confidence,
                                   CoordinateConfidence_unavailable);
                if (it->vehData.yDistAbs.isAvailable() && it->vehData.yDistAbs.getData() < 131071 &&
                                                          it->vehData.yDistAbs.getData() > -131072) {
                    asn1cpp::setField (PO->position.yCoordinate.value,
                                       it->vehData.yDistAbs.getData());
                } else {
                    asn1cpp::setField (PO->position.yCoordinate.value,
                                       it->vehData.yDistAbs.getData());
                }
                asn1cpp::setField (PO->position.yCoordinate.confidence,
                                   CoordinateConfidence_unavailable);

                auto velocity = asn1cpp::makeSeq (Velocity3dWithConfidence);

                asn1cpp::setField (velocity->present,
                                   Velocity3dWithConfidence_PR_cartesianVelocity);
                auto cartesianVelocity = asn1cpp::makeSeq (VelocityCartesian);
                  if (!sendClusterInfo) {
                      if (it->vehData.xSpeedAbs.isAvailable()) {
                          asn1cpp::setField (cartesianVelocity->xVelocity.value,
                                             it->vehData.xSpeedAbs.getData());
                      } else {
                          asn1cpp::setField (cartesianVelocity->xVelocity.value,
                                             VelocityComponentValue_unavailable);
                      }
                      asn1cpp::setField (cartesianVelocity->xVelocity.confidence,
                                         SpeedConfidence_unavailable);
                      if (it->vehData.ySpeedAbs.isAvailable()) {
                          asn1cpp::setField (cartesianVelocity->yVelocity.value,
                                             it->vehData.ySpeedAbs.getData());
                      } else {
                          asn1cpp::setField (cartesianVelocity->yVelocity.value,
                                             VelocityComponentValue_unavailable);
                      }
                      asn1cpp::setField (cartesianVelocity->yVelocity.confidence,
                                         SpeedConfidence_unavailable);
                  } else {
                      if (it->vehData.xSpeedAbs.isAvailable()) {
                          asn1cpp::setField (cartesianVelocity->xVelocity.value,
                                             clusterXspeed);
                      } else {
                          asn1cpp::setField (cartesianVelocity->xVelocity.value,
                                             clusterXspeed);
                      }
                      asn1cpp::setField (cartesianVelocity->xVelocity.confidence,
                                         SpeedConfidence_unavailable);
                      if (it->vehData.ySpeedAbs.isAvailable()) {
                          asn1cpp::setField (cartesianVelocity->yVelocity.value,
                                             clusterYspeed);
                      } else {
                          asn1cpp::setField (cartesianVelocity->yVelocity.value,
                                             clusterYspeed);
                      }
                      asn1cpp::setField (cartesianVelocity->yVelocity.confidence,
                                         SpeedConfidence_unavailable);
                  }
                asn1cpp::setField (velocity->choice.cartesianVelocity, cartesianVelocity);
                asn1cpp::setField (PO->velocity, velocity);

                auto acceleration = asn1cpp::makeSeq (Acceleration3dWithConfidence);
                asn1cpp::setField (acceleration->present,
                                   Acceleration3dWithConfidence_PR_cartesianAcceleration);
                auto cartesianAcceleration = asn1cpp::makeSeq (AccelerationCartesian);
                if (it->vehData.xAccAbs.isAvailable() && it->vehData.xAccAbs.getData()<160 &&
                                                        it->vehData.xAccAbs.getData()>-161) {
                    asn1cpp::setField (cartesianAcceleration->xAcceleration.value,
                                       it->vehData.xAccAbs.getData());
                }
                else {
                    asn1cpp::setField (cartesianAcceleration->xAcceleration.value,
                                       AccelerationValue_unavailable);
                }
                asn1cpp::setField (cartesianAcceleration->xAcceleration.confidence,
                                   AccelerationConfidence_unavailable);
                if (it->vehData.yAccAbs.isAvailable() && it->vehData.yAccAbs.getData()<160 &&
                      it->vehData.yAccAbs.getData()>-161) {
                    asn1cpp::setField (cartesianAcceleration->yAcceleration.value,
                                       it->vehData.yAccAbs.getData());
                }    else {
                    asn1cpp::setField (cartesianAcceleration->yAcceleration.value,
                                       AccelerationValue_unavailable);
                }
                asn1cpp::setField (cartesianAcceleration->yAcceleration.confidence,
                                   AccelerationConfidence_unavailable);
                asn1cpp::setField (acceleration->choice.cartesianAcceleration,
                                   cartesianAcceleration);
                asn1cpp::setField (PO->acceleration, acceleration);

                //Only z angle
                auto angle = asn1cpp::makeSeq (EulerAnglesWithConfidence);
                if (!sendClusterInfo) {
                    if ((it->vehData.heading * DECI) < CartesianAngleValue_unavailable &&
                        (it->vehData.heading * DECI) > 0)
                        asn1cpp::setField (angle->zAngle.value, (it->vehData.heading * DECI));
                    else
                        asn1cpp::setField (angle->zAngle.value, CartesianAngleValue_unavailable);
                } else {
                    if (clusterHeading < CartesianAngleValue_unavailable &&
                        clusterHeading > 0)
                        asn1cpp::setField (angle->zAngle.value, static_cast<long>(clusterHeading));
                    else
                        asn1cpp::setField (angle->zAngle.value, CartesianAngleValue_unavailable);
                }
                asn1cpp::setField (angle->zAngle.confidence, AngleConfidence_unavailable);
                asn1cpp::setField (PO->angles, angle);
                auto OD1 = asn1cpp::makeSeq (ObjectDimension);
                if (it->vehData.vehicleLength.getData () < 1023 &&
                    it->vehData.vehicleLength.getData () > 0)
                  asn1cpp::setField (OD1->value, it->vehData.vehicleLength.getData ());
                else
                  asn1cpp::setField (OD1->value, 50); //usual value for SUMO vehicles
                asn1cpp::setField (OD1->confidence, ObjectDimensionConfidence_unavailable);
                asn1cpp::setField (PO->objectDimensionX, OD1);
                auto OD2 = asn1cpp::makeSeq (ObjectDimension);
                if (it->vehData.vehicleWidth.getData () < 1023 &&
                    it->vehData.vehicleWidth.getData () > 0)
                  asn1cpp::setField (OD2->value, it->vehData.vehicleWidth.getData ());
                else
                  asn1cpp::setField (OD2->value, 18); //usual value for SUMO vehicles
                asn1cpp::setField (OD2->confidence, ObjectDimensionConfidence_unavailable);
                asn1cpp::setField (PO->objectDimensionY, OD2);

                /*Rest of optional fields handling left as future work*/
//                auto classificationList = asn1cpp::makeSeq(ObjectClassDescription);
                auto classification = asn1cpp::makeSeq(ObjectClassWithConfidence);
//
//                //For vehicles
                if (it->vehData.itsType == itsType_vehicle) {
                    auto objectClass = asn1cpp::makeSeq(ObjectClass);
                    asn1cpp::setField(objectClass->present, ObjectClass_PR_vehicleSubClass);
                    asn1cpp::setField(objectClass->choice.vehicleSubClass, TrafficParticipantType_passengerCar);
                    asn1cpp::setField(classification->objectClass, objectClass);
                    std::cout << "Vehicle " << it->vehData.stationID << " included on CPM" << std::endl;
                  }
                else if (it->vehData.itsType == itsType_motorcycle){
                    auto objectClass = asn1cpp::makeSeq(ObjectClass);
                    asn1cpp::setField(objectClass->present, ObjectClass_PR_vehicleSubClass);
                    asn1cpp::setField(objectClass->choice.vehicleSubClass, TrafficParticipantType_unknown); // There is a constraint for the vehicleSubClass choice (ObjectClass.c). Value must be either 0 or within the range 5 to 11, or 14.
                    asn1cpp::setField(classification->objectClass, objectClass);
                    std::cout << "Motorcycle " << it->vehData.stationID << " included on CPM" << std::endl;
                }
                else if (it->vehData.itsType == itsType_pedestrian ){
                    if(!sendClusterInfo) {
//                        auto VruProfile = asn1cpp::makeSeq(VruProfileAndSubprofile);
//                        asn1cpp::setField(VruProfile->present, VruProfileAndSubprofile_PR_pedestrian);
//                        asn1cpp::setField(VruProfile->choice.pedestrian, VruSubProfilePedestrian_ordinary_pedestrian);
                        auto objectClass = asn1cpp::makeSeq(ObjectClass);
                        asn1cpp::setField(objectClass->present, ObjectClass_PR_vruSubClass);
                        asn1cpp::setField(objectClass->choice.vruSubClass.present, VruProfileAndSubprofile_PR_pedestrian);
                        asn1cpp::setField(objectClass->choice.vruSubClass.choice.pedestrian,VruSubProfilePedestrian_ordinary_pedestrian);
//                        asn1cpp::setField(objectClass->choice.vruSubClass, VruProfile);
                        asn1cpp::setField(classification->objectClass, objectClass);
                        LDM_VRUs.push_back(it->vehData);
                        numberofVRUs++;
                        vruIDs.push_back(it->vehData.stationID);
                        std::cout << "Pedestrian " << it->vehData.stationID << " included on CPM" << std::endl;
                    }
                    else {
                        auto coordinates = asn1cpp::makeSeq(CartesianPosition3d);
                        asn1cpp::setField(coordinates->xCoordinate,static_cast<long>(center_x));
                        asn1cpp::setField(coordinates->yCoordinate,static_cast<long>(center_y));
                        auto ShapeInfo = asn1cpp::makeSeq(CircularShape);
                        asn1cpp::setField(ShapeInfo->shapeReferencePoint,coordinates);
                        asn1cpp::setField(ShapeInfo->radius,static_cast<long>(radius));
                        auto clusterShape = asn1cpp::makeSeq(Shape);
                        asn1cpp::setField(clusterShape->present, Shape_PR_circular);
                        asn1cpp::setField(clusterShape->choice.circular, ShapeInfo);
                        auto clusterInfo = asn1cpp::makeSeq(VruClusterInformation);
                        asn1cpp::setField(clusterInfo->clusterId, ClusterID);
                        asn1cpp::setField(clusterInfo->clusterCardinalitySize,static_cast<long>(cardinalitySize));
                        asn1cpp::setField(clusterInfo->clusterBoundingBoxShape,clusterShape);
                        auto objectClass = asn1cpp::makeSeq(ObjectClass);
                        asn1cpp::setField(objectClass->present, ObjectClass_PR_groupSubClass);
                        asn1cpp::setField(objectClass->choice.groupSubClass , clusterInfo);
                        asn1cpp::setField(classification->objectClass, objectClass);
                        clusterCPM_this_loop[ClusterID] = true;
                        std::cout << "Cluster " << ClusterID << " included on CPM" << std::endl;
                    }
                }
                else if (it->vehData.itsType == itsType_bicycle){
                    auto VruProfile = asn1cpp::makeSeq(VruProfileAndSubprofile);
                    asn1cpp::setField(VruProfile->present, VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle);
                    asn1cpp::setField(VruProfile->choice.bicyclistAndLightVruVehicle, VruSubProfileBicyclist_bicyclist);
                    auto objectClass = asn1cpp::makeSeq(ObjectClass);
                    asn1cpp::setField(objectClass->present, ObjectClass_PR_vruSubClass);
                    asn1cpp::setField(objectClass->choice.vruSubClass, VruProfile);
                    asn1cpp::setField(classification->objectClass, objectClass);
                    LDM_VRUs.push_back(it->vehData);
                    numberofVRUs++;
                    vruIDs.push_back(it->vehData.stationID);
                    std::cout << "Bicycle " << it->vehData.stationID << " included on CPM" << std::endl;
                }


                asn1cpp::setField(classification->confidence, ConfidenceLevel_unavailable);
//                asn1cpp::sequenceof::pushList(*classificationList, classification);
//                asn1cpp::setField(PO->classification, classificationList);
                asn1cpp::sequenceof::pushList(PO->classification, classification);

                //Push Perceived Object to the container
                asn1cpp::sequenceof::pushList (*CPM_POs, PO);
                //Update the timestamp of the last time this PO was included in a CPM
                  if(!sendClusterInfo) {
                      m_LDM->updateCPMincluded(it->vehData.stationID,
                                               computeTimestampUInt64() / NANO_TO_MILLI);
                  }
                  else {
                      m_LDM->updateClusterCPMincluded(ClusterID,
                                               computeTimestampUInt64() / NANO_TO_MILLI);
                  }
                //Increase number of POs for the numberOfPerceivedObjects field in cpmParameters container
                numberOfPOs++;

              }
            if (numberOfPOs != 0)
              {
                asn1cpp::setField (POsContainer->perceivedObjects, CPM_POs);
                asn1cpp::setField (POsContainer->numberOfPerceivedObjects, numberOfPOs);
              }
          }
      }

    /* Fill the header */
    asn1cpp::setField (cpm->header.messageId, MessageId_cpm);
    asn1cpp::setField (cpm->header.protocolVersion, 2);
    asn1cpp::setField (cpm->header.stationId, m_station_id);

    /*
     * Compute the generationDeltaTime, "computed as the time corresponding to the
     * time of the reference position in the CPM, considered as time of the CPM generation.
     * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
     * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
     * generationDeltaTime = TimestampIts mod 65 536"
    */
//todo reference time
      uint64_t referenceTime64 = compute_timestampIts(m_real_time) % 65536;

// Convert to long (if within range) or unsigned long
      if (referenceTime64 <= LONG_MAX) {
          long referenceTimeLong = static_cast<long>(referenceTime64);
          asn1cpp::setField(cpm->payload.managementContainer.referenceTime, referenceTimeLong);
      } else {
          unsigned long referenceTimeULong = static_cast<unsigned long>(referenceTime64);
          asn1cpp::setField(cpm->payload.managementContainer.referenceTime, referenceTimeULong);
      }
//    asn1cpp::setField (cpm->payload.managementContainer.referenceTime, compute_timestampIts(m_real_time) % 65536);


    cpm_mandatory_data = m_vdp->getCPMMandatoryData ();

    /* Fill the managementContainer */
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.altitude.altitudeValue,
                       cpm_mandatory_data.altitude.getValue ());
    asn1cpp::setField (
        cpm->payload.managementContainer.referencePosition.altitude.altitudeConfidence,
        cpm_mandatory_data.altitude.getConfidence ());
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.latitude,
                       cpm_mandatory_data.latitude);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.longitude,
                       cpm_mandatory_data.longitude);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                           .semiMajorConfidence,
                       cpm_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                           .semiMinorConfidence,
                       cpm_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
    asn1cpp::setField (cpm->payload.managementContainer.referencePosition.positionConfidenceEllipse
                           .semiMajorOrientation,
                       cpm_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
    //TODO:  compute segmentInfo, get MTU and deal with needed segmentation

    /* Fill the originatingVehicleContainer */
    auto wrappedCpmContainer = asn1cpp::makeSeq (WrappedCpmContainer);
    asn1cpp::setField (wrappedCpmContainer->containerId, 1);
    auto originatingVehicleContainer = asn1cpp::makeSeq (OriginatingVehicleContainer);
    asn1cpp::setField (originatingVehicleContainer->orientationAngle.value,
                       cpm_mandatory_data.heading.getValue ());
    asn1cpp::setField (originatingVehicleContainer->orientationAngle.confidence,
                       cpm_mandatory_data.heading.getConfidence ());
    asn1cpp::setField (wrappedCpmContainer->containerData.present,
                       WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer);
    asn1cpp::setField (wrappedCpmContainer->containerData.choice.OriginatingVehicleContainer,
                       originatingVehicleContainer);
    asn1cpp::sequenceof::pushList (cpm->payload.cpmContainers, wrappedCpmContainer);

    /* Generate Sensor Information Container as detailed in ETSI TS 103 324, Section 6.1.2.2 */
    if (now - m_T_LastSensorInfoContainer >= m_T_AddSensorInformation)
      {
        auto CPMcontainer = asn1cpp::makeSeq (WrappedCpmContainer);
        asn1cpp::setField (CPMcontainer->containerId, 3);
        auto sensorInfoContainer = asn1cpp::makeSeq (SensorInformationContainer);

        //For now we only consider one sensor
        //We assume sensor fusion or aggregation of 50m sensing range from the vehicle front bumper
        auto sensorInfo = asn1cpp::makeSeq (SensorInformation);
        asn1cpp::setField (sensorInfo->sensorId, 2);
        asn1cpp::setField (sensorInfo->sensorType, SensorType_localAggregation);
        asn1cpp::setField (sensorInfo->shadowingApplies, true);
        auto detectionArea = asn1cpp::makeSeq (Shape);
        asn1cpp::setField (detectionArea->present, Shape_PR_circular);
        auto circularArea = asn1cpp::makeSeq (CircularShape);
        auto egoPos = m_vdp->getPositionXY ();
        auto refPos = asn1cpp::makeSeq (CartesianPosition3d);
        asn1cpp::setField (refPos->xCoordinate, egoPos.x);
        asn1cpp::setField (refPos->yCoordinate, egoPos.y);
        asn1cpp::setField (circularArea->shapeReferencePoint, refPos);
        asn1cpp::setField (circularArea->radius, 50);
        asn1cpp::setField (detectionArea->choice.circular, circularArea);
        asn1cpp::setField (sensorInfo->perceptionRegionShape, detectionArea);

        asn1cpp::sequenceof::pushList (*sensorInfoContainer, sensorInfo);

        asn1cpp::setField (CPMcontainer->containerData.present,
                           WrappedCpmContainer__containerData_PR_SensorInformationContainer);
        asn1cpp::setField (CPMcontainer->containerData.choice.SensorInformationContainer,
                           sensorInfoContainer);
        asn1cpp::sequenceof::pushList (cpm->payload.cpmContainers, CPMcontainer);
        m_T_LastSensorInfoContainer = now;
      }
    else
      {
        //If no sensorInformationContainer and no perceivedObjectsContainer
        if (numberOfPOs == 0)
          return; //No CPM is generated in the current cycle
      }

    if (numberOfPOs != 0)
      {
        auto CPMcontainer = asn1cpp::makeSeq (WrappedCpmContainer);
        asn1cpp::setField (CPMcontainer->containerId, 5);
        asn1cpp::setField (CPMcontainer->containerData.present,
                           WrappedCpmContainer__containerData_PR_PerceivedObjectContainer);
        asn1cpp::setField (CPMcontainer->containerData.choice.PerceivedObjectContainer,
                           POsContainer);
        asn1cpp::sequenceof::pushList(cpm->payload.cpmContainers,CPMcontainer);
      }

    // TODO: Support for Perception Region information from LDM (to be implemented in both SUMOensor and CARLAsensor)
    try {
        encode_result = asn1cpp::uper::encode(cpm);
    } catch (const std::exception& e) {
        // Handle any exceptions thrown during encoding
        std::cerr << "Encoding failed: " << e.what() << std::endl;
    }
    if(encode_result.size()<1)
    {
        NS_LOG_ERROR("Warning: unable to encode CPM.");
        return;
    }

    packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());
    //packet = Create<Packet> ((uint8_t*) bytes, length);

    dataRequest.BTPType = BTP_B; //!< BTP-B
    dataRequest.destPort = CP_PORT;
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

    m_cpm_sent++;

    // Estimation of the transmission time
    m_last_transmission = (double) Simulator::Now().GetMilliSeconds();
    uint32_t packetSize = packet->GetSize();
    m_Ton_pp = (double) (NanoSeconds((packetSize * 8) / 0.006) + MicroSeconds(68)).GetNanoSeconds();
    m_Ton_pp = m_Ton_pp / 1e6;

    //toffUpdateAfterTransmission();




      //"timestamp,numberOfObjects,numberOfVRU,averageDistanceBetweenVRU,CPMsize"
      uint64_t ID1 {};
      uint64_t ID2 {};
      uint64_t ID3 {};
      uint64_t ID4 {};
      uint64_t ID5 {};
      uint64_t ID6 {};
      uint64_t ID7 {};
      uint64_t ID8 {};
      uint64_t ID9 {};
      uint64_t ID10 {};
      uint64_t clusterID1 {};
      uint64_t clusterID2 {};
      uint64_t clusterID3 {};
      uint64_t sizeC1 {};
      uint64_t sizeC2 {};
      uint64_t sizeC3 {};
      uint64_t n_clusters {};
      try {
          ID1 = vruIDs.at(0);
      }catch (const std::out_of_range& e){}
      try {
          ID2 = vruIDs.at(1);
      }catch (const std::out_of_range& e){}
      try {
          ID3 = vruIDs.at(2);
      }catch (const std::out_of_range& e){}
      try {
          ID4 = vruIDs.at(3);
      }catch (const std::out_of_range& e){}
      try {
          ID5 = vruIDs.at(4);
      }catch (const std::out_of_range& e){}
      try {
          ID6 = vruIDs.at(5);
      }catch (const std::out_of_range& e){}
      try {
          ID7 = vruIDs.at(6);
      }catch (const std::out_of_range& e){}
      try {
          ID8 = vruIDs.at(7);
      }catch (const std::out_of_range& e){}
      try {
          ID9 = vruIDs.at(8);
      }catch (const std::out_of_range& e){}
      try {
          ID10 = vruIDs.at(9);
      }catch (const std::out_of_range& e){}
      try {
          if (clusterCPM_this_loop[1]==true) {
              clusterID1 = 1;
              sizeC1 = cluster_map[1].cardinality;
              n_clusters++;
          }
      }catch (const std::out_of_range& e){}
      try {
          if (clusterCPM_this_loop[2]==true) {
              clusterID2 = 2;
              sizeC2 = cluster_map[2].cardinality;
              n_clusters++;
          }
      }catch (const std::out_of_range& e){}
      try {
          if (clusterCPM_this_loop[3]==true) {
              clusterID3 = 3;
              sizeC3 = cluster_map[3].cardinality;
              n_clusters++;
          }
      }catch (const std::out_of_range& e){}

      m_csv_ofstream_cpm << now << "," << m_station_id << "," << numberOfPOs << "," << numberofVRUs <<","<< n_clusters <<","<<
      encode_result.size()<< "," << ID1 << "," << ID2 << "," << ID3 << "," << ID4 << "," << ID5 << "," << ID6 << "," <<
      ID7 << "," << ID8 << "," << ID9 << "," << ID10  << "," << clusterID1 << "," << sizeC1 << "," << clusterID2  <<
      "," << sizeC2 << "," << clusterID3  << "," << sizeC3 << std::endl;

    // Store the time in which the last CPM (i.e. this one) has been generated and successfully sent
    m_T_GenCpm_ms=now-lastCpmGen;
    lastCpmGen = now;
  }
  void
  CPBasicService::startCpmDissemination()
  {
    // Old desync code kept just for reference
    // It may lead to nodes not being desynchronized properly in specific situations in which
    // Simulator::Now().GetNanoSeconds () returns the same seed for multiple nodes
    // std::srand(Simulator::Now().GetNanoSeconds ());
    // double desync = ((double)std::rand()/RAND_MAX);
    m_csv_ofstream_cpm.open (m_csv_name+"-"+std::to_string(m_station_id)+"-CPM.csv",std::ofstream::trunc);
    m_csv_ofstream_cpm << "Timestamp,CAV,NumberOfObjects,NumberOfVRU,n_clusters,CPMsize,ID1,ID2,ID3,ID4,"
                          "ID5,ID6,ID7,ID8,ID9,ID10,clusterID1,SizeC1,clusterID2,SizeC2,clusterID3,SizeC3" << std::endl;

    Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
    desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
    desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
    double desync = desync_rvar->GetValue ();
    m_event_cpmDisseminationStart = Simulator::Schedule (Seconds(desync), &CPBasicService::initDissemination, this);
  }

  uint64_t
  CPBasicService::terminateDissemination()
  {
      if (!m_csv_name.empty ())
      {
          m_csv_ofstream_cpm.close ();
      }
    Simulator::Remove(m_event_cpmDisseminationStart);
    Simulator::Remove(m_event_cpmSend);
    return m_cpm_sent;
  }

  void
  CPBasicService::receiveCpm (BTPDataIndication_t dataIndication, Address from)
  {
    Ptr<Packet> packet;
    asn1cpp::Seq<CollectivePerceptionMessage> decoded_cpm,cpm_test;

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


    /** Decoding **/
    decoded_cpm = asn1cpp::uper::decodeASN(packetContent, CollectivePerceptionMessage);
    free(buffer);

    if(bool(decoded_cpm)==false) {
        NS_LOG_ERROR("Warning: unable to decode a received CPM.");
        return;
      }

    if(m_CPReceiveCallback!=nullptr) {
        m_CPReceiveCallback(decoded_cpm,from);
      } else if(m_CPReceiveCallbackExtended!=nullptr) {
        m_CPReceiveCallbackExtended(decoded_cpm,from,m_station_id,m_stationtype,GetSignalInfo());
      }
  }
  int64_t
  CPBasicService::computeTimestampUInt64()
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
  CPBasicService::toffUpdateAfterDeltaUpdate(double delta)
  {
//    if (m_last_transmission == 0)
//      return;
//    double waiting = Simulator::Now().GetMilliSeconds() - m_last_transmission;
//    double aux = m_Ton_pp / delta * (m_N_GenCpm - waiting) / m_N_GenCpm + waiting;
//    aux = std::max (aux, 25.0);
//    double new_gen_time = std::min (aux, 1000.0);
//    setCheckCpmGenMs ((long) new_gen_time);
//    m_last_delta = delta;
  }

  void
  CPBasicService::toffUpdateAfterTransmission()
  {
    if (m_last_delta == 0)
      return;
    double aux = m_Ton_pp / m_last_delta;
    double new_gen_time = std::max(aux, 25.0);
    new_gen_time = std::min(new_gen_time, 1000.0);
    setCheckCpmGenMs ((long) new_gen_time);
  }
}
//struct GeoPoint {
//    double latitude;  // in degrees
//    double longitude; // in degrees
//};

// Function to calculate heading and velocity
std::pair <double, double> ns3::CPBasicService::calculateHeadingAndVelocity(double lat1, double lon1, double timestamp1,
                                 double lat2, double lon2, double timestamp2){
    constexpr double EARTH_RADIUS = 6371000.0;
    // Calculate the time difference in seconds (from milliseconds)
    double timeDifference = (timestamp2 - timestamp1) / 1000.0; // Convert to seconds

    // Convert latitude and longitude from degrees to radians
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Calculate the differences
    double dLon = lon2 - lon1;
    double dLat = lat2 - lat1;

    // Calculate the heading
    double heading = std::atan2(dLon, dLat) * 180.0 / M_PI; // Convert to degrees
    if (heading < 0) {
        heading += 360; // Normalize heading to be between 0 and 360
    }

    // Calculate the distance using the Haversine formula
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = EARTH_RADIUS * c; // Distance in meters

    // Calculate velocity in m/s
    double velocity = (timeDifference > 0) ? distance / timeDifference : 0; // Velocity in m/s
    // No movement if timestamps are the same

    return {heading, velocity};

}