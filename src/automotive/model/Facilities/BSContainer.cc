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
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "ns3/BSContainer.h"

#include "../utilities/sumo-sensor.h"
#include "ns3/vdp.h"
#include "ns3/VRUdp.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("BSContainer");

  BSContainer::~BSContainer() {
    NS_LOG_INFO("BSContainer object destroyed.");
  }

  BSContainer::BSContainer()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket = nullptr;
    m_btp = nullptr;
    m_gn = nullptr;
    m_real_time = false;
    m_mobility_client = nullptr;
    m_LDM = nullptr;

    m_metric_sup_ptr = nullptr;
    m_prrsup_beacons = true;

    m_sumo_vehid_prefix = "veh";
    m_sumo_pedid_prefix = "ped";

    m_csv_file_name = "";
    m_VAM_metrics = false;

    m_is_configured = false;
    m_DENMs_enabled = false;
  }

  BSContainer::BSContainer(unsigned long fixed_stationid, long fixed_stationtype, Ptr<TraciClient> mobility_client, bool real_time, Ptr<Socket> socket_rxtx)
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket = nullptr;
    m_btp = nullptr;
    m_gn = nullptr;
    m_real_time = false;
    m_mobility_client = nullptr;
    m_LDM = nullptr;

    m_metric_sup_ptr = nullptr;
    m_prrsup_beacons = true;

    m_sumo_vehid_prefix = "veh";
    m_sumo_pedid_prefix = "ped";

    m_csv_file_name = "";
    m_VAM_metrics = false;

    m_is_configured = false;
    m_DENMs_enabled = false;

    m_station_id = (StationId_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;

    m_mobility_client=mobility_client;
    m_real_time=real_time;

    m_socket=socket_rxtx;
  }

  void
  BSContainer::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
  }

  void
  BSContainer::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
  }

  void
  BSContainer::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
  }

  void
  BSContainer::setSocket(Ptr<Socket> socket_rxtx)
  {
    m_socket=socket_rxtx;
  }

  void BSContainer::setVAMmetricsfile(std::string file_name, bool collect_metrics){
    m_csv_file_name = file_name;
    m_VAM_metrics = collect_metrics;
  }

  void
  BSContainer::setupContainer(bool CABasicService_enabled,bool DENBasicService_enabled,bool VRUBasicService_enabled,bool CPService_enabled,bool MCMBasicService_enabled,bool security_enabled) {
    if(CABasicService_enabled==false && DENBasicService_enabled==false && VRUBasicService_enabled==false && CPService_enabled==false && MCMBasicService_enabled) {
      NS_FATAL_ERROR("Error. Called setupContainer() asking for enabling zero Basic Services. Aborting simulation.");
    }

    if(m_socket==nullptr || m_mobility_client==nullptr || m_station_id==ULONG_MAX || m_stationtype==LONG_MAX) {
      NS_FATAL_ERROR("Error. Attempted to call setupContainer() on an uninitialized BSContainer. Aborting simulation.");
    }

    // If dealing with a pedestrian create the local dynamic map
    m_LDM = CreateObject<LDM> ();
    m_LDM->setTraCIclient (m_mobility_client);
    m_LDM->setStationType (m_stationtype);

    if(m_stationtype == StationType_pedestrian){
        m_LDM->setStationID (m_sumo_pedid_prefix + std::to_string (m_station_id));
    }
    else{
        m_LDM->setStationID (m_sumo_vehid_prefix + std::to_string (m_station_id));
    }

    // Setup the required services
    // ETSI Transport and Networking layers
    m_btp = CreateObject <btp>();
    m_gn = CreateObject <GeoNet>();

    if(m_metric_sup_ptr!=nullptr) {
      m_gn->setMetricSupervisor (m_metric_sup_ptr);
    }

    m_gn->setSecurity(security_enabled);

    if(m_prrsup_beacons==false) {
      m_gn->disablePRRsupervisorForBeacons();
    }

    m_btp->setGeoNet (m_gn);

    if(CABasicService_enabled==true) {
      m_cabs.setBTP (m_btp);
      m_cabs.setSocketTx (m_socket);
      m_cabs.setSocketRx (m_socket);
      m_cabs.setLDM (m_LDM);

      // Remember that setStationProperties() must always be called *after* setBTP()
      m_cabs.setStationProperties (m_station_id, m_stationtype);

      if(m_CAReceiveCallbackExtended!=nullptr) {
        m_cabs.addCARxCallbackExtended (m_CAReceiveCallbackExtended);
      }

      m_CAMs_enabled = true;
    }

    if(m_stationtype != StationType_pedestrian){
        m_vdp_ptr = new VDPTraCI(m_mobility_client,m_sumo_vehid_prefix + std::to_string(m_station_id));
        m_btp->setVDP(m_vdp_ptr);

        if(CABasicService_enabled==true) {
          m_cabs.setVDP(m_vdp_ptr);
        }
        if (CPService_enabled==true)
          {
            m_cpbs.setVDP (m_vdp_ptr);
          }
        if (MCMBasicService_enabled==true)
          {
            m_mcbs.setVDP (m_vdp_ptr);
          }
      }

    if(DENBasicService_enabled==true) {
      m_denbs.setBTP (m_btp);
      m_denbs.setSocketTx (m_socket);
      m_denbs.setSocketRx (m_socket);
      m_denbs.setStationProperties (m_station_id, (long)m_stationtype);

      if(m_DENReceiveCallbackExtended!=nullptr) {
        m_denbs.addDENRxCallbackExtended (m_DENReceiveCallbackExtended);
      }

      m_DENMs_enabled = true;
    }

    if(VRUBasicService_enabled==true) {
      m_vrubs.setBTP (m_btp);
      m_vrubs.setSocketTx (m_socket);
      m_vrubs.setSocketRx (m_socket);
      //m_vrubs.setVAMmetricsfile (m_csv_file_name, m_VAM_metrics);
      m_vrubs.setLDM (m_LDM);

      // Remember that setStationProperties() must always be called *after* setBTP()
      m_vrubs.setStationProperties (m_station_id, m_stationtype);

      if(m_VAMReceiveCallbackExtended!=nullptr) {
        m_vrubs.addVAMRxCallbackExtended (m_VAMReceiveCallbackExtended);
      }

      m_VAMs_enabled = true;
    }

    if(m_stationtype == StationType_pedestrian){
        m_vrudp_ptr = new VRUdp(m_mobility_client,m_sumo_pedid_prefix + std::to_string(m_station_id));
        m_btp->setVRUdp(m_vrudp_ptr);

        if(VRUBasicService_enabled==true) {
            m_vrubs.setVRUdp (m_vrudp_ptr);
          }
      }

    if(CPService_enabled==true)
      {
        m_cpbs.setBTP (m_btp);
        m_cpbs.setSocketTx (m_socket);
        m_cpbs.setSocketRx (m_socket);

        m_LDM->setVDP (m_vdp_ptr);
        // m_LDM->setWriteContents (true);
        m_cpbs.setLDM (m_LDM);

        m_cpbs.addCPRxCallback (std::bind(&BSContainer::receiveCPM,this,std::placeholders::_1,std::placeholders::_2));
        m_cpbs.setRealTime (m_real_time);
        m_cpbs.setRedundancyMitigation (false);

        m_sumo_sensor = CreateObject<SUMOSensor>();
        m_sumo_sensor->setStationID(m_sumo_vehid_prefix + std::to_string(m_station_id));
        m_sumo_sensor->setTraCIclient(m_mobility_client);
        m_sumo_sensor->setVDP(m_vdp_ptr);
        m_sumo_sensor->setLDM (m_LDM);

        // Remember that setStationProperties() must always be called *after* setBTP()
        m_cpbs.setStationProperties (m_station_id, m_stationtype);

        if(m_CPMReceiveCallbackExtended!=nullptr) {
            m_cpbs.addCPRxCallbackExtended (m_CPMReceiveCallbackExtended);
          }

        m_CPMs_enabled = true;
      }

    if(MCMBasicService_enabled==true) {
        m_mcbs.setBTP (m_btp);
        m_mcbs.setSocketTx (m_socket);
        m_mcbs.setSocketRx (m_socket);
        m_mcbs.setLDM (m_LDM);

        // Remember that setStationProperties() must always be called *after* setBTP()
        m_mcbs.setStationProperties (m_station_id, m_stationtype);

        if(m_MCMReceiveCallbackExtended!=nullptr) {
            m_mcbs.addMCRxCallbackExtended (m_MCMReceiveCallbackExtended);
          }

        m_MCMs_enabled = true;
      }

    m_is_configured = true;
  }

  void
  BSContainer::setDENMCircularGeoArea(double lat, double lon, int radius_meters)
  {
    if(m_is_configured==false) {
      NS_FATAL_ERROR("Error. setDENMCircularGeoArea() should not be called before setupContainer(). Please adjust your code.");
    }

    if(m_DENMs_enabled==true) {
      /* Compute GeoArea for DENMs */
      GeoArea_t geoArea;
      // Longitude and Latitude in [0.1 microdegree]
      geoArea.posLong = lon*DOT_ONE_MICRO;
      geoArea.posLat = lat*DOT_ONE_MICRO;
      // Radius [m] of the circle that covers the whole square area of the map in (x,y)
      geoArea.distA = radius_meters;
      // DistB [m] and angle [deg] equal to zero because we are defining a circular area as specified in ETSI EN 302 636-4-1 [9.8.5.2]
      geoArea.distB = 0;
      geoArea.angle = 0;
      geoArea.shape = CIRCULAR;

      m_denbs.setGeoArea (geoArea);
    }
  }

  void
  BSContainer::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
  {
   /* Implement CPM strategy here */
   (void) from;
   int fromID = asn1cpp::getField(cpm->header.stationId,long);
   if (m_recvCPMmap.find(fromID) == m_recvCPMmap.end())
     {
       m_recvCPMmap[fromID] = std::map<int,int>(); // First CPM from this vehicle
     }

   //For every PO inside the CPM, if any
   bool POs_ok;
   //auto wrappedContainer = asn1cpp::makeSeq(WrappedCpmContainer);
   int wrappedContainer_size = asn1cpp::sequenceof::getSize(cpm->payload.cpmContainers);
   for (int i=0; i<wrappedContainer_size; i++)
     {
       auto wrappedContainer = asn1cpp::sequenceof::getSeq(cpm->payload.cpmContainers,WrappedCpmContainer,i);
       WrappedCpmContainer__containerData_PR present = asn1cpp::getField(wrappedContainer->containerData.present,WrappedCpmContainer__containerData_PR);
       if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
        {
          auto POcontainer = asn1cpp::getSeq(wrappedContainer->containerData.choice.PerceivedObjectContainer,PerceivedObjectContainer);
          int PObjects_size = asn1cpp::sequenceof::getSize(POcontainer->perceivedObjects);
          // std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_station_id <<" received a new CPMv2 from " << asn1cpp::getField(cpm->header.stationId,long) << " with " << PObjects_size << " perceived objects." << std::endl;
          for(int j=0; j<PObjects_size;j++)
            {
              LDM::returnedVehicleData_t PO_data;
              auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
              PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);
              //If PO is already in local copy of vLDM
              if(m_recvCPMmap[fromID].find(asn1cpp::getField(PO_seq->objectId,long)) == m_recvCPMmap[fromID].end())
                {
                  // First time we have received this object from this vehicle
                  // If PO id is already in local copy of LDM
                  if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectId,long),PO_data) == LDM::LDM_OK)
                    {
                      // We need a new ID for object
                      std::set<int> IDs;
                      m_LDM->getAllIDs (IDs);
                      int newID = 1;
                      for (int num : IDs) {
                          if (num == newID) {
                              ++newID;
                            } else if (num > newID) {
                              break;
                            }
                        }
                      //Translate CPM data to LDM format
                      m_LDM->insert(BSContainer::translateCPMdata(cpm,PO_seq,i,newID));
                      //Update recvCPMmap
                      m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)] = newID;
                    }
                  else
                    {
                      //Translate CPM data to LDM format
                      m_LDM->insert(BSContainer::translateCPMdata(cpm,PO_seq,i,-1));
                      //Update recvCPMmap
                      m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)] = asn1cpp::getField(PO_seq->objectId,long);
                    }
                }
              else
                {
                  // We have already receive this object from this vehicle
                  m_LDM->insert(BSContainer::translateCPMdata(cpm,PO_seq,i,m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)]));
                }
            }
        }
     }
  }

  vehicleData_t
  BSContainer::translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                           asn1cpp::Seq<PerceivedObject> object, int objectIndex, int newID)
  {
   vehicleData_t retval;
   retval.detected = true;
   if(newID == -1)
     retval.stationID = asn1cpp::getField(object->objectId,long);
   else
     retval.stationID = newID;
   retval.ID = std::to_string(retval.stationID);
   retval.vehicleLength = asn1cpp::getField(object->objectDimensionX->value,long);
   retval.vehicleWidth = asn1cpp::getField(object->objectDimensionY->value,long);
   retval.heading = asn1cpp::getField(object->angles->zAngle.value,double) / DECI;
   retval.xSpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.xVelocity.value,long));
   retval.ySpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.yVelocity.value,long));
   retval.speed_ms = (sqrt (pow(retval.xSpeedAbs.getData(),2) +
                            pow(retval.ySpeedAbs.getData(),2)))/CENTI;

   VDP::VDP_position_cartesian_t objectPosition;
   double fromLon = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.longitude,double)/DOT_ONE_MICRO;
   double fromLat = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.latitude,double)/DOT_ONE_MICRO;
   if(m_mobility_client != nullptr)
     {
       libsumo::TraCIPosition traciPosition = m_mobility_client->TraCIAPI::simulation.convertLonLattoXY (fromLon,fromLat);
       objectPosition.x = traciPosition.x;
       objectPosition.y = traciPosition.y;
     }
   VDP::VDP_position_latlon_t objectPositionGeo;
   objectPosition.x += asn1cpp::getField(object->position.xCoordinate.value,long)/CENTI;
   objectPosition.y += asn1cpp::getField(object->position.yCoordinate.value,long)/CENTI;
   retval.x = objectPosition.x;
   retval.y = objectPosition.y;
   if(m_mobility_client != nullptr)
     {
       libsumo::TraCIPosition traciPosition = m_mobility_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);
       objectPositionGeo.lon = traciPosition.x;
       objectPositionGeo.lat = traciPosition.y;
     }

   retval.lon = objectPositionGeo.lon;
   retval.lat = objectPositionGeo.lat;

   retval.camTimestamp = asn1cpp::getField(cpm->payload.managementContainer.referenceTime,long);
   retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(object->measurementDeltaTime,long)*1000);
   retval.stationType = StationType_passengerCar;
   retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));

   return retval;
  }

  void
  BSContainer::cleanup()
  {
    if(m_CAMs_enabled==true) {
      m_cabs.terminateDissemination();
    }

    if(m_DENMs_enabled==true) {
      m_denbs.cleanup ();
    }

    if(m_VAMs_enabled==true) {
      m_vrubs.terminateDissemination();
    }

    if (m_CPMs_enabled == true) {
      m_cpbs.terminateDissemination();
      if (m_sumo_sensor != nullptr)
        {
          m_sumo_sensor->cleanup();
        }
    }

    if(m_gn!=nullptr) {
      m_gn->cleanup();
    }
    if(m_LDM!=nullptr) {
      m_LDM->cleanup();
    }
  }
}
