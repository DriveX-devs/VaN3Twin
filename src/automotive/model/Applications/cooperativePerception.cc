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

#include "cooperativePerception.h"

#include "ns3/CAM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"
#include "ns3/vrudpOpenCDA.h"
#include <memory>

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("cooperativePerception");

  NS_OBJECT_ENSURE_REGISTERED(cooperativePerception);


  TypeId
  cooperativePerception::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::cooperativePerception")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<cooperativePerception> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&cooperativePerception::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&cooperativePerception::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&cooperativePerception::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&cooperativePerception::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue (""),
            MakeStringAccessor (&cooperativePerception::m_model),
            MakeStringChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&cooperativePerception::m_traci_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("OpenCDAClient",
            "OpenCDA client",
            PointerValue (0),
            MakePointerAccessor (&cooperativePerception::m_opencda_client),
            MakePointerChecker<OpenCDAClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&cooperativePerception::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&cooperativePerception::m_send_cam),
            MakeBooleanChecker ())
        .AddAttribute ("VisualizeSensor",
           "To enable/disable the visualization of the sensor",
           BooleanValue(false),
           MakeBooleanAccessor (&cooperativePerception::m_vis_sensor),
           MakeBooleanChecker ())
        .AddAttribute ("itsType",
                     "To identify the type of actor",
                       StringValue(""),
                     MakeStringAccessor (&cooperativePerception::m_type),
                       MakeStringChecker ())
        .AddAttribute ("SendVAM",
                       "To enable/disable the transmission of VAM messages",
                       BooleanValue(true),
                       MakeBooleanAccessor (&cooperativePerception::m_send_vam),
                       MakeBooleanChecker ());
        return tid;
  }

  cooperativePerception::cooperativePerception ()
  {
    NS_LOG_FUNCTION(this);
    m_traci_client = nullptr;
    m_opencda_client = nullptr;
    m_print_summary = true;
    m_already_print = false;
    m_send_cam = true;
    m_vis_sensor = false;
    m_metric_supervisor = nullptr;

    m_cam_received = 0;
    m_cpm_received = 0;
    m_vam_received = 0;

    m_distance_threshold = 75; // Distance used in GeoNet to determine the radius of the circumference arounf the emergency vehicle where the DENMs are valid
    m_heading_threshold = 45; // Max heading angle difference between the normal vehicles and the emergenecy vehicle, that triggers a reaction in the normal vehicles
  }

  cooperativePerception::~cooperativePerception ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  cooperativePerception::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  cooperativePerception::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    /* Save the vehicles informations */

    if(m_traci_client != nullptr)
      {
        // SUMO mobility
        m_id = m_traci_client->GetVehicleId (this->GetNode ());
        m_type = m_traci_client->TraCIAPI::vehicle.getVehicleClass (m_id);

        m_vdp = std::make_unique<VDPTraCI>(m_traci_client,m_id);
        //vrudp = new VRUdp(m_traci_client,m_id);

        //Create LDM and sensor object
        m_LDM = CreateObject<LDM>();
        m_LDM->setStationID(m_id);
        m_LDM->setTraCIclient(m_traci_client);
        m_LDM->setVDP(m_vdp.get());

        m_sumo_sensor = CreateObject<SUMOSensor>();
        m_sumo_sensor->setStationID(m_id);
        m_sumo_sensor->setTraCIclient(m_traci_client);
        m_sumo_sensor->setVDP(m_vdp.get());
        m_sumo_sensor->setLDM (m_LDM);

      }
    else if(m_opencda_client != nullptr)
      {
        int stationID = m_opencda_client->getVehicleID (this->GetNode ());
        m_id = std::to_string (stationID);
        m_type = m_type;//StationType_passengerCar; //TODO: add support for multiple vehicle types

        m_vdp = std::make_unique<VDPOpenCDA>(m_opencda_client, m_id);
        m_vrudp = std::make_unique<VRUdpOpenCDA>(m_opencda_client, m_id);

        m_LDM = CreateObject<LDM>();
        m_LDM->setStationID(stationID);
        m_LDM->setVDP(m_vdp.get());
        //m_LDM->enableOutputFile (m_id);


        m_opencda_sensor = CreateObject<OpenCDASensor>();
        m_opencda_sensor->setStationID(stationID);
        m_opencda_sensor->setOpenCDAClient (m_opencda_client);
        m_opencda_sensor->setVDP(m_vdp.get());
        m_opencda_sensor->setLDM (m_LDM);
        m_opencda_sensor->enableGUI(m_vis_sensor);
        m_opencda_sensor->setClustering(false);
      }
    else
      {
        NS_FATAL_ERROR ("No mobility set - check simulation script - valid mobilities: 'SUMO' or 'CARLA'");
      }

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
        m_geoNet->setMetricSupervisor(m_metric_supervisor);
//      m_metric_supervisor->startCheckCBR();
    }

    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);
    m_cpService.setBTP(m_btp);
    m_caService.setLDM(m_LDM);
    m_cpService.setLDM(m_LDM);
    m_vruService.setBTP(m_btp);
    m_vruService.setLDM(m_LDM);

    /* Create the Sockets for TX and RX */
    TypeId tid;
    if(m_model=="80211p")
      tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    else if(m_model=="cv2x" || m_model=="nrv2x")
      tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script - valid models: '80211p' or 'lte'");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    if(m_model=="80211p")
    {
        /* Bind the socket to local address */
        PacketSocketAddress local = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress () );
        if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
        // Set the socketAddress for broadcast
        PacketSocketAddress remote = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetBroadcast () );
        m_socket->Connect (remote);
    }
    else // m_model=="cv2x"
    {
        /* The C-V2X model requires the socket to be bind to "any" IPv4 address, and to be connected to the
         * IP address of the transmitting node. Then, the model will take care of broadcasting the packets.
        */
        if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
        }
        m_socket->Connect (InetSocketAddress(m_ipAddress,19));
    }

    /* Set Station Type in DENBasicService */
    StationType_t stationtype;
    if (m_type=="StationType_passengerCar") //"StationType_passengerCar" or StationType_pedestrian
      stationtype = StationType_passengerCar;
    else if (m_type=="StationType_pedestrian"){
      stationtype = StationType_pedestrian;
      //m_LDM->enablePolygons (); // Uncomment to enable detected object polygon visualization for this specific vehicle
      }
    else
      stationtype = StationType_unknown;


    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx(m_socket);
    m_caService.setSocketRx(m_socket);
    m_caService.setStationProperties(std::stol(m_id), (long) stationtype);
    m_caService.addCARxCallback(
            std::bind(&cooperativePerception::receiveCAM, this, std::placeholders::_1, std::placeholders::_2));
    m_caService.setRealTime(m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CPBasicService */
    m_cpService.setSocketTx (m_socket);
    m_cpService.setSocketRx (m_socket);
    m_cpService.setStationProperties (std::stol(m_id), (long)stationtype);
    m_cpService.addCPRxCallback (std::bind(&cooperativePerception::receiveCPM,this,std::placeholders::_1,std::placeholders::_2));
    m_cpService.setRealTime (m_real_time);
    m_cpService.setRedundancyMitigation (true);
    m_cpService.setClustering(true);

    /* Set sockets, callback, station properties and TraCI VDP in VRUBasicService */
    m_vruService.setSocketTx(m_socket);
    m_vruService.setSocketRx(m_socket);
    m_vruService.setStationProperties(std::stol(m_id), (long) stationtype);
    m_vruService.addVAMRxCallback(
            std::bind(&cooperativePerception::receiveVAM, this, std::placeholders::_1, std::placeholders::_2));


    m_vruService.setRoleAndClustState();
    m_vruService.setVAMmetricsfile(std::string("Ped"), (bool) true);

    /* Schedule CPM dissemination */
    if( stationtype == StationType_passengerCar){
        m_caService.setVDP(m_vdp.get());
        m_denService.setVDP(m_vdp.get());
        m_cpService.setVDP(m_vdp.get());

        m_cpService.startCpmDissemination ();
    }
    else if(stationtype == StationType_pedestrian){
        /* Set VRUdp for GeoNet object */
        m_vruService.setVRUdp(m_vrudp.get());
    }
    /* Schedule CAM dissemination */
    if(m_send_cam == true && stationtype == StationType_passengerCar)
    {
        Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
        desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
        desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
        double desync = desync_rvar->GetValue ();
        m_caService.startCamDissemination(desync);
    }
    /* Schedule VAM dissemination */
    if(m_send_vam == true && stationtype == StationType_pedestrian)
    {
        Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
        desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
        desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
        double desync = desync_rvar->GetValue ();
        m_vruService.startVamDissemination(desync);
    }

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name+"-"+m_id+"-CAM.csv",std::ofstream::trunc);
      m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }
  }

  void
  cooperativePerception::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_send_cam_ev);

    uint64_t cam_sent, cpm_sent, vam_sent;

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.close ();
    }

    cam_sent = m_caService.terminateDissemination ();
    cpm_sent = m_cpService.terminateDissemination ();
    vam_sent = m_vruService.terminateDissemination ();
    m_denService.cleanup();

    if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id
                << ", CAM-SENT: " << cam_sent
                << ", CAM-RECEIVED: " << m_cam_received
                << ", CPM-SENT: " << cpm_sent
                << ", CPM-RECEIVED: " << m_cpm_received
                << ", VAM-SENT: " << vam_sent
                << ", VAM-RECEIVED: " << m_vam_received
                << std::endl;
      m_already_print=true;
    }
    if(true){
        m_opencda_sensor->logPerceivedPedestrians();
    }

  }

  void
  cooperativePerception::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  cooperativePerception::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* Implement CAM strategy here */
   m_cam_received++;
   LDM::returnedVehicleData_t retveh;
   double fromLon = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO;
   double fromLat = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO;
   double fromID = asn1cpp::getField(cam->header.stationId,long);
   if(m_opencda_client)
     { if(m_type == "StationType_pedestrian"){
         return;
     }
       carla::Vector carlaPosition = m_opencda_client->getCartesian (fromLon,fromLat);
//       std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CAM from vehicle "
//                 << fromID << " --> GeoPosition: [" << fromLon << ", " << fromLat << "]"
//                 << " CartesianPosition: [" << carlaPosition.x () << ", " << carlaPosition.y () << "]" <<std::endl;

         LDM::LDM_error_t retval = m_LDM->lookup(fromID,retveh);
         VDP::VDP_position_cartesian_t objectPosition;
         if (m_traci_client != nullptr) {
             libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(
                     fromLon, fromLat);
             objectPosition.x = traciPosition.x;
             objectPosition.y = traciPosition.y;
         } else {
             carla::Vector carlaPosition = m_opencda_client->getCartesian(fromLon, fromLat);
             objectPosition.x = carlaPosition.x();
             objectPosition.y = carlaPosition.y();

             carla::Actor CV = m_opencda_client->GetActorById(std::stoi(m_id));
             carla::Vector CVpos = m_opencda_client->getCartesian(CV.longitude(),CV.latitude());
             retveh.vehData.xDistAbs = long(objectPosition.x - CVpos.x())*CENTI;
             retveh.vehData.yDistAbs = long(objectPosition.y - CVpos.y())*CENTI;
         }
         retveh.vehData.x = objectPosition.x;
         retveh.vehData.y = objectPosition.y;

         if (retval == LDM::LDM_OK){
             retval = m_LDM->insert(retveh.vehData);
         }
     }

   if (!m_csv_name.empty ())
     {
       // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
       m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
       m_csv_ofstream_cam << cam->cam.generationDeltaTime << "," << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO << ",";
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO << "," ;
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,double)/DECI << "," << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,double)/CENTI << ",";
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value,double)/DECI << std::endl;
     }

  }

  void
  cooperativePerception::receiveCPMV1 (asn1cpp::Seq<CPMV1> cpm, Address from)
  {
   /* Implement CPM strategy here */
   m_cpm_received++;
   (void) from;
   std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv1 from vehicle " << asn1cpp::getField(cpm->header.stationId,long) <<" with "<< asn1cpp::getField(cpm->cpm.cpmParameters.numberOfPerceivedObjects,long)<< " perceived objects." <<std::endl;
   int fromID = asn1cpp::getField(cpm->header.stationId,long);
   if (m_recvCPMmap.find(fromID) == m_recvCPMmap.end())
     m_recvCPMmap[fromID] = std::map<int,int>(); // First CPM from this vehicle
   //For every PO inside the CPM, if any
   bool POs_ok;
   auto PObjects = asn1cpp::getSeqOpt(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObjectContainer,&POs_ok);
   if (POs_ok)
     {
       int PObjects_size = asn1cpp::sequenceof::getSize(cpm->cpm.cpmParameters.perceivedObjectContainer);
       for(int i=0; i<PObjects_size;i++)
        {
          LDM::returnedVehicleData_t PO_data;
          auto PO_seq = asn1cpp::makeSeq(PerceivedObjectV1);
          PO_seq = asn1cpp::sequenceof::getSeq(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObjectV1,i);
          if(m_recvCPMmap[fromID].find(asn1cpp::getField(PO_seq->objectID,long)) == m_recvCPMmap[fromID].end())
            {
              // First time we have received this object from this vehicle
              //If PO id is already in local copy of LDM
              if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectID,long),PO_data) == LDM::LDM_OK)
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
                  m_LDM->insert(translateCPMV1data(cpm,i,newID));
                  //Update recvCPMmap
                  m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectID,long)] = newID;
                }
              else
                {
                  //Translate CPM data to LDM format
                  m_LDM->insert(translateCPMV1data(cpm,i,-1));
                  //Update recvCPMmap
                  m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectID,long)] = asn1cpp::getField(PO_seq->objectID,long);
                }
            }
          else
            {
              // We have already receive this object from this vehicle
              m_LDM->insert(translateCPMV1data(cpm,i,m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectID,long)]));
            }
        }
     }
  }

  vehicleData_t
  cooperativePerception::translateCPMV1data (asn1cpp::Seq<CPMV1> cpm, int objectIndex, int newID)
  {
   vehicleData_t retval;
   auto PO_seq = asn1cpp::makeSeq(PerceivedObjectV1);
   using namespace boost::geometry::strategy::transform;
   PO_seq = asn1cpp::sequenceof::getSeq(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObject,objectIndex);
   retval.detected = true;
   if(newID == -1)
     retval.stationID = asn1cpp::getField(PO_seq->objectID,long);
   else
     retval.stationID = newID;
   retval.ID = std::to_string(retval.stationID);
   retval.vehicleLength = asn1cpp::getField(PO_seq->planarObjectDimension1->value,long);
   retval.vehicleWidth = asn1cpp::getField(PO_seq->planarObjectDimension2->value,long);
   retval.heading = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue,double)/10 +
                    asn1cpp::getField(PO_seq->yawAngle->value,double)/10;
   if (retval.heading > 360.0)
     retval.heading -= 360.0;

   retval.speed_ms = (double) (asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue,long) +
                               asn1cpp::getField(PO_seq->xSpeed.value,long))/CENTI;

   double fromLon = asn1cpp::getField(cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude,double)/DOT_ONE_MICRO;
   double fromLat = asn1cpp::getField(cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude,double)/DOT_ONE_MICRO;

   VDP::VDP_position_cartesian_t objectPosition;
   if(m_traci_client != nullptr)
     {
       libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertLonLattoXY (fromLon,fromLat);
       objectPosition.x = traciPosition.x;
       objectPosition.y = traciPosition.y;
     }
   else
     {
       carla::Vector carlaPosition = m_opencda_client->getCartesian (fromLon,fromLat);
       objectPosition.x = carlaPosition.x ();
       objectPosition.y = carlaPosition.y ();
     }

   point_type objPoint(asn1cpp::getField(PO_seq->xDistance.value,double)/CENTI,asn1cpp::getField(PO_seq->yDistance.value,double)/CENTI);
   double fromAngle = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue,double)/10;
   rotate_transformer<boost::geometry::degree, double, 2, 2> rotate(fromAngle-90);
   boost::geometry::transform(objPoint, objPoint, rotate);// Transform points to the reference (x,y) axises
   VDP::VDP_position_latlon_t objectPositionGeo;
   if(m_traci_client != nullptr)
     {
       objectPosition.x += boost::geometry::get<0>(objPoint);
       objectPosition.y += boost::geometry::get<1>(objPoint);
       libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);
       objectPositionGeo.lon = traciPosition.x;
       objectPositionGeo.lat = traciPosition.y;
     }
   else
     {
       objectPosition.x += asn1cpp::getField(PO_seq->xDistance.value,double)/CENTI;
       objectPosition.y += asn1cpp::getField(PO_seq->yDistance.value,double)/CENTI;
       carla::Vector carlaPosition = m_opencda_client->getGeo (objectPosition.x,objectPosition.y);
       objectPositionGeo.lat = carlaPosition.x ();
       objectPositionGeo.lon = carlaPosition.y ();
     }

   retval.lon = objectPositionGeo.lon;
   retval.lat = objectPositionGeo.lat;

   point_type speedPoint(asn1cpp::getField(PO_seq->xSpeed.value,double)/CENTI,asn1cpp::getField(PO_seq->ySpeed.value,double)/CENTI);
   boost::geometry::transform(speedPoint, speedPoint, rotate);// Transform points to the reference (x,y) axises
   retval.speed_ms = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue,double)/CENTI + boost::geometry::get<0>(speedPoint);

   retval.camTimestamp = asn1cpp::getField(cpm->cpm.generationDeltaTime,long);
   retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(PO_seq->timeOfMeasurement,long)*1000);
   retval.stationType = StationType_passengerCar;
   retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));
   retval.confidence = asn1cpp::getField(PO_seq->objectConfidence,long);

   // FOR DEBUGGING
   if(m_traci_client == nullptr)
     {
       carla::Vector pos = m_opencda_client->getCartesian (retval.lon, retval.lat);
       std::cout << "[" << retval.stationID << "] --> Position: [" << pos.x ()<<", "<< pos.y () << "]" << std::endl;
     }

   return retval;

  }

  void
  cooperativePerception::receiveVAM (asn1cpp::Seq<VAM> vam, Address from)
  {
      //! /* Implement VAM strategy here */
      m_vam_received++;
      Ptr<Packet> packet;
      asn1cpp::Seq<VAM> decoded_vam;
      uint8_t *buffer;
      auto stationtype = asn1cpp::getField(vam->vam.vamParameters.basicContainer.stationType, StationType_t);
      auto stationID = asn1cpp::getField(vam->header.stationId,StationID_t);


      if (m_LDM != NULL) {
          if (m_type == "StationType_passengerCar") {
//              std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new VAM from " << stationID << std::endl;
              //if a car is receiving a VAM, update its LDM with "ground truth" information
              vehicleData_t vehdata;
              LDM::LDM_error_t db_retval;
              LDM::returnedVehicleData_t retVehicleData;
              bool lowFreq_ok;


              // detected = true, so we can send CPM about this pedestrian
              vehdata.detected = false;
              vehdata.VAM = true;
              vehdata.stationType = asn1cpp::getField(vam->vam.vamParameters.basicContainer.stationType, long);
              vehdata.stationID = asn1cpp::getField(vam->header.stationId, StationID_t);
              vehdata.lat =
                      asn1cpp::getField(vam->vam.vamParameters.basicContainer.referencePosition.latitude, double) /
                      (double) DOT_ONE_MICRO;
              vehdata.lon =
                      asn1cpp::getField(vam->vam.vamParameters.basicContainer.referencePosition.longitude, double) /
                      (double) DOT_ONE_MICRO;
              vehdata.elevation =
                      asn1cpp::getField(vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue,
                                        double) / (double) CENTI;
              vehdata.heading =
                      asn1cpp::getField(vam->vam.vamParameters.vruHighFrequencyContainer.heading.value, double) /
                      (double) DECI;
              vehdata.speed_ms =
                      asn1cpp::getField(vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue, double) /
                      (double) CENTI;
              vehdata.vamTimestamp = asn1cpp::getField(vam->vam.generationDeltaTime, long);
              vehdata.itsType = itsType_pedestrian;
              vehdata.timestamp_us = Simulator::Now().GetMicroSeconds();

              VDP::VDP_position_cartesian_t objectPosition;
              if (m_traci_client != nullptr) {
                  libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(
                          vehdata.lon, vehdata.lat);
                  objectPosition.x = traciPosition.x;
                  objectPosition.y = traciPosition.y;
              } else {
                  carla::Vector carlaPosition = m_opencda_client->getCartesian(vehdata.lon, vehdata.lat);
                  objectPosition.x = carlaPosition.x();
                  objectPosition.y = carlaPosition.y();

                  carla::Actor CV = m_opencda_client->GetActorById(std::stoi(m_id));
                  carla::Vector CVpos = m_opencda_client->getCartesian(CV.longitude(),CV.latitude());
                  vehdata.xDistAbs = long(objectPosition.x - CVpos.x())*CENTI;
                  vehdata.yDistAbs = long(objectPosition.y - CVpos.y())*CENTI;
              }
              vehdata.x = objectPosition.x;
              vehdata.y = objectPosition.y;

              // width and length are arbitrarily set,
              // otherwise it breaks IoU when matching LDM on CARLA side
              vehdata.vehicleWidth = OptionalDataItem<long>(long(6)) ;
              vehdata.vehicleLength = OptionalDataItem<long>(long(5));


              vehdata.perceivedBy = (long) -1;
//              if (m_LDM->lookup(vehdata.stationID,retVehicleData) == LDM::LDM_OK){
//                  vehdata.detected = retVehicleData.vehData.detected;
//              }
//              else {
//                  vehdata.detected = false;
//              }

              db_retval = m_LDM->insert(vehdata);
              if (db_retval != LDM::LDM_OK && db_retval != LDM::LDM_UPDATED) {
                  std::cerr << "Warning! Insert on the database for pedestrian "
                            << asn1cpp::getField(vam->header.stationId, int) << "failed!" << std::endl;
              }
          }
      }
//      if(m_VRU_role != VRU_ROLE_OFF){
//          buffer=(uint8_t *)malloc((dataIndication.data->GetSize ())*sizeof(uint8_t));
//          dataIndication.data->CopyData (buffer, dataIndication.data->GetSize ());
//          std::string packetContent((char *)buffer,(int) dataIndication.data->GetSize ());
//
//          /* Try to check if the received packet is really a VAM */
//          if (buffer[1]!=FIX_VAMID)
//          {
//              NS_LOG_ERROR("Warning: received a message which has messageID '"<<buffer[1]<<"' but '16' was expected.");
//              free(buffer);
//              return;
//          }
//
//          free(buffer);
//
//          /** Decoding **/
//          decoded_vam = asn1cpp::uper::decodeASN(packetContent, VAM);
//
//          if(bool(decoded_vam)==false) {
//              NS_LOG_ERROR("Warning: unable to decode a received VAM.");
//              return;
//          }
//
//          if(m_LDM != NULL){
//              //Update LDM
//              vLDM_handler(decoded_vam);
//          }
//
//          if(m_VAMReceiveCallback!=nullptr) {
//              m_VAMReceiveCallback(decoded_vam,from);
//          }
//      }
  }

  void
  cooperativePerception::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
  {
      if (m_type == "StationType_pedestrian"){
          return;
      }
   /* Implement CPM strategy here */
   m_cpm_received++;
   (void) from;
   int fromID = asn1cpp::getField(cpm->header.stationId,long);
   if (m_recvCPMmap.find(fromID) == m_recvCPMmap.end()) {
       m_recvCPMmap[fromID] = std::map<int, int>(); // First CPM from this vehicle
   }
      m_cluster_map_cp.clear();
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
          std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv2 from " << asn1cpp::getField(cpm->header.stationId,long) << " with " << PObjects_size << " perceived objects." << std::endl;
          for(int j=0; j<PObjects_size;j++)
            {
              LDM::returnedVehicleData_t PO_data;
              auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
              PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);

              auto ClassificationList = asn1cpp::sequenceof::getSeq(PO_seq->classification, ObjectClassWithConfidence, 0);
              auto objectClass = asn1cpp::getSeq(ClassificationList->objectClass,ObjectClass);
              ObjectClass_PR present = asn1cpp::getField(ClassificationList->objectClass.present,ObjectClass_PR);
              if (present == ObjectClass_PR_groupSubClass){
                  auto groupClass = asn1cpp::getSeq(objectClass->choice.groupSubClass,VruClusterInformation);
                  UpdateClusterInformation(cpm, PO_seq);
                  continue;

              }
                // First time we have received this object from this vehicle
              if(m_recvCPMmap[fromID].find(asn1cpp::getField(PO_seq->objectId,long)) == m_recvCPMmap[fromID].end())
                {
                  //If PO id is already in local copy of LDM
                  //(if two diff cavs send the same PO ID)
                  if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectId,long),PO_data) == LDM::LDM_OK)
                    {
                      //todo: we should try to merge the PO before assigning a new ID

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
                      m_LDM->insert(translateCPMdata(cpm,PO_seq,i,newID));
                      //Update recvCPMmap
                      m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)] = newID;
                    }
                  else
                    {
                      //Translate CPM data to LDM format
                      m_LDM->insert(translateCPMdata(cpm,PO_seq,i,-1));
                      //Update recvCPMmap
                      m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)] = asn1cpp::getField(PO_seq->objectId,long);
                    }
                }
              else
                {
                  // We have already received this object from this vehicle
                  m_LDM->insert(translateCPMdata(cpm,PO_seq,i,m_recvCPMmap[fromID][asn1cpp::getField(PO_seq->objectId,long)]));
                }
            }
        }
     }
   if (!m_cluster_map_cp.empty()) {
       m_LDM->updateClusterMap(m_cluster_map_cp, false);
   }//
  }
  vehicleData_t
  cooperativePerception::translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                           asn1cpp::Seq<PerceivedObject> object, int objectIndex, int newID)
  {
   vehicleData_t retval;
   retval.detected = true;
   retval.VAM = false;
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
   if(m_traci_client != nullptr)
     {
       libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertLonLattoXY (fromLon,fromLat);
       objectPosition.x = traciPosition.x;
       objectPosition.y = traciPosition.y;
     }
   else
     {
       carla::Vector carlaPosition = m_opencda_client->getCartesian (fromLon,fromLat);
       objectPosition.x = carlaPosition.x ();
       objectPosition.y = carlaPosition.y ();
     }
   VDP::VDP_position_latlon_t objectPositionGeo;
   objectPosition.x += asn1cpp::getField(object->position.xCoordinate.value,long)/CENTI;
   objectPosition.y += asn1cpp::getField(object->position.yCoordinate.value,long)/CENTI;
   retval.x = objectPosition.x;
   retval.y = objectPosition.y;
   if(m_traci_client != nullptr)
     {
       libsumo::TraCIPosition traciPosition = m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);
       objectPositionGeo.lon = traciPosition.x;
       objectPositionGeo.lat = traciPosition.y;
     }
   else
     {
       carla::Vector carlaPosition = m_opencda_client->getGeo (objectPosition.x,objectPosition.y);
       objectPositionGeo.lat = carlaPosition.x ();
       objectPositionGeo.lon = carlaPosition.y ();
     }

   retval.lon = objectPositionGeo.lon;
   retval.lat = objectPositionGeo.lat;

   retval.camTimestamp = asn1cpp::getField(cpm->payload.managementContainer.referenceTime,long);
   retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(object->measurementDeltaTime,long)*1000);
   retval.stationType = StationType_passengerCar;
   retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));


   auto ClassificationList = asn1cpp::sequenceof::getSeq(object->classification, ObjectClassWithConfidence, 0);
   auto objectClass = asn1cpp::getSeq(ClassificationList->objectClass,ObjectClass);
   ObjectClass_PR present = asn1cpp::getField(ClassificationList->objectClass.present,ObjectClass_PR);

    if (present == ObjectClass_PR_vehicleSubClass){
        TrafficParticipantType_t VehicleSubclass = asn1cpp::getField(objectClass->choice.vehicleSubClass,TrafficParticipantType_t);
        if (VehicleSubclass == TrafficParticipantType_passengerCar){
            retval.itsType = itsType_vehicle;
        }
        else if (VehicleSubclass == TrafficParticipantType_unknown){
            retval.itsType = itsType_motorcycle;
        }
    }
    else if (present == ObjectClass_PR_vruSubClass){
        auto VRUclassification = asn1cpp::getSeq(objectClass->choice.vruSubClass, VruProfileAndSubprofile);
        VruProfileAndSubprofile_PR vruSubClass_present = asn1cpp::getField(VRUclassification->present,VruProfileAndSubprofile_PR);
        if (vruSubClass_present == VruProfileAndSubprofile_PR_pedestrian){
            retval.itsType = itsType_pedestrian;
        }
        else if (vruSubClass_present == VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle){
            retval.itsType = itsType_bicycle;
        }
    }

    // FOR DEBUGGING
    if(m_opencda_client != nullptr)
      {
        carla::Vector pos = m_opencda_client->getCartesian (retval.lon, retval.lat);
        std::cout << "[OBJECT " << retval.stationID << "] -->" << "GeoPostiion" << "[" << retval.lon << ", " << retval.lat << "]" <<
        "  CartesianPosition: [" << pos.x ()<<", "<< pos.y () << "]" << "  CartPositionfromCPM: [" << retval.x<<", "<< retval.y << "]" << std::endl;
      }

   return retval;
  }

  bool
  cooperativePerception::UpdateClusterInformation(asn1cpp::Seq<CollectivePerceptionMessage> cpm,asn1cpp::Seq<PerceivedObject>object){
      std::map<size_t, ClusterInfo> cluster_map;
      ClusterInfo info;
      int fromID = asn1cpp::getField(cpm->header.stationId,long);
      float x {0};
      float y {0};
      auto ClassificationList = asn1cpp::sequenceof::getSeq(object->classification, ObjectClassWithConfidence, 0);
      auto objectClass = asn1cpp::getSeq(ClassificationList->objectClass,ObjectClass);

      auto groupClass = asn1cpp::getSeq(objectClass->choice.groupSubClass,VruClusterInformation);
      Identifier1B_t clusterID = asn1cpp::getField(groupClass->clusterId,Identifier1B_t);
      CardinalNumber1B_t clusterCardinality = asn1cpp::getField(groupClass->clusterCardinalitySize, CardinalNumber1B_t );
      auto shape_present = asn1cpp::getField(groupClass->clusterBoundingBoxShape->present, Shape_PR );
      if (shape_present == Shape_PR_circular)
      {
          auto circular_shape = asn1cpp::getSeq(groupClass->clusterBoundingBoxShape->choice.circular, CircularShape);
          CartesianCoordinate_t center_x = asn1cpp::getField(circular_shape->shapeReferencePoint->xCoordinate, CartesianCoordinate_t);
          CartesianCoordinate_t center_y = asn1cpp::getField(circular_shape->shapeReferencePoint->yCoordinate, CartesianCoordinate_t);
          StandardLength12b_t radius = asn1cpp::getField(circular_shape->radius, StandardLength12b_t);
          float cluster_center_x = static_cast<float>(center_x) /10;
          float cluster_center_y = static_cast<float>(center_y) /10;
          float cluster_radius = static_cast<float>(radius) /10;
          info.center = cv::Point2f(cluster_center_x,cluster_center_y);
          info.radius = cluster_radius;
          //debug
          x= cluster_center_x;
          y= cluster_center_y;
      }
      auto xSpeedAbs = (asn1cpp::getField(object->velocity->choice.cartesianVelocity.xVelocity.value,long));
      auto ySpeedAbs = (asn1cpp::getField(object->velocity->choice.cartesianVelocity.yVelocity.value,long));
      info.speed_ms = (sqrt (pow(xSpeedAbs,2) + pow(ySpeedAbs,2)))/CENTI;
      info.heading = asn1cpp::getField(object->angles->zAngle.value, double) / DECI;
      info.cardinality = clusterCardinality;
      info.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(object->measurementDeltaTime,long)*1000);
      info.perceivedBy = asn1cpp::getField(cpm->header.stationId,long);
      cluster_map[clusterID] = info;
      std::cout << "---FROM CPM MESSAGES---\n Cluster ID: " << clusterID << " (x ="<< x<<",y ="<< y<<")\n"
      <<" Size: "<< clusterCardinality << "\n Speed: " << info.speed_ms << "m/s\n Heading: " << info.heading << std::endl;
      m_recvCPMmap[fromID][clusterID] = clusterID;
      m_cluster_map_cp.insert({clusterID,info});

  }

}





