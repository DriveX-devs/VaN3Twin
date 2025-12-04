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
 *  Giuseppe Avino, Politecnico di Torino (giuseppe.avino@polito.it)
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/
#include "simpleCAMSender-gps-tc.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/vdpGPSTraceClient.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"


namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("simpleCAMSender");

  NS_OBJECT_ENSURE_REGISTERED(simpleCAMSender);

  TypeId
  simpleCAMSender::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::simpleCAMSender")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<simpleCAMSender> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&simpleCAMSender::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("GPSClient",
            "GPSTC client",
            PointerValue (0),
            MakePointerAccessor (&simpleCAMSender::m_gps_tc_client),
            MakePointerChecker<GPSTraceClient> ())
        .AddAttribute ("DCC",
                   "",
                   BooleanValue(false),
                   MakeBooleanAccessor (&simpleCAMSender::m_enable_dcc),
                   MakeBooleanChecker ())
        .AddAttribute("DCCWindow",
                   "",
                   IntegerValue(0),
                   MakeIntegerAccessor (&simpleCAMSender::m_dcc_time_window),
                   MakeIntegerChecker<int>())
        .AddAttribute("DCCModality",
                   "",
                   StringValue("reactive"),
                   MakeStringAccessor (&simpleCAMSender::m_dcc_modality),
                   MakeStringChecker())
        .AddAttribute ("MetricSupervisor",
                       "Metric Supervisor",
                       PointerValue (0),
                       MakePointerAccessor (&simpleCAMSender::m_met_sup),
                       MakePointerChecker<MetricSupervisor> ());
        return tid;
  }

  simpleCAMSender::simpleCAMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_gps_tc_client = nullptr;

    m_cam_sent = 0;
  }

  simpleCAMSender::~simpleCAMSender ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  simpleCAMSender::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  simpleCAMSender::StartApplication (void)
  {

    /*
     * This application connects with the gps-tc module and simply generates CAM.
     * In this case, the position of the vehicle will arrive from a GPS trace placed
     * in src/gps-tc/examples/GPS-Traces-Sample, NOT FROM SUMO!
     */

    NS_LOG_FUNCTION(this);

    // Ensure that the mobility client has been set
    if(m_gps_tc_client==nullptr)
    {
        NS_FATAL_ERROR("No mobility client specified in simpleCAMSender");
    }
    m_id = m_gps_tc_client->getID();

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();
    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);

    /* Create the socket for TX and RX */
    TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    /* TX socket for CAMs */
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

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setSocketRx (m_socket);

    m_caService.setStationProperties (std::stol(m_id), StationType_passengerCar);
    m_denService.setStationProperties (std::stol(m_id), StationType_passengerCar);

    m_denService.addDENRxCallback (std::bind(&simpleCAMSender::receiveDENM,this,std::placeholders::_1,std::placeholders::_2));
    m_denService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallbackExtended (std::bind(&simpleCAMSender::receiveCAM,this,std::placeholders::_1,std::placeholders::_2, std::placeholders::_3,std::placeholders::_4, std::placeholders::_5));
    m_caService.setRealTime (m_real_time);

    VDP* gpstc_vdp = new VDPGPSTraceClient(m_gps_tc_client,m_id);
    m_caService.setVDP(gpstc_vdp);
    m_denService.setVDP(gpstc_vdp);

    m_caService.SetLogTriggering(true, "cam-log.txt");

    /* Schedule CAM dissemination */
    std::srand(Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);
    m_caService.startCamDissemination(desync);
    if (m_enable_dcc)
      {
        m_dcc = CreateObject<DCC>();
        m_dcc->SetupDCC(m_id, m_met_sup, m_node, m_dcc_modality, m_dcc_time_window);
        m_geoNet->setDCC (m_dcc);
        m_dcc->StartDCC();
      }
  }

  void
  simpleCAMSender::StopApplication ()
  {
    NS_LOG_FUNCTION(this);

    uint64_t cam_sent;

    m_gps_tc_client->StopUpdates();

    cam_sent = m_caService.terminateDissemination ();

    /*std::cout << "Vehicle " << m_id
              << " has sent " << cam_sent
              << " CAMs" << std::endl;*/
  }

  void
  simpleCAMSender::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  simpleCAMSender::receiveCAM (asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
  {
    /* Implement CAM strategy here */
    /*
    std::cout <<"VehicleID: " << m_id
            <<" | Rx CAM from "<<cam->header.stationId
            <<" | Remote vehicle position: ("<<asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO<<","
            <<asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO<<")"<<std::endl;
    */
    /* Write on a CSV file the time, sender, receiver, rx power*/
    Time now = Simulator::Now ();
    double time = now.GetSeconds ();
    std::ofstream logFile;
    logFile.open("cam-reception-log.csv", std::ios_base::app);
    logFile << "Time: " << time << ", Sender: " << cam->header.stationId << ", Receiver: " << m_id << ", RSSI: " << phy_info.rssi << std::endl;
    logFile.close();
  }

  void
  simpleCAMSender::receiveDENM (denData denm, Address from)
  {
    /* This is just a sample dummy receiveDENM function. The user can customize it to parse the content of a DENM when it is received. */
    (void) denm; // Contains the data received from the DENM
    (void) from; // Contains the address from which the DENM has been received
    std::cout<<"Received a new DENM."<<std::endl;
  }

}



