/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#define HORIZON_TIME 8000
#define NEGOTIATION_TIME 1000
#define DECELERATION_TIME 1000
#define STEP_TIME 100
#define LC_TIME_MSEC 1500
#define START_TIME 5000

#include "ns3/carla-module.h"

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include <iostream>
#include "ns3/MetricSupervisor.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/gn-utils.h"
#include "ns3/csv-utils.h"
#include "ns3/foresee.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleMCMExchange80211p");

// ******* DEFINE HERE ANY LOCAL GLOBAL VARIABLE, ACCESSIBLE FROM ANY FUNCTION IN THIS FILE *******
// Variables defined here should always be "static"
static int packet_count=0;
BSMap basicServices;
void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  // Logging the distance with respect to the RSSI
  double lat_sender=asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender=asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;
  //
  libsumo::TraCIPosition pos=basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  pos=basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);
  //
  double distance=haversineDist (lat_sender, lon_sender, pos.y, pos.x);
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate
  int up=0;
  int interfering_up=0;
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 33.0; // IEEE 802.11p transmission power in dBm (default: 23 dBm)
  xmlDocPtr rou_xml_file;
  double simTime = 2000.0; // Total simulation time (default: 100 seconds)
  bool sumo_gui = false;
  bool store_coordinations_in_csv = true;
  int seed = 42;

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_foresee/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_foresee/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("interfering-userpriority","User Priority for interfering traffic (default: 0, i.e., AC_BE)",interfering_up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sumo-gui", "Activate SUMO GUI", sumo_gui);
  cmd.AddValue("seed", "Random seed", seed);
  cmd.Parse (argc, argv);

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  // Check if there are enough nodes
  // This application requires at least three vehicles (as vehicle 3 is the one generating interfering traffic, it should exist)
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  // Create numberOfNodes nodes
  NodeContainer c;
  c.Create (numberOfNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  // Set up the IEEE 802.11p model and PHY layer
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // We need a QosWaveMac, as we need to enable QoS and EDCA
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging, only if verbose is true
    }

  // In order to properly set the IEEE 802.11p modulation for broadcast messages, you must always specify a "NonUnicastMode" too
  // This line sets the modulation and rata rate
  // Supported "phyMode"s:
  // OfdmRate3MbpsBW10MHz, OfdmRate6MbpsBW10MHz, OfdmRate9MbpsBW10MHz, OfdmRate12MbpsBW10MHz, OfdmRate18MbpsBW10MHz, OfdmRate24MbpsBW10MHz, OfdmRate27MbpsBW10MHz
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Enable saving to Wireshark PCAP traces
  wifiPhy.EnablePcap ("v2v-80211p-foresee-mcm", devices.Get (22));

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  MobilityHelper mobility;
  mobility.Install (c);
  // Set up the TraCI interface and start SUMO with the default parameters
  // The simulation time step can be tuned by changing "SynchInterval"
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (seed));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (10)));

  // Set up a Metricsupervisor
  // This module enables a trasparent and seamless collection of one-way latency (in ms) and PRR metrics
  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj(m_baseline_prr);
  metSup = &metSupObj;
  metSup->setTraCIClient(sumoClient);
  PacketSocketHelper packetSocket;
  packetSocket.Install(c);

  std::unordered_map<ulong, foresee> lc_model;
  // Set the coordination avoidance range to check ahead of ego vehicle (in meters)
  double ca_range = 200;

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  double avg_speed_cars = 33.3;  // m/s
  double avg_speed_trucks = 22.2;  // m/s
  double deviation = 0.2;   // 20%

  double min_speed_cars = avg_speed_cars * (1.0 - deviation);
  double min_speed_trucks = avg_speed_trucks * (1.0 - deviation);
  double max_speed_cars = avg_speed_cars * (1.0 + deviation);
  double max_speed_trucks = avg_speed_trucks * (1.0 + deviation);

  // Random number generator
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> dist1(min_speed_cars, max_speed_cars);
  std::uniform_real_distribution<double> dist2(min_speed_trucks, max_speed_trucks);

  bool use_foresee = true;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3))-1;

      std::string type = sumoClient->vehicle.getTypeID (vehicleID);

      double speed = type == "Car0" ? dist1(gen) : dist2(gen);

      // Set the desired speed
      sumoClient->vehicle.setMaxSpeed(vehicleID, speed);
      // Prevent uncontrolled lane change
      sumoClient->vehicle.setParameter(vehicleID, "laneChangeMode", "256");

      // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
      Ptr<Socket> sock;
      sock=GeoNet::createGNPacketSocket(c.Get(nodeID));
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);
      StationType_t st_type = type == "Car0" ? StationType_passengerCar : StationType_lightTruck;
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),st_type,sumoClient,false,sock);
      // Setup the PRRsupervisor inside the BSContainer, to make each vehicle collect latency and PRR metrics
      bs_container->linkMetricSupervisor(metSup);
      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
      // bs_container->addMCMRxCallback (std::bind(&receiveMCM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      // bs_container->addMCMRxCallback (std::bind(&receiveMCM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      bs_container->addCAMRxCallback (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      bs_container->setupContainer(true,false,false,false,true,false);

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      lc_model[nodeID].setDesiredSpeed (speed);
      lc_model[nodeID].setNode(c.Get(nodeID));
      lc_model[nodeID].setStationType(st_type);
      lc_model[nodeID].setLDM (bs_container->getLDM());
      lc_model[nodeID].setVDP (bs_container->getVDP());
      lc_model[nodeID].setVehicleID (vehicleID);
      lc_model[nodeID].setTraciAPI(sumoClient);
      lc_model[nodeID].setNumberOfLanes();
      lc_model[nodeID].setCoordinationAvoidanceRange(ca_range);
      lc_model[nodeID].setMCBasicService(bs_container->getMCBasicService());
      lc_model[nodeID].addMCMRxCallback ();
      lc_model[nodeID].setStartTime(START_TIME);
      lc_model[nodeID].setNegotiationTime(NEGOTIATION_TIME);
      lc_model[nodeID].setVerbose();
      lc_model[nodeID].setSeed(seed);
      lc_model[nodeID].setRegisterLog();
      if (use_foresee)
      {
        lc_model[nodeID].WrapperFORESEEMobilityModel(use_foresee);
        use_foresee = false;
      }
      else use_foresee = true;

      // Start transmitting CAMs
      // We randomize the instant in time in which the CAM dissemination is going to start
      // This simulates different startup times for the OBUs of the different vehicles, and
      // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
      // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start
      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);
      bs_container->getCABasicService ()->startCamDissemination (desync);
      // bs_container->getMCBasicService()->startMCMDissemination(desync);

      if (true)
      {
        std::cout << "\n[CURRENT TIME CHECK]" << std::endl;
        std::cout << Simulator::Now().GetSeconds() << "s" << std::endl;
      }

      return c.Get(nodeID);
    };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
    {
      /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));

      // Turn off the Basic Services and the ETSI ITS-G5 stack for the vehicle
      // which has exited from the simulated scenario, and should be thus no longer considered
      // We need to get the right Ptr<BSContainer> based on the station ID (not the nodeID used
      // as index for the nodeContainer), so we don't use "-1" to compute "intVehicleID" here
      unsigned long intVehicleID = std::stol(vehicleID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
      bsc->cleanup();
    };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));
  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  if (store_coordinations_in_csv)
  {
    std::cout << "Writing CSV log after simulation..." << std::endl;
    // Create the header
    std::ofstream file;
    file.open("coordinations_seed" + std::to_string(seed) + ".csv", std::ios::out | std::ios::trunc);
    // Write CSV header
    file << "coordination_id,"
          << "sim_time_ms,"
          << "desired_speed_hv,"
          << "min_lane_speed_hv,"
          << "min_lane_speed_target,"
          << "type_hv,"
          << "type_rv,"
          << "type_rvahead,"
          << "speed_hv,"
          << "speed_rv,"
          << "speed_rvahead,"
          << "acc_hv,"
          << "acc_rv,"
          << "acc_rvahead,"
          << "gap_hv_rv,"
          << "gap_hv_rvahead,"
          << "rel_speed_hv_rv,"
          << "rel_speed_hv_rvahead,"
          << "dec_rv_requested,"
          << "acc_rvahead_requested,"
          << "time_rv_requested,"
          << "time_rvahead_requested,"
          << "mean_speed_ahead,"
          << "mean_speed_behind,"
          << "std_speed_ahead,"
          << "std_speed_behind,"
          << "num_vehicles_ahead,"
          << "num_vehicles_behind,"
          << "density_target_lane_ahead,"
          << "density_target_lane_behind,"
          << "execution_success"
          << "\n";

    for (auto it = lc_model.begin(); it != lc_model.end(); ++it)
    {
        auto coordination_log = it->second.getCoordinationLog();
        for (auto s = coordination_log.begin(); s != coordination_log.end(); ++s)
        {
          file << s->coordination_id               << ","
            << s->sim_time_ms                   << ","
            << s->desired_speed_hv              << ","
            << s->lane_speed_hv                 << ","
            << s->lane_speed_target             << ","
            << s->type_hv                       << ","
            << s->type_rv                       << ","
            << s->type_rvahead                  << ","
            << s->speed_hv                      << ","
            << s->speed_rv                      << ","
            << s->speed_rvahead                 << ","
            << s->acc_hv                        << ","
            << s->acc_rv                        << ","
            << s->acc_rvahead                   << ","
            << s->gap_hv_rv                     << ","
            << s->gap_hv_rvahead                << ","
            << s->rel_speed_hv_rv               << ","
            << s->rel_speed_hv_rvahead          << ","
            << s->dec_rv_requested              << ","
            << s->acc_rvahead_requested         << ","
            << s->time_rv_requested             << ","
            << s->time_rvahead_requested        << ","
            << s->mean_speed_ahead              << ","
            << s->mean_speed_behind             << ","
            << s->std_speed_ahead               << ","
            << s->std_speed_behind              << ","
            << s->num_vehicles_ahead            << ","
            << s->num_vehicles_behind           << ","
            << s->density_target_lane_ahead     << ","
            << s->density_target_lane_behind    << ","
            << s->execution_success             << "\n";
        }
    }
    file.close();
    std::cout << "Write operation finished" << std::endl;
  }

  std::cout << "End" << std::endl;

  Simulator::Destroy ();

  return 0;
}
