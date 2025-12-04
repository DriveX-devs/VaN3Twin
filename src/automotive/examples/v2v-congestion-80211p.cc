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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr> (initial IEEE 802.11p example)
 * Author: Junling Bu <linlinjavaer@gmail.com> (initial IEEE 802.11p example)
 * Author: Francesco Raviglione <francescorav.es483@gmail.com> (IEEE 802.11p simple CAM exchange application)
 *
 * This is a simple example of a ms-van3t V2V communication scenario configured with a single .cc file,
 * where vehicles exchange Cooperative Awareness Messages (CAMs) using the IEEE 802.11p standard.
 * The user can specify several parameters, including the priority (Access Category) for the transmission
 * of CAMs. BSContainers are used to simplify the configuration of the ETSI C-ITS stack of each vehicle.
 * In this scenario, all vehicles transmit CAMs according to the ETSI standards, and one vehicle (vehicle 3)
 * sends a heavy interfering traffic, without useful informative content, to simulate a congested channel.
 * Through the --interfering-userpriority option, the user can specify the Access Category (AC) for the
 * interfering traffic, which is broadcasted by vehicle 3.
 * When a new CAM is received by one of the vehicles, the callback "receiveCAM()" is called, and the stationID of
 * the received is available in "my_stationID".
 * Currently, the function just counts the total number of CAMs, but it can be customized to impement more complex
 * approaches.
 * As output, the simulation provides, thanks to the PRRSupervisor module, the average latency and PRR over the
 * whole simulation, and the average one-way latency for each vehicle up to vehicle 4.
 * The reported latency of vehicle 3 is expected to be 0 as it transmits interfering traffic (so, no ETSI-compliant
 * message is transmitted), that is not considered by the PRRSupervisor.
 */
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
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/gn-utils.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/DCC.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VCAMCongestion80211p");

// ******* DEFINE HERE ANY LOCAL GLOBAL VARIABLE, ACCESSIBLE FROM ANY FUNCTION IN THIS FILE *******
// Variables defined here should always be "static"
static int cam_packet_count=0;
static int cpm_packet_count=0;
BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles
// ************************************************************************************************

// If you want to make a comparison with a received CAM MAC address, you can use:
// from == getGNAddress (0,Mac48Address(comparison_string)), where "comparison_string" is something like "00:00:00:00:00:09"

// Useful tip: you can get the latitude and longitude position of a given vehicle at any time with:
//   libsumo::TraCIPosition pos=<MobilityClient>->TraCIAPI::vehicle.getPosition(<string ID of the vehicle>);
//   pos=<MobilityClient>->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);
// <MobilityClient> should be the right Ptr<TraciClient> for the current vehicle, which can be retrieved from the
//   global Basic Services container with basicServices.get(my_stationID)->getTraCIclient ()
// The string ID of the vehicle should match the one in the XML file, i.e., for vehicle 7, the string id should be "veh7"
// After these two lines, pos.y will contain the latitude of the vehicle, while pos.x the longitude of the vehicle
void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  cam_packet_count++;
}

void receiveCPM(asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  cpm_packet_count++;
}

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zeros
      if (socket->Send (Create<Packet> (pktSize)) != -1 && pktCount%100==0)
        {
          // std::cout << "Interfering packet sent" << std::endl;
        }
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate
  int up=0;
  int interfering_up=0;
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 33.0; // IEEE 802.11p transmission power in dBm
  xmlDocPtr rou_xml_file;
  double simTime = 150.0; // Total simulation time (default: 100 seconds)
  int dcc_nodes = 7;

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars_7.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map_7.sumo.cfg";

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

  if(numberOfNodes<3)
    {
      NS_FATAL_ERROR("Fatal error: at least three vehicles are required.");
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
  wifiPhy.EnablePcap ("v2v-congestion-80211p", devices);

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
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (false));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.5)));

  // Set up a Metricsupervisor
  // This module enables a trasparent and seamless collection of one-way latency (in ms) and PRR metrics
  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new PRRsupervisor object
  MetricSupervisor metSupObj(m_baseline_prr);
  metSup = &metSupObj;
  metSup->setTraCIClient(sumoClient);
  // This function enables printing the current and average latency and PRR for each received packet
  metSup->setChannelTechnology("80211p");
  metSup->enableCBRVerboseOnStdout();
  metSup->enableCBRWriteToFile();
  metSup->setCBRWindowValue(100);
  metSup->setCBRAlphaValue(0.1);
  metSup->setSimulationTimeValue(simTime);
  metSup->setNodeContainer(c);
  metSup->startCheckCBR(dcc_nodes);

  // Create a new socket for the generation of broadcast interfering traffic
  // With the aim of sending generic broadcast packets, we use a PacketSocket
  // This kind of facility enables the transmission of data with a customizable
  // Ethertype, and no specific stack for the higher layers
  // As we are just sending interfering traffic (without any useful informative content),
  // we can use the local experimental Ethertype (0x88B5)
  PacketSocketHelper packetSocket;
  packetSocket.Install(c);
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  std::unordered_map<uint32_t, Ptr<Socket>> source_interfering;
  uint32_t number = c.GetN();
  for(uint32_t i = 0; i < number; i++)
    {
      Ptr<Socket> socket = Socket::CreateSocket (c.Get (i), tid);
      PacketSocketAddress local_source_interfering;
      local_source_interfering.SetSingleDevice (c.Get(i)->GetDevice(0)->GetIfIndex ());
      local_source_interfering.SetPhysicalAddress (c.Get(i)->GetDevice(0)->GetAddress());
      local_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
      if (socket->Bind (local_source_interfering) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
      PacketSocketAddress remote_source_interfering;
      remote_source_interfering.SetSingleDevice (c.Get(i)->GetDevice(0)->GetIfIndex());
      // Set the broadcast MAC address as interfering traffic will be broadcasted by vehicle 3
      remote_source_interfering.SetPhysicalAddress (c.Get(i)->GetDevice(0)->GetBroadcast());
      remote_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
      socket->Connect (remote_source_interfering);
      // Important: this line lets you set the AC of the interfering traffic, through a User Priority (UP) value, like in real Linux kernels/OS
      socket->SetPriority (interfering_up); // Setting the priority of the interfering traffic from vehicle 0
      source_interfering[i] = socket;
    }

  std::unordered_map<Ptr<Node>, Ptr<DCC>> dcc_per_node;
  for (uint8_t i = 0; i < dcc_nodes; i++)
  {
    dcc_per_node[c.Get(i)] = CreateObject<DCC>();
  }

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  // Important: what you write inside setupNewWifiNode() will be executed every time a new vehicle enters the simulation in SUMO
  // This kind of "std::function" is called lambda function, and it can access all variables outside its scope, thanks to the [&] capture
  // We setup here the ETSI stack for each vehicle (except the one generating interfering traffic), thanks to the BSContainer object
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      std::cout << "Setting up the ETSI ITS-G5 stack for vehicle " << vehicleID << std::endl;
      unsigned long nodeID = std::stol(vehicleID.substr (3))-1;
      uint32_t id = source_interfering[nodeID]->GetNode()->GetId();
      if (nodeID == 2)
        Simulator::ScheduleWithContext (id,
                                      Seconds (1.0), &GenerateTraffic_interfering,
                                      source_interfering[nodeID], 2250, simTime*2000, MilliSeconds (2));

      // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
      Ptr<Socket> sock;
      sock=GeoNet::createGNPacketSocket(c.Get(nodeID));
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);

      // Create a new Basic Service Container object, which includes, for the current vehicle, both the ETSI CA Basic Service, for the transmission/reception
      // of periodic CAMs, and the ETSI DEN Basic Service, for the transmission/reception of event-based DENMs
      // An ETSI Basic Services container is a wrapper class to enable easy handling of both CAMs and DENMs
      // The station ID is set to be equal to the SUMO ID without "veh" (i.e., the station ID of "veh1" will be "1")
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),StationType_passengerCar,sumoClient,false,sock);
      // Setup the Metricupervisor inside the BSContainer, to make each vehicle collect latency and Metric metrics
      bs_container->linkMetricSupervisor(metSup);
      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
      bs_container->addCAMRxCallback (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      // Setup the new ETSI Basic Services container
      // The first parameter is true is you want to setup a CA Basic Service (for sending/receiving CAMs)
      // The second parameter should be true if you want to setup a DEN Basic Service (for sending/receiving DENMs)
      // The third parameter should be true if you want to setup a VRU Basic Service (for sending/receiving VAMs)

      bs_container->addCPMRxCallback (std::bind(&receiveCPM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      bs_container->setupContainer(true,false,false,false);
      Ptr<GeoNet> gn = bs_container->getGeoNet();

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      // Setup DCC for both internal behavior and GeoNet link
      if (dcc_per_node.find(c.Get(nodeID))!=dcc_per_node.end())
        {
          std::cout << "DCC enabled for vehicle " << vehicleID << std::endl;
          dcc_per_node[c.Get(nodeID)]->SetupDCC(vehicleID, metSup, c.Get(nodeID), "adaptive", 200);
          dcc_per_node[c.Get(nodeID)]->setBitRate (6e6);
          gn->setDCC (dcc_per_node[c.Get(nodeID)]);
          dcc_per_node[c.Get(nodeID)]->StartDCC();
        }

      // Start transmitting CAMs
      // We randomize the instant in time in which the CAM dissemination is going to start
      // This simulates different startup times for the OBUs of the different vehicles, and
      // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
      // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start
      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);
      bs_container->getCABasicService ()->startCamDissemination (desync);
      // bs_container->getCPBasicService()->startCpmDissemination();

      return c.Get(nodeID);
    };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [&] (Ptr<Node> exNode, std::string vehicleID)
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

  metSup->startChannelOccupationBytesPerSecondsPerSquareMeter();

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));
  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the Metricsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::cout << "Average CBR: " << metSup->getAverageCBROverall() << std::endl;
  std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
  std::cout << "TX overall: " << metSup->getNumberTx_overall() << std::endl;
  std::cout << "RX overall: " << cam_packet_count + cpm_packet_count << std::endl;


  Simulator::Destroy ();

  return 0;
}
