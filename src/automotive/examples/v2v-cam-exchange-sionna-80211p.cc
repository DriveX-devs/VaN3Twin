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
 */

// 802.11p
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
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
#include <fstream>
#include "ns3/sionna-helper.h"
#include <chrono>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchange80211p");

void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
      {
          { "0", 0 },
          { "1", 1 },
      };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}

static int packet_count=0;

BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles

void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(snr) && !std::isnan(sinr))
    {
      snr = sinr;
    }
  if (std::isnan(rssi) && !std::isnan(rsrp))
    {
      rssi = rsrp;
    }

  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos.y, pos.x);

  std::ofstream camFile;
  camFile.open("src/sionna/phy_without_sionna_11p.csv", std::ios::out | std::ios::app);
  camFile.seekp (0, std::ios::end);
  if (camFile.tellp() == 0)
    {
      camFile << "tx_id,rx_id,distance,rssi,snr" << std::endl;
    }
  
  camFile << cam->header.stationId << "," << my_stationID << "," << distance << "," << rssi << "," << snr << std::endl;
  camFile.close();
}

void savePRRs(Ptr<MetricSupervisor> metSup, uint64_t numberOfNodes)
{
  std::ofstream file;
  file.open("src/sionna/prr_without_sionna_11p.csv", std::ios::out | std::ios::app);
  file << "node_id,prr" << std::endl;
  for (int i = 1; i <= numberOfNodes; i++)
    {
      double prr = metSup->getAveragePRR_vehicle (i);
      file << i << "," << prr << std::endl;
    }
  file.close();
}

int main (int argc, char *argv[])
{
  // std::string phyMode ("OfdmRate6MbpsBW10MHz");
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  int up = 0;
  bool realtime = false;
  bool verbose = false; // Set to true to get a lot of verbose output from the PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 30.0; // Transmission power in dBm (default: 23 dBm)
  double sensitivity = -93.0;
  double snr_threshold = 4; // Default value
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 200 seconds)

  bool sionna = false;
  std::string server_ip = "";
  bool local_machine = false;
  bool verb = false;

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sionna", "Enable SIONNA usage", sionna);
  cmd.AddValue ("sionna-server-ip", "SIONNA server IP address", server_ip);
  cmd.AddValue ("sionna-local-machine", "SIONNA will be executed on local machine", local_machine);
  cmd.AddValue ("sionna-verbose", "SIONNA server IP address", verb);
  cmd.Parse (argc, argv);

  std::cout << "Start running v2v-cam-exchange-sionna-80211p simulation" << std::endl;

  SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();

  if (sionna)
    {
      sionnaHelper.SetSionna(sionna);
      sionnaHelper.SetServerIp(server_ip);
      sionnaHelper.SetLocalMachine(local_machine);
      sionnaHelper.SetVerbose(verb);
    }

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

  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();

  if (sionna)
    {
      sumoClient->SetSionnaUp();
    }

  uint64_t numberOfNodes_11p = numberOfNodes;

  Ptr<MetricSupervisor> metSup_11p = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_11p(m_baseline_prr);
  metSup_11p = &metSupObj_11p;
  metSup_11p->setTraCIClient(sumoClient);
  // This function enables printing the current and average latency and PRR for each received packet
  // metSup_11p->enablePRRVerboseOnStdout ();

  MobilityHelper mobility;

  // Create numberOfNodes nodes
  NodeContainer wifiNodes;
  wifiNodes.Create (numberOfNodes_11p);

  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  wifiPhy.SetPreambleDetectionModel ("ns3::ThresholdPreambleDetectionModel",
                                     "MinimumRssi", DoubleValue (sensitivity),
                                      "Threshold", DoubleValue (snr_threshold));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  channel->SetAttribute ("PropagationLossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
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

  // Supported "phyMode"s:
  // OfdmRate3MbpsBW10MHz, OfdmRate6MbpsBW10MHz, OfdmRate9MbpsBW10MHz, OfdmRate12MbpsBW10MHz, OfdmRate18MbpsBW10MHz, OfdmRate24MbpsBW10MHz, OfdmRate27MbpsBW10MHz
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, wifiNodes);

  mobility.Install (wifiNodes);

  PacketSocketHelper packetSocket;
  packetSocket.Install(wifiNodes);

  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    unsigned long nodeID = std::stol(vehicleID.substr (3)) - 1;
    
    // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
    Ptr<Socket> sock;
    sock=GeoNet::createGNPacketSocket(wifiNodes.Get(nodeID));
    // Set the proper AC, through the specified UP
    sock->SetPriority (up);

    Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),StationType_passengerCar,sumoClient,false,sock);
    // Setup the PRRsupervisor inside the BSContainer, to make each vehicle collect latency and PRR metrics
    bs_container->linkMetricSupervisor(metSup_11p);
    // This is needed just to simplify the whole application
    bs_container->disablePRRSupervisorForGNBeacons ();

    // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
    bs_container->addCAMRxCallback (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
    bs_container->setupContainer(true, false, false, false, false, false);

    basicServices.add(bs_container);

    std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
    double desync = ((double)std::rand()/RAND_MAX);
    bs_container->getCABasicService ()->startCamDissemination (desync);

    return wifiNodes.Get(nodeID);
  };
  
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
  {
    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));
    
    unsigned long intVehicleID = std::stol(vehicleID.substr (3));

    Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
    bsc->cleanup();
  };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));

  auto start_time = std::chrono::high_resolution_clock::now();

  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::cout << "\nTotal number of CAMs received: " << packet_count << std::endl;

  std::cout << "\nMetric Supervisor statistics for 802.11p" << std::endl;
  std::cout << "Average PRR: " << metSup_11p->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup_11p->getAverageLatency_overall () << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup_11p->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup_11p->getNumberTx_overall () << std::endl;

  savePRRs(metSup_11p, numberOfNodes_11p);
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_11p->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  Simulator::Destroy ();

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  std::cout << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;


  return 0;
}

