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

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/sionna-helper.h"
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/cv2x-module.h"
#include <ns3/node-list.h>
#include <unistd.h>
#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchangeCv2x");

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
  camFile.open("src/sionna/phy_with_sionna_cv2x.csv", std::ios::out | std::ios::app);
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
  file.open("src/sionna/prr_with_sionna_cv2x.csv", std::ios::out | std::ios::app);
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
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  int up = 0;
  bool realtime = false;
  bool verbose = false; // Set to true to get a lot of verbose output from the PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 30.0; // Transmission power in dBm (default: 23 dBm)
  // double sensitivity = -93.0;
  // double snr_threshold = 10; // Default value
  // double sinr_threshold = 10; // Default value
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 200 seconds)

  /*** 0.b LENA + V2X Options ***/
  double ueTxPower = 30.0;                // Transmission power in dBm
  double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
  uint32_t mcs = 20;                      // Modulation and Coding Scheme
  bool harqEnabled = false;               // Retransmission enabled (harq not available yet)
  bool adjacencyPscchPssch = true;        // Subchannelization scheme
  bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
  uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
  // uint16_t numSubchannel = 3;             // Number of subchannels per subframe
  uint16_t numSubchannel = 1;             // Number of subchannels per subframe
  uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
  uint16_t pRsvp = 20;                    // Resource reservation interval
  // uint16_t t1 = 4;                        // T1 value of selection window
  uint16_t t1 = 2;                        // T1 value of selection window
  // uint16_t t2 = 100;                      // T2 value of selection window
  uint16_t t2 = 81;                      // T2 value of selection window
  uint16_t slBandwidth = 10;                   // Sidelink bandwidth
  bool hardcoded_bw = true;
  bool m_metric_sup = true;

  // (T2-T1+1) x (1/(2^numerology)) < reservation period
  // (81-2+1) x (1/2^2) < 20

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

  std::cout << "Start running v2v-cam-exchange-sionna-cv2x.cc simulation" << std::endl;

  SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();

  if (sionna)
    {
      sionnaHelper.SetSionna(sionna);
      sionnaHelper.SetServerIp(server_ip);
      sionnaHelper.SetLocalMachine(local_machine);
      sionnaHelper.SetVerbose(verb);
    }

  NS_LOG_INFO("Configuring C-V2X channel...");
  /*** 0.c V2X Configurations ***/
  /* Set the UEs power in dBm */
  Config::SetDefault ("ns3::cv2x_LteUePhy::TxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
  /* Enable V2X communication on PHY layer */
  Config::SetDefault ("ns3::cv2x_LteUePhy::EnableV2x", BooleanValue (true));

  /* Set power */
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::cv2x_LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

  if (!hardcoded_bw)
    {
      if (adjacencyPscchPssch)
        {
          slBandwidth = sizeSubchannel * numSubchannel;
        }
      else
        {
          slBandwidth = (sizeSubchannel+2) * numSubchannel;
        }
    }


  /* Configure for UE selected */
  Config::SetDefault ("ns3::cv2x_LteUeMac::UlBandwidth", UintegerValue (slBandwidth));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnableV2xHarq", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnableAdjacencyPscchPssch", BooleanValue (adjacencyPscchPssch));
  Config::SetDefault ("ns3::cv2x_LteUeMac::EnablePartialSensing", BooleanValue (partialSensing));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlGrantMcs", UintegerValue (mcs));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlPrsvp", UintegerValue (pRsvp));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SlProbResourceKeep", DoubleValue (probResourceKeep));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT1", UintegerValue (t1));
  Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT2", UintegerValue (t2));

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  /* Use the realtime scheduler of ns3 */
  if(realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.d Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create LTE objects   ***/
  Ptr<cv2x_PointToPointEpcHelper> epcHelper = CreateObject<cv2x_PointToPointEpcHelper> ();
  Ptr<cv2x_LteHelper> lteHelper = CreateObject<cv2x_LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Disable eNBs for out-of-coverage modelling
  lteHelper->DisableNewEnbPhy();

  /* V2X */
  Ptr<cv2x_LteV2xHelper> lteV2xHelper = CreateObject<cv2x_LteV2xHelper> ();
  lteV2xHelper->SetLteHelper (lteHelper);

  /* Configure eNBs' antenna parameters before deploying them. */
  lteHelper->SetEnbAntennaModelType ("ns3::cv2x_NistParabolic3dAntennaModel");
  lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
  Config::SetDefault ("ns3::cv2x_LteEnbNetDevice::UlEarfcn", StringValue ("54990")); // EARFCN 54990 -> 5855-5890-5925 MHz
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::cv2x_CniUrbanmicrocellPropagationLossModel"));
  NS_LOG_INFO("Antenna parameters set. Current EARFCN: 54990, current frequency: 5.89 GHz");

  /*** 2. Create Internet and ipv4 helpers ***/
  InternetStackHelper internet;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  /*** 3. Create containers for UEs and eNB ***/
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create(1);
  ueNodes.Create(numberOfNodes);

  /*** 4. Create and install mobility (SUMO will be attached later) ***/
  MobilityHelper mobility;
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);
  /* Set the eNB to a fixed position */
  Ptr<MobilityModel> mobilityeNBn = enbNodes.Get (0)->GetObject<MobilityModel> ();
  mobilityeNBn->SetPosition (Vector (0, 0, 20.0)); // set eNB to fixed position - it is still disabled

  /*** 5. Install LTE Devices to the nodes + assign IP to UEs + manage buildings ***/
  lteHelper->InstallEnbDevice (enbNodes); // If you don't do it, the simulation crashes

  /* Required to use NIST 3GPP model */
  BuildingsHelper::Install (ueNodes);
  BuildingsHelper::Install (enbNodes);
  // BuildingsHelper::MakeMobilityModelConsistent (); Removed because DEPRECATED from 3.31
  for (NodeList::Iterator nit = NodeList::Begin (); nit != NodeList::End (); ++nit)
    {
      Ptr<MobilityModel> mm = (*nit)->GetObject<MobilityModel> ();
      if (mm != 0)
        {
          Ptr<MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo> ();
          NS_ABORT_MSG_UNLESS (0 != bmm, "node " << (*nit)->GetId () << " has a MobilityModel that does not have a MobilityBuildingInfo");
          bmm->MakeConsistent (mm);
        }
    }

  lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  /* Install the IP stack on the UEs */
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;

  /* Assign IP address to UEs */
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      NS_LOG_INFO("Node "<< ueNode->GetId () << " has been assigned an IP address: " << ueNode->GetObject<Ipv4> ()->GetAddress(1,0).GetLocal());
    }

  NS_LOG_INFO("Configuring sidelink...");

  /* Create sidelink groups */
  std::vector<NetDeviceContainer> txGroups;
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueLteDevs, numberOfNodes);

  /* Compute average number of receivers associated per transmitter and vice versa */
  std::map<uint32_t, uint32_t> txPerUeMap;
  std::map<uint32_t, uint32_t> groupsPerUe;
  std::vector<NetDeviceContainer>::iterator gIt;
  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
    {
      uint32_t numDevs = gIt->GetN();

      uint32_t nId;

      for(uint32_t i=1; i< numDevs; i++)
        {
          nId = gIt->Get(i)->GetNode()->GetId();
          txPerUeMap[nId]++;
        }
    }

  std::map<uint32_t, uint32_t>::iterator mIt;
  for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
    {
      groupsPerUe [mIt->second]++;
    }

  uint32_t groupL2Address = 255;
  std::vector<Ipv4Address> ipAddresses;
  Ipv4Address groupAddress4 ("225.0.0.0");

  /* Create Sidelink bearers */
  Ptr<cv2x_LteSlTft> tft = Create<cv2x_LteSlTft> (cv2x_LteSlTft::BIDIRECTIONAL, groupAddress4, groupL2Address);
  lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), ueLteDevs, tft);

  /* Creating sidelink configuration */
  Ptr<cv2x_LteUeRrcSl> ueSidelinkConfiguration = CreateObject<cv2x_LteUeRrcSl>();
  ueSidelinkConfiguration->SetSlEnabled(true);
  ueSidelinkConfiguration->SetV2xEnabled(true);

  cv2x_LteRrcSap::SlV2xPreconfiguration preconfiguration;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

  cv2x_SlV2xPreconfigPoolFactory pFactory;
  pFactory.SetHaveUeSelectedResourceConfig (true);
  pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
  pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
  pFactory.SetSizeSubchannel (sizeSubchannel);
  pFactory.SetNumSubchannel (numSubchannel);
  pFactory.SetStartRbSubchannel (startRbSubchannel);
  pFactory.SetStartRbPscchPool (0);
  pFactory.SetDataTxP0 (-4);
  pFactory.SetDataTxAlpha (0.9);

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
  ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);

  lteHelper->InstallSidelinkV2xConfiguration (ueLteDevs, ueSidelinkConfiguration);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  if (sionna)
    {
      sumoClient->SetSionnaUp();
    }
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
  std::string sumo_additional_options = "--verbose true";
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID, TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    unsigned long vehID = std::stol(vehicleID.substr (3));
    unsigned long nodeID = vehID - 1;

    Ptr<NetDevice> netDevice;
    Ptr<Socket> sock;
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Node> includedNode = ueNodes.Get(nodeID);
    sock = Socket::CreateSocket (includedNode, tid);
    if (sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
      }
    Ipv4Address groupAddress4 ("225.0.0.0");
    sock->Connect (InetSocketAddress (groupAddress4, 19));

    netDevice = includedNode->GetDevice(0);
    Ptr<cv2x_LteUeNetDevice> cv2x_device = DynamicCast<cv2x_LteUeNetDevice>(netDevice);
    cv2x_device->GetPhy()->GetSlSpectrumPhy()->GetChannel()->SetAttribute ("MaxLossDb", DoubleValue(120.0));

    Ptr<BSContainer> bs_container = CreateObject<BSContainer>(vehID,StationType_passengerCar,sumoClient,false,sock);
    bs_container->addCAMRxCallback (std::bind(&receiveCAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    bs_container->linkMetricSupervisor (metSup);
    bs_container->disablePRRSupervisorForGNBeacons();
    bs_container->setupContainer(true,false,false,false,false,false);
    basicServices.add(bs_container);
    std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
    double desync = ((double)std::rand()/RAND_MAX);
    bs_container->getCABasicService ()->startCamDissemination (desync);

    return ueNodes.Get(nodeID);
  };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
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

  std::ofstream outputFile("src/output.txt");
  if (!outputFile.is_open())
    {
      std::cerr << "Unable to open file";
    }

  outputFile << "\nTotal number of CAMs received: " << packet_count << std::endl;

  outputFile << "\nMetric Supervisor statistics for C-V2X" << std::endl;
  outputFile << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
  outputFile << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
  outputFile << "RX packet count (from PRR Supervisor): " << metSup->getNumberRx_overall () << std::endl;
  outputFile << "TX packet count (from PRR Supervisor): " << metSup->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_nr->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  savePRRs(metSup, numberOfNodes);

  Simulator::Destroy ();

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  outputFile << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}

