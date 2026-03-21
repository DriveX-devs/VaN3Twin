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
#include "ns3/gn-utils.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/sionna-helper.h"
#include <unistd.h>
#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchangeNrv2x");

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
  camFile.open("src/sionna/phy_with_sionna_nrv2x.csv", std::ios::out | std::ios::app);
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
  file.open("src/sionna/prr_with_sionna_nrv2x.csv", std::ios::out | std::ios::app);
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
  double sensitivity = -93.0;
  double snr_threshold = 10; // Default value
  double sinr_threshold = 10; // Default value
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 200 seconds)

  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  // uint16_t bandwidthBandSl = 400;
  uint16_t bandwidthBandSl = 100; // 10 MHz
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  // uint16_t numerologyBwpSl = 0;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  // uint16_t t2 = 21;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

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

  std::cout << "Start running v2v-simple-cam-exchange-80211p-nrv2x simulation" << std::endl;

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

  uint64_t numberOfNodes_nr = numberOfNodes;

  Ptr<MetricSupervisor> metSup_nr = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_nr(m_baseline_prr);
  metSup_nr = &metSupObj_nr;
  metSup_nr->setTraCIClient(sumoClient);
  // metSup_nr->enablePRRVerboseOnStdout ();

  // Ptr<MultiModelSpectrumChannel> spectrumChannel = nullptr;

  MobilityHelper mobility;

  Time slBearersActivationTime = Seconds (2.0);

  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  NodeContainer nrNodes;
  nrNodes.Create(numberOfNodes_nr);

  mobility.Install (nrNodes);

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});

  // IMPORTANT: Get the SpectrumChannel for NR-V2X, to be used as SpectrumChannel for WiFi 80211.p
  // spectrumChannel = DynamicCast<MultiModelSpectrumChannel>(bandSl.GetBwpAt (0, 0)->m_channel);

  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold1", DoubleValue (sinr_threshold));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold2", DoubleValue (sinr_threshold));

  // nrHelper->SetUeAntennaAttribute ();

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (nrNodes, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);

  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();

  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (nrNodes);
  rxSlUes.Add (nrNodes);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  InternetStackHelper internet;
  internet.Install (nrNodes);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < nrNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = nrNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

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

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID, TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    unsigned long vehID = std::stol(vehicleID.substr (3));
    unsigned long nodeID = vehID - 1;

    Ptr<NetDevice> netDevice;
    Ptr<Socket> sock;
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Node> includedNode = nrNodes.Get(nodeID);
    sock = Socket::CreateSocket (includedNode, tid);
    if (sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
      }
    Ipv4Address groupAddress4 ("225.0.0.0");
    sock->Connect (InetSocketAddress (groupAddress4, 19));

    netDevice = nrNodes.Get(nodeID)->GetDevice(0);
    Ptr<NrUeNetDevice> nrDevice = DynamicCast<NrUeNetDevice>(netDevice);
    nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold1 (sinr_threshold);
    nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold2 (sinr_threshold);
    nrDevice->GetPhy(0)->GetSpectrumPhy ()->GetSpectrumChannel()->SetAttribute ("MaxLossDb", DoubleValue(128.0));

    Ptr<BSContainer> bs_container = CreateObject<BSContainer>(vehID,StationType_passengerCar,sumoClient,false,sock);
    bs_container->addCAMRxCallback (std::bind(&receiveCAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    bs_container->linkMetricSupervisor (metSup_nr);
    bs_container->disablePRRSupervisorForGNBeacons();
    bs_container->setupContainer(true,false,false,false,false,false);
    basicServices.add(bs_container);
    std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
    double desync = ((double)std::rand()/RAND_MAX);
    bs_container->getCABasicService ()->startCamDissemination (desync);

    return nrNodes.Get(nodeID);
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

  outputFile << "\nMetric Supervisor statistics for NR-V2X" << std::endl;
  outputFile << "Average PRR: " << metSup_nr->getAveragePRR_overall () << std::endl;
  outputFile << "Average latency (ms): " << metSup_nr->getAverageLatency_overall () << std::endl;
  outputFile << "RX packet count (from PRR Supervisor): " << metSup_nr->getNumberRx_overall () << std::endl;
  outputFile << "TX packet count (from PRR Supervisor): " << metSup_nr->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_nr->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  savePRRs (metSup_nr, numberOfNodes);

  Simulator::Destroy ();

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  outputFile << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}

