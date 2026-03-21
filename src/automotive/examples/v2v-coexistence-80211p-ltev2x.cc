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
#include <fstream>

// LTE-V2X
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
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/cv2x-module.h"
#include <ns3/node-list.h>
#include <unistd.h>
#include "ns3/core-module.h"

#include "ns3/txTracker.h"

#include "ns3/sionna-helper.h"

#include <chrono>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchange80211pLtev2x");

enum class NodeType
{
  DSRC,
  LTE,
  INTERFERING
};

std::vector<uint8_t> dsrcVehicles;
std::vector<uint8_t > ltev2xVehicles;
static int packet_count = 0;
static int counter = 0;
BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles
bool phy_collection = true;
bool use_sionna = false;

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

void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationId_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  if (!phy_collection)
    {
      return;
    }

  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(sinr) && !std::isnan(snr))
    {
      sinr = snr;
    }

  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  libsumo::TraCIPosition pos_lat_lon = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;

  libsumo::TraCIPosition pos_sender = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(cam->header.stationId));

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos_lat_lon.y, pos_lat_lon.x);

  ulong time = Simulator::Now().GetMicroSeconds();
  uint8_t los;
  Vector a_position = {pos.x, pos.y, pos.z};
  Vector b_position = {pos_sender.x, pos_sender.y, 0};
  if (use_sionna)
  {
    std::string los_str = getLOSStatusFromSionna(a_position, b_position);
    if (los_str == "[False]")
      {
        los = 0;
      }
    else
      {
        los = 1;
      }
  }
  else
  {
    los = 0; // Default, with ns-3 models we should check if the model used provides LOS/NLOS state and its current value
  }
  
  std::ifstream camFileHeader("src/sinr_ni.csv");
  std::ofstream camFile;
  camFile.open("src/sinr_ni.csv", std::ios::out | std::ios::app);
  if (!camFileHeader.is_open())
    {
      if (camFile.is_open())
      {
        camFile << "time,rx,tx,rx_lat,rx_lon,tx_lat,tx_lon,technology,distance,los,sinr" << std::endl;
      } 
      else
      {
        std::cerr << "Unable to create file .csv" << std::endl;
      }
    }

  std::string technology;
  if (std::find(dsrcVehicles.begin(), dsrcVehicles.end(), my_stationID) != dsrcVehicles.end())
    {
      technology = "DSRC";
    }
  else
    {
      technology = "NR-V2X";
    }
  
  camFile << time << "," << my_stationID << "," << std::to_string(cam->header.stationId) << "," << pos_lat_lon.y << "," << pos_lat_lon.x << "," << lat_sender << "," << lon_sender << "," << technology << "," << distance << "," << std::to_string(los) << "," << sinr << std::endl;
  camFile.close();
}

void receiveCPM(asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from, StationId_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  if (!phy_collection)
    {
      return;
    }

  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(sinr) && !std::isnan(snr))
    {
      sinr = snr;
    }

  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  libsumo::TraCIPosition pos_lat_lon = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.longitude,double)/1e7;

  libsumo::TraCIPosition pos_sender = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(cpm->header.stationId));

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos_lat_lon.y, pos_lat_lon.x);

  ulong time = Simulator::Now().GetMicroSeconds();
  uint8_t los;
  Vector a_position = {pos.x, pos.y, pos.z};
  Vector b_position = {pos_sender.x, pos_sender.y, pos_sender.z};
  if (use_sionna)
  {
    std::string los_str = getLOSStatusFromSionna(a_position, b_position);
    if (los_str == "[False]")
      {
        los = 0;
      }
    else
      {
        los = 1;
      }
  }
  else
  {
    los = 0; // Default, with ns-3 models we should check if the model used provides LOS/NLOS state and its current value
  }
  std::ifstream cpmFileHeader("src/sinr_ni.csv");
  std::ofstream cpmFile;
  cpmFile.open("src/sinr_ni.csv", std::ios::out | std::ios::app);
  if (!cpmFileHeader.is_open())
    {
      if (cpmFile.is_open())
        {
          cpmFile << "time,rx,tx,rx_lat,rx_lon,tx_lat,tx_lon,technology,distance,los,sinr" << std::endl;
        }
      else
        {
          std::cerr << "Unable to create file .csv" << std::endl;
        }
    }

  std::string technology;
  if (std::find(dsrcVehicles.begin(), dsrcVehicles.end(), my_stationID) != dsrcVehicles.end())
    {
      technology = "DSRC";
    }
  else
    {
      technology = "LTE-V2X";
    }

  cpmFile << time << "," << my_stationID << "," << std::to_string(cpm->header.stationId) << "," << pos_lat_lon.y << "," << pos_lat_lon.x << "," << lat_sender << "," << lon_sender << "," << technology << "," << distance << "," << std::to_string(los) << "," << sinr << std::endl;
  cpmFile.close();
}

void savePRRs(Ptr<MetricSupervisor> metSup, std::vector<std::string> nodes, std::string type)
{
  std::ofstream file;
  if (type == "lte")
    {
      file.open("src/prr_latency_ns3_coexistence_ltev2x.csv", std::ios::out | std::ios::app);
    }
  else if (type == "11p")
    {
      file.open("src/prr_latency_ns3_coexistence_11p.csv", std::ios::out | std::ios::app);
    }
  file << "node_id,prr,latency(ms)" << std::endl;
  for (int i = 0; i < nodes.size(); i++)
    {
      double prr = metSup->getAveragePRR_vehicle (std::stol(nodes[i].substr (3)));
      double latency = metSup->getAverageLatency_vehicle (std::stol(nodes[i].substr (3)));
      file << i << "," << prr << "," << latency << std::endl;
    }
  file.close();
}

void txTrackerSetup(std::vector<std::string> wifiVehicles, NodeContainer wifiNodes, std::vector<std::string> lteVehicles, NetDeviceContainer nrDevices, bool dsrc_interference, bool lte_interference)
{
  auto& tracker = TxTracker::GetInstance();

  std::vector<std::tuple<std::string, uint8_t, Ptr<WifiNetDevice>>> wifiVehiclesList;
  std::vector<std::tuple<std::string, uint8_t, Ptr<NrUeNetDevice>>> lteVehiclesList;

  uint8_t i = 1 ? dsrc_interference : 0; // Start from 1 because the first node is the interfering one
  for (auto v : wifiVehicles)
    {
      uint8_t id = wifiNodes.Get(i)->GetId();
      Ptr<WifiNetDevice> netDevice = DynamicCast<WifiNetDevice>(wifiNodes.Get(i)->GetDevice(0));
      wifiVehiclesList.push_back (std::make_tuple (v, id, netDevice));
      i++;
    }
  tracker.Insert11pNodes (wifiVehiclesList);

  i = 1 ? lte_interference : 0;
  for (auto v : lteVehicles)
    {
      Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice>(nrDevices.Get(i));
      uint8_t id = nrDevices.Get (i)->GetNode()->GetId();
      lteVehiclesList.push_back (std::make_tuple (v, id, netDevice));
      i++;
    }
  tracker.InsertNrNodes (lteVehiclesList);
}

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, uint8_t vehicles, Ptr<TraciClient> traci)
{
  uint8_t present = traci->GetVehicleMapSize();
  if (present != vehicles)
    {
      Simulator::Schedule(pktInterval, &GenerateTraffic_interfering, socket, pktSize, pktCount, pktInterval, vehicles, traci);
      return;
    }
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zero
      // Populate the packet with random data
      std::string data = "";
      uint16_t packetSize = 1000;
      for (int i = 0; i < packetSize; i++)
        {
          data += std::to_string (counter);
          data += ",";
          long long now = Simulator::Now().GetMicroSeconds();
          data += std::to_string (now);
          data += ",";
        }
      Ptr<Packet> packet = Create<Packet>((uint8_t*) data.c_str(), packetSize);
      if (socket->Send (packet) != -1)
        {
          // std::cout << "Interfering packet sent" << std::endl;
          counter ++;
        }
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering, socket, pktSize, pktCount, pktInterval, vehicles, traci);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  phy_collection = true;

  // std::string phyMode ("OfdmRate6MbpsBW10MHz");
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  double bandwidth_11p = 10;
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
  double simTime = 50.0; // Total simulation time (default: 200 seconds)

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
  double centralFrequencyBandSl = 5.89e9;
  bool hardcoded_bw = true;
  bool m_metric_sup = true;

  // (T2-T1+1) x (1/(2^numerology)) < reservation period
  // (81-2+1) x (1/2^2) < 20

  bool sionna = false;
  std::string server_ip = "";
  bool local_machine = false;
  bool verb = false;

  bool interference = false;
  bool dsrc_interference = false;
  bool lte_interference = false;

  Time slBearersActivationTime = Seconds (2.0);

  if ((dsrc_interference && lte_interference) /*|| (interference && !(dsrc_interference || nr_interference))*/)
    {
      NS_FATAL_ERROR ("Check the interference setup.");
    }

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
  
  use_sionna = sionna;
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
  /*if (interference)
    {
      numberOfNodes --;
    }*/
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

  bool odd = false;
  if (numberOfNodes%2 != 0)
    {
      odd = true;
    }

  std::vector<std::string> wifiVehicles;
  std::vector<std::string> lteVehicles;

  uint8_t i = 1;
  if (dsrc_interference || lte_interference)
    {
      i = 2;
    }

  for (; i <= numberOfNodes; i++)
    {
      if (i%2 == 0)
        {
          wifiVehicles.push_back ("veh" + std::to_string (i));
          dsrcVehicles.push_back (i);
        }
      else
        {
          lteVehicles.push_back("veh" + std::to_string (i));
          ltev2xVehicles.push_back (i);
        }
    }

  uint64_t numberOfNodes_11p = wifiVehicles.size();
  uint64_t numberOfNodes_lte = lteVehicles.size();

  if (dsrc_interference)
    {
      numberOfNodes_11p ++;
    }
  else if (lte_interference)
    {
      numberOfNodes_lte ++;
    }

  Ptr<MetricSupervisor> metSup_11p = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_11p(m_baseline_prr);
  metSup_11p = &metSupObj_11p;
  metSup_11p->setTraCIClient(sumoClient);
  // This function enables printing the current and average latency and PRR for each received packet
  // metSup_11p->enablePRRVerboseOnStdout ();
  metSup_11p->setChannelTechnology("80211p");
  metSup_11p->setCBRWindowValue(200);
  metSup_11p->setCBRAlphaValue(0.1);
  metSup_11p->setSimulationTimeValue(simTime);

  Ptr<MetricSupervisor> metSup_lte = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_nr(m_baseline_prr);
  metSup_lte = &metSupObj_nr;
  metSup_lte->setTraCIClient(sumoClient);
  // metSup_nr->enablePRRVerboseOnStdout ();

  MobilityHelper mobility;

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
  ueNodes.Create(numberOfNodes_lte);

  /*** 4. Create and install mobility (SUMO will be attached later) ***/
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
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueLteDevs, numberOfNodes_lte);

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

  // metSup_11p->setNodeContainer(wifiNodes);
  // metSup_11p->startCheckCBR();

  // Enable saving to Wireshark PCAP traces
  // wifiPhy.EnablePcap ("v2v-80211p-student-application", devices);

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  mobility.Install (wifiNodes);

  PacketSocketHelper packetSocket;
  packetSocket.Install(wifiNodes);

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
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  if (interference)
  {
    std::cout << "Interference mode enabled" << std::endl;
    auto& tracker = TxTracker::GetInstance();
    tracker.SetCentralFrequencies(centralFrequencyBandSl, centralFrequencyBandSl, centralFrequencyBandSl);
    tracker.SetBandwidths(bandwidth_11p * 1e6, slBandwidth * 1e6, 0.0);
    txTrackerSetup(wifiVehicles, wifiNodes, lteVehicles, ueLteDevs, dsrc_interference, lte_interference);
  }

  uint8_t node11pCounter = 0;
  if (dsrc_interference)
    {
      node11pCounter = 1;
    }

  uint8_t nodeNrCounter = 0;
  if (lte_interference)
    {
      nodeNrCounter = 1;
    }

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID, TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    NodeType type;
    unsigned long vehID = std::stol(vehicleID.substr (3));
    unsigned long nodeID;

    std::cout << "Vehicle entering in the simulation: " << vehicleID << " at time " << Simulator::Now().GetMicroSeconds() << std::endl;

    if (std::find(wifiVehicles.begin(), wifiVehicles.end(), vehicleID) != wifiVehicles.end())
      {
        type = NodeType::DSRC;
        nodeID = node11pCounter;
        node11pCounter++;
      }
    else if (std::find(lteVehicles.begin(), lteVehicles.end(), vehicleID) != lteVehicles.end())
      {
        type = NodeType::LTE;
        nodeID = nodeNrCounter;
        nodeNrCounter++;
      }
    else
      {
        type = NodeType::INTERFERING;
      }

    Ptr<NetDevice> netDevice;
    Ptr<Socket> sock;
    Ptr<WifiNetDevice> wifiDevice;
    Ptr<Node> includedNode;
    Ipv4Address groupAddress4 ("225.0.0.0");
    Ptr<NrUeNetDevice> nrDevice;
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    std::ofstream file;
    switch (type)
      {
      case NodeType::DSRC:
        sock = GeoNet::createGNPacketSocket (wifiNodes.Get (nodeID));
        netDevice = wifiNodes.Get (nodeID)->GetDevice (0);
        wifiDevice = DynamicCast<WifiNetDevice> (netDevice);
        wifiDevice->GetPhy ()->SetRxSensitivity (sensitivity);
        metSup_lte->addExcludedID (vehID);
        break;
      case NodeType::LTE:
        includedNode =ueNodes.Get (nodeID);
        sock = Socket::CreateSocket (includedNode, tid);
        if (sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
          {
            NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
          }
        sock->Connect (InetSocketAddress (groupAddress4, 19));
        netDevice = ueNodes.Get (nodeID)->GetDevice (0);
        nrDevice = DynamicCast<> (netDevice);
        nrDevice->GetPhy(0)->GetSpectrumPhy ()->GetSpectrumChannel()->SetAttribute ("MaxLossDb", DoubleValue(120.0));
        // nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold1 (snr_threshold);
        // nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold2 (snr_threshold);
        metSup_11p->addExcludedID (vehID);
        break;
      case NodeType::INTERFERING:
        metSup_11p->addExcludedID (vehID);
        metSup_lte->addExcludedID (vehID);
        if (dsrc_interference)
          {
            Ptr<Socket> socket = Socket::CreateSocket (wifiNodes.Get (0), TypeId::LookupByName ("ns3::PacketSocketFactory"));
            PacketSocketAddress local_source_interfering;
            local_source_interfering.SetSingleDevice (wifiNodes.Get (0)->GetDevice(0)->GetIfIndex ());
            local_source_interfering.SetPhysicalAddress (wifiNodes.Get (0)->GetDevice(0)->GetAddress());
            local_source_interfering.SetProtocol (0x88B5);
            if (socket->Bind (local_source_interfering) == -1)
              {
                NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
              }
            PacketSocketAddress remote_source_interfering;
            remote_source_interfering.SetSingleDevice (wifiNodes.Get (0)->GetDevice(0)->GetIfIndex());
            remote_source_interfering.SetPhysicalAddress (wifiNodes.Get (0)->GetDevice(0)->GetBroadcast());
            remote_source_interfering.SetProtocol (0x88B5);
            socket->Connect (remote_source_interfering);
            socket->SetPriority (0);
            Simulator::ScheduleWithContext (0,
                                            Seconds (1.0), &GenerateTraffic_interfering,
                                            socket, 1000, simTime*2000, MilliSeconds (5),
                                            numberOfNodes, sumoClient);
          }
        else if (lte_interference)
          {
            // Da testare
            Ptr<Socket> socket = Socket::CreateSocket (ueNodes.Get (0), tid);
            if (socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
              {
                NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
              }
            socket->Connect (InetSocketAddress (groupAddress4, 19));
            socket->SetPriority (0);
            Simulator::ScheduleWithContext (0,
                                            Seconds (3.0), &GenerateTraffic_interfering,
                                            socket, 1000, simTime*2000, MilliSeconds (5),
                                            numberOfNodes, sumoClient);
          }
        break;
      default:
        NS_FATAL_ERROR ("No technology recognized.");
      }

    if (type != NodeType::INTERFERING)
      {
        Ptr<BSContainer> bs_container = CreateObject<BSContainer>(vehID,StationType_passengerCar,sumoClient,false,sock);
        bs_container->addCAMRxCallback (std::bind(&receiveCAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
        bs_container->addCPMRxCallback (std::bind(&receiveCPM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
        switch (type)
          {
          case NodeType::DSRC:
            bs_container->linkMetricSupervisor (metSup_11p);
            break;
          case NodeType::LTE:
            bs_container->linkMetricSupervisor (metSup_lte);
            break;
          }
        bs_container->disablePRRSupervisorForGNBeacons();
        bs_container->setupContainer(true,false,false,true,false);
        basicServices.add(bs_container);
        std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
        bs_container->getCABasicService ()->startCamDissemination ();
        bs_container->getCPBasicService ()->startCpmDissemination ();
      }

    switch (type)
      {
      case NodeType::DSRC:
        return wifiNodes.Get(nodeID);
      case NodeType::LTE:
        return ueNodes.Get(nodeID);
      case NodeType::INTERFERING:
        return wifiNodes.Get(0);
      }
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

  std::cout << "\nTotal number of packets received: " << packet_count << std::endl;

  std::cout << "\nMetric Supervisor statistics for 802.11p" << std::endl;
  std::cout << "Average PRR: " << metSup_11p->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup_11p->getAverageLatency_overall () << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup_11p->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup_11p->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_11p->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  std::cout << "\nMetric Supervisor statistics for LTE-V2X" << std::endl;
  std::cout << "Average PRR: " << metSup_lte->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup_lte->getAverageLatency_overall () << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup_lte->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup_lte->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_nr->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  Simulator::Destroy ();

  savePRRs(metSup_11p, wifiVehicles, "11p");
  savePRRs(metSup_lte, lteVehicles, "lte");

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  std::cout << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}

