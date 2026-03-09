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
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/
#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/emergencyVehicleAlert-helper.h"
#include "ns3/emergencyVehicleAlert.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-80211p");

int
main (int argc, char *argv[])
{
  /*
   * In this example the generated vehicles will broadcast their CAMs. Every vehicle transmits its messages
   * through a 802.11p interface. There are two types of vehicles: normal "passenger" cars and emergency vehicles.
   * When a normal vehicle receives a CAM from an emergency vehicle it performs the following checks:
   * 1) It checks if the distance between itself and the emergency vehicle is under a certain threshold
   * 2) It checks if the difference in the heading angle between itself and the emergency vehicle is under a certain threshold
   * In case the two conditions are met, then the receiving vehicle will take some actions to facilitate the takover
   * maneuver (like slow down). When the emergency vehicle is far away, the normal vehicle will resume its old speed
   */

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  int txPower=30;
  double penetrationRate = 0.7;

  float datarate=12;  
  bool vehicle_vis = false;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = false;
  bool m_security = false;

  double simTime = 100;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("enable-security","Enable security header inside Geonet or not",m_security);
  cmd.AddValue ("penetrationRate", "Rate of vehicles equipped with wireless communication devices", penetrationRate);

  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2v-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
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

  /*** 1. Create containers for OBUs ***/
  NodeContainer obuNodes;
  obuNodes.Create(numberOfNodes);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  std::cout << "Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm." << std::endl;

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  /*Example of using diffrent propagation model (after applying extension with apply-extension.sh in automotive/propagation-extended/)*/
  // YansWifiChannelHelper wifiChannel;
  // wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // wifiChannel.AddPropagationLoss ("ns3::CniUrbanmicrocellPropagationLossModel");
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  /* To be removed when BPT is implemented */
  //Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue (Seconds (1)));

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  std::cout << "Datarate: " << datarate_config << std::endl;
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (datarate_config),
                                      "ControlMode", StringValue (datarate_config),
                                      "NonUnicastMode",StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);

  wifiPhy.EnablePcap ("v2v-EVA",netDevices);

  /*** 4. Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, CreateSocket will end up with a segmentation fault */
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetrationRate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
      // Uncomment this to enable verbose output from the PRRsupervisor, printing latency and PRR for each single packet
      // metSup->enablePRRVerboseOnStdout();
    }

  /*** 7. Setup interface and application for dynamic nodes ***/
  emergencyVehicleAlertHelper EmergencyVehicleAlertHelper;
  EmergencyVehicleAlertHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleAlertHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleAlertHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleAlertHelper.SetAttribute ("Model", StringValue ("80211p"));
  EmergencyVehicleAlertHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleAlertHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  EmergencyVehicleAlertHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  EmergencyVehicleAlertHelper.SetAttribute ("EnableSecurity", BooleanValue (m_security));

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      ApplicationContainer AppSample = EmergencyVehicleAlertHelper.Install (includedNode);

      AppSample.Start (Seconds (0.0));
      AppSample.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* Callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* stop all applications */
      Ptr<emergencyVehicleAlert> appSample_ = exNode->GetApplication(0)->GetObject<emergencyVehicleAlert>();

      if(appSample_)
        appSample_->StopApplicationNow();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start TraCI client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;

      for(int i=1;i<numberOfNodes+1;i++) {
          std::cout << "Average latency of vehicle " << i << " (ms): " << metSup->getAverageLatency_vehicle (i) << std::endl;
          std::cout << "Average PRR of vehicle " << i << " (%): " << metSup->getAveragePRR_vehicle (i) << std::endl;
          std::cout << "Total number of TX packets by vehicle " << i << ": " << metSup->getNumberTx_vehicle(i) << std::endl;
          std::cout << "Total number of RX packets by vehicle " << i << ": " << metSup->getNumberRx_vehicle(i) << std::endl;
      }

      std::cout << "Average latency of CAMs (ms): " << metSup->getAverageLatency_messagetype (MetricSupervisor::messageType_cam) << std::endl;
      std::cout << "Average PRR of CAMs (%): " << metSup->getAveragePRR_messagetype (MetricSupervisor::messageType_cam) << std::endl;
      std::cout << "Total number of CAMs transmitted: " << metSup->getNumberTx_messagetype (MetricSupervisor::messageType_cam) << std::endl;
      std::cout << "Total number of CAMs received: " << metSup->getNumberRx_messagetype (MetricSupervisor::messageType_cam) << std::endl;

      std::cout << "Average latency of DENMSs (ms): " << metSup->getAverageLatency_messagetype (MetricSupervisor::messageType_denm) << std::endl;
      std::cout << "Average PRR of DENMSs (%): " << metSup->getAveragePRR_messagetype (MetricSupervisor::messageType_denm) << std::endl;
      std::cout << "Total number of DENMs transmitted: " << metSup->getNumberTx_messagetype (MetricSupervisor::messageType_denm) << std::endl;
      std::cout << "Total number of DENMs received: " << metSup->getNumberRx_messagetype (MetricSupervisor::messageType_denm) << std::endl;

      std::cout << "Average latency of CPMs (ms): " << metSup->getAverageLatency_messagetype (MetricSupervisor::messageType_cpm) << std::endl;
      std::cout << "Average PRR of CPMs (%): " << metSup->getAveragePRR_messagetype (MetricSupervisor::messageType_cpm) << std::endl;
      std::cout << "Total number of CPMs transmitted: " << metSup->getNumberTx_messagetype (MetricSupervisor::messageType_cpm) << std::endl;
      std::cout << "Total number of CPMs received: " << metSup->getNumberRx_messagetype (MetricSupervisor::messageType_cpm) << std::endl;

      std::cout << "Average latency of GN Beacons (ms): " << metSup->getAverageLatency_messagetype (MetricSupervisor::messageType_GNbeacon) << std::endl;
      std::cout << "Average PRR of GN Beacons (%): " << metSup->getAveragePRR_messagetype (MetricSupervisor::messageType_GNbeacon) << std::endl;
      std::cout << "Total number of GN Beacons transmitted: " << metSup->getNumberTx_messagetype (MetricSupervisor::messageType_GNbeacon) << std::endl;
      std::cout << "Total number of GN Beacons received: " << metSup->getNumberRx_messagetype (MetricSupervisor::messageType_GNbeacon) << std::endl;

      std::cout << "Total number of TX packets: " << metSup->getNumberTx_overall() << std::endl;
      std::cout << "Total number of RX packets: " << metSup->getNumberRx_overall() << std::endl;
    }

  return 0;
}
