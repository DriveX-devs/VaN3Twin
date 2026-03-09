#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/gn-utils.h"
#include "ns3/btp.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/caBasicService.h"
#include "ns3/denBasicService.h"
#include "ns3/VRUBasicService.h"
#include "ns3/BSContainer.h"
#include "ns3/BSMap.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2p-vam-80211p");

// Global variables, accessible from any function in this file
static int received_CAMs_veh = 0;
static int received_CAMs_ped = 0;
static int received_VAMs_veh = 0;
static int received_VAMs_ped = 0;

BSMap basicServices;
// ******************************************************************************************************************************************************************

void receiveCAM_veh(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType)
{
  received_CAMs_veh++;
}

void receiveVAM_veh(asn1cpp::Seq<VAM> vam, Address from)
{
  received_VAMs_veh++;
}

void receiveCAM_ped(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType)
{
  received_CAMs_ped++;
}

void receiveVAM_ped(asn1cpp::Seq<VAM> vam, Address from)
{
  received_VAMs_ped++;
}

int main (int argc, char *argv[])
{
  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/v2p_map/";
  std::string mob_trace_veh = "vehicles.rou.xml";
  std::string mob_trace_ped = "pedestrians.rou.xml";
  std::string sumo_config = "src/automotive/examples/v2p_map/v2p_map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  int txPower=23;

  std::string csv_name = "VAM_metrics";
  std::string csv_name_cumulative = "VAM_PRR_latency";

  float datarate = 12;
  bool vehicle_vis = false;

  long computationT_acc = 0;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = false;
  bool m_VAM_metrics = true;

  double simTime = 250;

  int numberOfNodes_veh;
  int numberOfNodes_ped;
    
  int up = 0; // User priority

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line options for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace-veh", "Name of the mobility trace file for vehicles", mob_trace_veh);
  cmd.AddValue ("mob-trace-ped", "Name of the mobility trace file for pedestrians", mob_trace_ped);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric Supervisor or not",m_metric_sup);

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
      LogComponentEnable ("v2p-vam-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace_veh the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from vehicles.rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file containing vehicles' routes...");
  std::string path_veh = sumo_folder + mob_trace_veh;

  /* Load the rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path_veh.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path_veh);
    }
  numberOfNodes_veh = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes_veh==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path_veh<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes_veh << " vehicles will be present in the simulation.");

  /*** 0.c Read from the mob_trace_ped the number of pedestrians that will be created.
   *       The number of pedestrians is directly parsed from pedestrians.rou.xml file, looking at all
   *       the valid XML elements of type <person>
  ***/
  NS_LOG_INFO("Reading the .rou file containing pedestrians' routes...");
  std::string path_ped = sumo_folder + mob_trace_ped;

  /* Load the rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path_ped.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path_ped);
    }
  numberOfNodes_ped = XML_rou_count_pedestrians (rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes_ped==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of pedestrians from the specified XML file: "<<path_ped<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes_ped << " pedestrians will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for both vehicles and pedestrians ***/
  NodeContainer Nodes;
  Nodes.Create(numberOfNodes_veh+numberOfNodes_ped);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  NS_LOG_INFO("Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm.");

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
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
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, Nodes);

  // Enable saving to Wireshark PCAP traces
  wifiPhy.EnablePcap ("v2p-vam-80211p", netDevices);

  /*** 4. Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, CreateSocket will end up with a segmentation fault) */
  PacketSocketHelper packetSocket;
  packetSocket.Install (Nodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (Nodes);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
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
    }

  // Creation of the file where VAM metrics will be stored
  std::ofstream csv_VAM_ofstream;
  std::string full_csv_VAM_name = csv_name + ".csv";
  if(m_VAM_metrics){
      if(full_csv_VAM_name != ""){
          csv_VAM_ofstream.open(full_csv_VAM_name, std::ofstream::out | std::ofstream::trunc);
          csv_VAM_ofstream << "station_ID,timestamp,time_elapsed_since_last_gen,triggering_condition,stationID_safe_dist,latitude,longitude,speed" << std::endl;
          csv_VAM_ofstream.close ();
        }
    }

  /*Ptr<WifiNetDevice> wifiAPdevice = DynamicCast<WifiNetDevice> (netDevices.Get(1)); // Getting the PHY of the first (and only, in this case, AP)
  Ptr<WifiPhy> actualPhy = wifiAPdevice->GetPhy ();
  std::cout<<actualPhy->GetTxPowerEnd()<<std::endl;*/

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string stationID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(stationID.substr (3));
      std::string station_type_ID = stationID.substr(0,3);
      StationType_t station_type;

      // Identification of the station type and extraction of the node
      Ptr<Node> includedNode = nullptr;
      if(!station_type_ID.compare("veh")){
          station_type = StationType_passengerCar;
          includedNode = Nodes.Get(nodeID);
      } else{
          station_type = StationType_pedestrian;
          includedNode = Nodes.Get(nodeID-1000+numberOfNodes_veh);
      }

      Ptr<Socket> sock = GeoNet::createGNPacketSocket(includedNode);
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);

      // Create a new Basic Service Container object, which includes, for the current vehicle, both the ETSI CA Basic Service, for the transmission/reception
      // of periodic CAMs, and the ETSI DEN Basic Service, for the transmission/reception of event-based DENMs
      // An ETSI Basic Services container is a wrapper class to enable easy handling of both CAMs and DENMs
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(nodeID,station_type,sumoClient,false,sock);

      // Setup the Metricsupervisor inside the BSContainer, to make each station collect latency and PRR metrics
      bs_container->linkMetricSupervisor(metSup);

      // Setup the file where VAM metrics will be stored
      bs_container->setVAMmetricsfile (full_csv_VAM_name, m_VAM_metrics);

      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received and the function called every time a VAM is received
      if(!station_type_ID.compare("veh")){
        bs_container->addCAMRxCallback (std::bind(&receiveCAM_veh,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
        bs_container->addVAMRxCallback (std::bind(&receiveVAM_veh,std::placeholders::_1,std::placeholders::_2));
      } else{
        bs_container->addCAMRxCallback (std::bind(&receiveCAM_ped,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
        bs_container->addVAMRxCallback (std::bind(&receiveVAM_ped,std::placeholders::_1,std::placeholders::_2));
      }
      // Setup the new ETSI Basic Services container
      // The first parameter should be true is you want to setup a CA Basic Service (for sending/receiving CAMs)
      // The second parameter should be true if you want to setup a DEN Basic Service (for sending/receiving DENMs)
      // The third parameter should be true if you want to setup a VRU Basic Service (for sending/receiving VAMs)
      // The fourth parameter should be true if you want to setup a CP Service (for sending/receiving CPMs)
      // The fifth parameter should be true if you want to enable V2X security
      bs_container->setupContainer(true, false, true, false, false);

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);

      // Start transmitting CAMs (only vehicles) and VAMs (only pedestrians)
      if(!station_type_ID.compare("veh"))
        bs_container->getCABasicService ()->startCamDissemination ();
      else{
          bs_container->getVRUBasicService ()->startAccelerationComputation (computationT_acc);
          bs_container->getVRUBasicService ()->startVamDissemination ();
        }

      return includedNode;
    };

  /* Callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string stationID)
    {
       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-100000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      // Terminate the transmission of CAMs and VAMs
      unsigned long nodeID = std::stol(stationID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(nodeID);
      bsc->cleanup();
    };

  /* Start TraCI client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative != "")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK) != -1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,datarate,num_vehicles,num_pedestrians,avg_PRR_vehicles,"
                              "avg_PRR_pedestrians,avg_PRR_individual_pedestrian,avg_latency_ms_vehicles,"
                              "avg_latency_ms_pedestrians,avg_latency_ms_individual_pedestrian,num_received_CAMs_veh,"
                              "num_received_CAMs_ped,num_received_VAMs_veh,num_received_VAMs_ped" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << datarate << "," << numberOfNodes_veh << "," << numberOfNodes_ped << "," <<
                            metSup->getAveragePRR_overall() << "," <<
                            metSup->getAveragePRR_pedestrian (1000) << "," << metSup->getAverageLatency_overall () <<
                            "," <<  metSup->getAverageLatency_pedestrian (1000) <<
                            "," << received_CAMs_veh << "," << received_CAMs_ped << "," << received_VAMs_veh << "," <<
                            received_VAMs_ped << std::endl;
      }
    }
    
  // Print the number of CAMs received by vehicles and pedestrians
  std::cout << "Number of received CAMs by vehicles =  " << received_CAMs_veh << std::endl;
  std::cout << "Number of received CAMs by pedestrians =  " << received_CAMs_ped << std::endl;

  // Print the number of VAMs received by vehicles and pedestrians
  std::cout << "Number of received VAMs by vehicles =  " << received_VAMs_veh << std::endl;
  std::cout << "Number of received VAMs by pedestrians =  " << received_VAMs_ped << std::endl;

  return 0;
}
