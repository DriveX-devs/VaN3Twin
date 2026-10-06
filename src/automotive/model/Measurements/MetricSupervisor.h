#ifndef METRICSUPERVISOR_H
#define METRICSUPERVISOR_H

#include "ns3/OpenCDAClient.h"
#include <list>
#include <unordered_map>
#include <string>
#include "ns3/traci-client.h"
#include "ns3/event-id.h"
#include "ns3/wifi-phy-state.h"
#include "ns3/gps-tc-module.h"

namespace ns3 {

/**
 * \ingroup automotive
 *
 * \brief This class is used to compute some metrics such as the Packet Reception Ratio (PRR), Channel Busy Ratio (CBR, and One-way latency of messages in the simulation.
 *
 * This class provides capabilities for computing PRR and Latency values for:
 * - All the messages sent and received in the simulation
 * - All the messages sent and received by a specific vehicle
 * - All the messages of a specific type (i.e CAM, DENM, CPM, IVIM or VAM) sent and received in the simulation
 *
 * This class provides capabilities for computing the CBR for all the nodes in the simulation.
 */
class MetricSupervisor : public Object {

  typedef struct baselineVehicleData {
    std::list<uint64_t> nodeList;
    int x;
  } baselineVehicleData_t;

public:

  typedef enum messageType {
    messageType_unsupported = 0,
    messageType_denm	= 1,
    messageType_cam	= 2,
    messageType_poi	= 3,
    messageType_spatem	= 4,
    messageType_mapem	= 5,
    messageType_ivim	= 6,
    messageType_ev_rsr	= 7,
    messageType_tistpgtransaction	= 8,
    messageType_srem	= 9,
    messageType_ssem	= 10,
    messageType_evcsn	= 11,
    messageType_saem	= 12,
    messageType_rtcmem	= 13,
    messageType_cpm     = 14,
    messageType_imzm    = 15,
    messageType_vam     = 16,
    messageType_dsm     = 17,
    messageType_pcim    = 18,
    messageType_pcvm    = 19,
    messageType_mcm     = 20,
    messageType_pam     = 21,
    messageType_cem    = 200,
    messageType_GNbeacon = 1000
  } messageType_e;

  static TypeId GetTypeId();
  /**
   * \brief Default constructor
   *
   */
  MetricSupervisor() {}
  /**
   * \brief Constructor
   *
   * This constructor initializes the PRRSupervisor object.
   * @param baseline_m The baseline distance in meters to consider for a packet to be received.
   */
  MetricSupervisor(int baseline_m) : m_baseline_m(baseline_m) {m_cbr_csv_log_file="";}
  virtual ~MetricSupervisor();

  static std::string bufToString(uint8_t *buf, uint32_t bufsize);

  /**
   * @brief Set the TraCI client pointer.
   * @param traci_ptr
   */
  void setTraCIClient(Ptr<TraciClient> traci_ptr) {m_traci_ptr = traci_ptr;}

  /**
   * @brief Set the Gps TC client pointer.
   * @param gps_tc
   */
  void setGPSTCClient(std::vector<GPSTraceClient *> v_gps_tc) {m_v_gpstc = v_gps_tc;}

  /**
   * @brief Set the OpenCDA client pointer.
   * @param traci_ptr
   */
  void setOpenCDACLient(Ptr<OpenCDAClient> carla_ptr) {m_carla_ptr = carla_ptr;}

  /**
   * @brief This function is called everytime a packet is sent in the simulation by the GeoNet object. It is not expected to be called by the user.
   * @param buf  The buffer containing the packet.
   * @param lat   The latitude of the sender.
   * @param lon   The longitude of the sender.
   * @param vehicleID  The ID of the sender.
   * @param messagetype  The ETSI type of the message.
   */
  void signalSentPacket(std::string buf,double lat,double lon,uint64_t vehicleID, messageType_e messagetype);
  /**
   * @brief This function is called everytime a packet is received in the simulation by the GeoNet object. It is not expected to be called by the user.
   * @param buf  The buffer containing the packet.
   * @param vehicleID  The ID of the receiver.
   */
  void signalReceivedPacket(std::string buf,uint64_t vehicleID);

  /**
   * @brief Get the average PRR for all the messages sent and received in the simulation.
   * @return  The average PRR.
   */
  double getAveragePRR_overall(void) {return m_avg_PRR;}
  /**
   * @brief Get the average latency for all the messages sent and received in the simulation.
   * @return  The average latency [ms]
   */
  double getAverageLatency_overall(void) {return m_avg_latency_ms;}

  /**
   * @brief Get the average SINR for all the messages received in the simulation.
   * @return  The average SINR [dB]

  double getAverageSINR_overall(void) {
    double sum = 0.0;
    for (auto item : m_sinr_per_veh)
      {
        sum += getAverageSINR_vehicle (item.first);
      }
    return sum / m_sinr_per_veh.size();
  }
  */

  /**
   * @brief Get the total number of packets transmitted in the whole simulation.
   * @return  The total number of packets transmitted.
   */
  uint64_t getNumberTx_overall(void) {return m_total_tx;}
  /**
   * @brief Get the average number of transmitted packets over all road users.
   * @return  The average number of transmitted packets.
   */
  double getAverageNumberTx_overall(void) {
    uint64_t totalTx = 0;
    uint64_t totalUsers = 0;

    // Sum up the total number of transmitted packets for each vehicle
    for (const auto &pair : m_ntx_per_veh) {
        totalTx += pair.second;
    }

    // Sum up the total number of transmitted packets for each pedestrian
    for (const auto &pair : m_ntx_per_ped) {
        totalTx += pair.second;
    }

    // Sum up the total number of transmitted packets for each RSU
    for (const auto &pair : m_ntx_per_rsu) {
        totalTx += pair.second;
    }

    // Calculate the total number of road users
    totalUsers = m_ntx_per_veh.size() + m_ntx_per_ped.size() + m_ntx_per_rsu.size();

    // Compute the average number of transmitted packets over all road users
    double averageTx = totalUsers > 0 ? static_cast<double>(totalTx) / static_cast<double>(totalUsers) : 0.0;

    return averageTx;
  }
  /**
   * @brief Get the total number of packets received in the whole simulation.
   * @return  The total number of packets received.
   */
  uint64_t getNumberRx_overall(void) {return m_total_rx;}
  /**
   * @brief Get the average number of received packets over all road users.
   * @return  The average number of received packets.
   */
  double getAverageNumberRx_overall(void) {
    uint64_t totalRx = 0;
    uint64_t totalUsers = 0;

    // Sum up the total number of received packets for each vehicle
    for (const auto &pair : m_nrx_per_veh) {
        totalRx += pair.second;
    }

    // Sum up the total number of received packets for each pedestrian
    for (const auto &pair : m_nrx_per_ped) {
        totalRx += pair.second;
    }

    // Sum up the total number of received packets for each RSU
    for (const auto &pair : m_nrx_per_rsu) {
        totalRx += pair.second;
    }

    // Calculate the total number of road users
    totalUsers = m_nrx_per_veh.size() + m_nrx_per_ped.size() + m_nrx_per_rsu.size();

    // Compute the average number of received packets over all road users
    double averageRx = totalUsers > 0 ? static_cast<double>(totalRx) / static_cast<double>(totalUsers) : 0.0;

    return averageRx;
  }

  /**
   * @brief Get the average number of vehicles within the PRR baseline for all road users.
   * @return  The average number of vehicles within the baseline.
   */
  double getAverageNumberOfVehiclesInBaseline_overall() {
    double totalAverageVehiclesInBaseline = 0;
    uint64_t totalUsers = 0;

    // Sum up the average number of vehicles within the baseline for each vehicle
    for (const auto &pair : m_avg_nvehbsln_per_veh) {
        totalAverageVehiclesInBaseline += pair.second;
    }

    // Sum up the average number of vehicles within the baseline for each pedestrian
    for (const auto &pair : m_avg_nvehbsln_per_ped) {
        totalAverageVehiclesInBaseline += pair.second;
    }

    // Sum up the average number of vehicles within the baseline for each RSU
    for (const auto &pair : m_avg_nvehbsln_per_rsu) {
        totalAverageVehiclesInBaseline += pair.second;
    }

    // Calculate the total number of samples
    totalUsers = m_avg_nvehbsln_per_veh.size() + m_avg_nvehbsln_per_ped.size() + m_avg_nvehbsln_per_rsu.size();

    // Compute the average number of vehicles within the baseline during the whole simulation
    double averageVehiclesInBaseline = totalUsers > 0 ? static_cast<double>(totalAverageVehiclesInBaseline) / static_cast<double>(totalUsers) : 0.0;

    return averageVehiclesInBaseline;
  }

  /**
   * @brief Get the average PRR for all the messages sent and received by a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The average PRR.
   */
  double getAveragePRR_vehicle(uint64_t vehicleID) {return m_avg_PRR_per_veh[vehicleID];}
  /**
   * @brief Get the average latency for all the messages sent and received by a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The average latency [ms]
   */
  /**
   * @brief Get the average SINR for all the messages sent and received by a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The average SINR [dB]

  double getAverageSINR_vehicle(uint64_t vehicleID) {
    double sum = 0.0;
    for (auto sinr : m_sinr_per_veh[vehicleID])
      {
        sum += sinr;
      }
    return sum / m_sinr_per_veh[vehicleID].size();
  }
  */
  double getAverageLatency_vehicle(uint64_t vehicleID) {return m_avg_latency_ms_per_veh[vehicleID];}
  /**
   * @brief Get the total number of packets transmitted by a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The total number of packets transmitted.
   */
  uint64_t getNumberTx_vehicle(uint64_t vehicleID) {return m_ntx_per_veh[vehicleID];}
  /**
   * @brief Get the total number of packets received by a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The total number of packets received.
   */
  uint64_t getNumberRx_vehicle(uint64_t vehicleID) {return m_nrx_per_veh[vehicleID];}

  /**
   * @brief Get the average number of vehicles within the PRR baseline for a specific vehicle.
   * @param vehicleID  The ID of the vehicle.
   * @return  The average number of vehicles within the baseline.
   */
  double getAverageNumberOfVehiclesInBaseline_vehicle(uint64_t vehicleID) {return m_avg_nvehbsln_per_veh[vehicleID];}

  /**
   * @brief Get the average PRR for all the messages sent and received by a specific VRU/pedestrian.
   * @param rsuID  The ID of the VRU.
   * @return  The average PRR.
   */
  double getAveragePRR_pedestrian(uint64_t pedestrianID) {return m_avg_PRR_per_ped[pedestrianID];}
  /**
   * @brief Get the average latency for all the messages sent and received by a specific VRU/pedestrian.
   * @param pedestrianID  The ID of the VRU.
   * @return  The average latency [ms]
   */
  double getAverageLatency_pedestrian(uint64_t pedestrianID) {return m_avg_latency_ms_per_ped[pedestrianID];}
  /**
   * @brief Get the total number of packets transmitted by a specific VRU/pedestrian.
   * @param pedestrianID  The ID of the VRU.
   * @return  The total number of packets transmitted.
   */
  uint64_t getNumberTx_pedestrian(uint64_t pedestrianID) {return m_ntx_per_ped[pedestrianID];}
  /**
   * @brief Get the total number of packets received by a specific VRU/pedestrian.
   * @param pedestrianID  The ID of the VRU.
   * @return  The total number of packets received.
   */
  uint64_t getNumberRx_pedestrian(uint64_t pedestrianID) {return m_nrx_per_ped[pedestrianID];}
  /**
   * @brief Get the average number of vehicles within the PRR baseline for a specific VRU/pedestrian.
   * @param pedestrianID  The ID of the VRU.
   * @return  The average number of vehicles within the baseline.
   */
  double getAverageNumberOfVehiclesInBaseline_pedestrian(uint64_t pedestrianID) {return m_avg_nvehbsln_per_ped[pedestrianID];}

  /**
   * @brief Get the average PRR for all the messages of a specific type sent and received in the simulation.
   * @param messagetype  The ETSI type of the message.
   * @return  The average PRR.
   */
  double getAveragePRR_messagetype(messageType_e messagetype) {return m_avg_PRR_per_messagetype[messagetype];}
  /**
   * @brief Get the average latency for all the messages of a specific type sent and received in the simulation.
   * @param messagetype  The ETSI type of the message.
   * @return  The average latency [ms]
   */
  double getAverageLatency_messagetype(messageType_e messagetype) {return m_avg_latency_ms_per_messagetype[messagetype];}
  /**
   * @brief Get the total number of packets transmitted given a specific message type.
   * @param messagetype  The ETSI message type.
   * @return  The total number of packets transmitted.
   */
  uint64_t getNumberTx_messagetype(messageType_e messagetype) {return m_ntx_per_messagetype[messagetype];}
  /**
   * @brief Get the total number of packets received given a specific message type.
   * @param messagetype  The ETSI message type.
   * @return  The total number of packets received.
   */
  uint64_t getNumberRx_messagetype(messageType_e messagetype) {return m_nrx_per_messagetype[messagetype];}
  /**
   * @brief Get the average number of vehicles within the PRR baseline for a specific message type.
   * @param messagetype  The ETSI message type.
   * @return  The average number of vehicles within the baseline.
   */
  double getAverageNumberOfVehiclesInBaseline_messagetype(messageType_e messagetype) {return m_avg_nvehbsln_per_messagetype[messagetype];}

  /**
   * @brief Get the total number of packets transmitted by a specific RSU.
   * @param rsuID  The ID of the RSU.
   * @return  The total number of packets transmitted by that specific RSU.
   */
  uint64_t getNumberTx_rsu(uint64_t rsuID) {return m_ntx_per_rsu[rsuID];}
  /**
   * @brief Get the total number of packets received by a specific RSU.
   * @param rsuID  The ID of the RSU.
   * @return  The total number of packets received by that specific RSU.
   */
  uint64_t getNumberRx_rsu(uint64_t rsuID) {return m_nrx_per_rsu[rsuID];}
  /**
   * @brief Get the average number of vehicles within the PRR baseline for a specific RSU.
   * @param messagetype  The ID of the VRU.
   * @return  The average number of vehicles within the baseline.
   */
  double getAverageNumberOfVehiclesInBaseline_rsu(uint64_t rsuID) {return m_avg_nvehbsln_per_rsu[rsuID];}

  void enablePRRVerboseOnStdout() {m_prr_verbose_stdout=true;}
  void disablePRRVerboseOnStdout() {m_prr_verbose_stdout=false;}
  void enableCBRVerboseOnStdout() {m_cbr_verbose_stdout=true;}
  void disableCBRVerboseOnStdout() {m_cbr_verbose_stdout=false;}

  /**
   * @brief Add a vehicle ID to the list of IDs to be excluded from the PRR computation.
   * @param m_vehid The ID of the vehicle to be excluded.
   */
  void addExcludedID(uint64_t m_vehid) {m_excluded_vehID_list.insert(m_vehid); m_excluded_vehID_enabled=true;}
  void clearExcludedIDs() {m_excluded_vehID_list.clear(); m_excluded_vehID_enabled=false;}


  /**
   * @brief Modify the timeout value for the PRR computation.
   *
   * This function lets the user customize the timeout value (in seconds) for the PRR computation.
   * By default, the PRRSupervisor starts a timeout of 3 seconds when a packet is sent. It will
   * then record all the times the same packet is received by other road users within the baseline and within
   * the 3 seconds (increasing each time the "X" at the numerator of the formula PRR = X/Y). After 3 seconds
   * the PRR for that packet is computed. Therefore, all packets with a latency greater than 3 seconds are
   * considered as lost (and they likely are lost).
   * The default value of 3 seconds has been considered as worst case, considering that this is the same
   * periodicity as the one of GeoNetworking beacons, when no CAMs are being exchanged.
   * With this function, the timeout can be adjusted depending on the user needs.
   *
   * @param prr_comp_timeout_sec The new timeout value in seconds.
   */
  void modifyPRRComputationTimeout(double prr_comp_timeout_sec) {m_pprcomp_timeout=prr_comp_timeout_sec;}


  void startCheckCBR(int num_nodes=-1);
  /**
   * @breif This function enables the writing of the CBR values to a file.
   */
  void enableCBRWriteToFile() {m_cbr_write_to_file=true;}
  /**
   * @breif This function disables the writing of the CBR values to a file.
   */
  void disableCBRWriteToFile() {m_cbr_write_to_file=false;}
  /**
   * @breif This function sets the CBR window value in Milliseconds.
   */
  void setCBRWindowValue(float window) {m_cbr_window=window;}
  /**
   * @breif This function sets the alpha value.
   */
  void setCBRAlphaValue(float alpha) {m_cbr_alpha=alpha;}
  /**
   * @breif This function sets the simulation time in Seconds.
   */
  void setSimulationTimeValue(float simTime) {m_simulation_time=simTime;}
  /**
   * @breif This function sets the channel technology.
   */
  void setChannelTechnology(std::string channelTechnology)
  {
    // Define the set of valid channel technologies
    std::set<std::string> validChannelTechnologies = {"80211p", "80211bd", "Nr", "Lte", "CV2X"};

    // Check if the provided channelTechnology is valid
    if (validChannelTechnologies.find(channelTechnology) == validChannelTechnologies.end()) {
        // If the channelTechnology is not valid, throw an error
        NS_FATAL_ERROR("Invalid channel technology. Must be one of '80211p', '80211bd', 'Nr', 'Lte', or 'CV2X'.");
      }

    // If the channelTechnology is valid, set it
    m_channel_technology = channelTechnology;
  }
  /**
   * @breif This function sets the window value for Channel Occupation checks in Seconds.
   */
  void setChannelWindowValue(float window) {m_channel_window=window;}

  /**
   * @breif This function gets the CBR for a specific node.
   */
  double getCBRPerItem(std::string vehicleID);
  /**
   * @breif This function gets the overall CBR.
   */
  float getAverageCBROverall();
  /**
   * @breif This function gets the CBR values for all the nodes.
   */
  std::unordered_map<std::string, std::vector<double>> getCBRValues();
  /**
   * @breif This function gets the channel technology.
   */
  std::string getChannelTechnology() {return m_channel_technology;};
  /**
   * @brief This function starts the channelOccupationBytesPerSecondsPerSquareMeter.
   */
  void startChannelOccupationBytesPerSecondsPerSquareMeter() {Simulator::Schedule (Seconds(m_channel_window), &MetricSupervisor::channelOccupationBytesPerSecondsPerSquareMeter, this);};
  /**
   * @brief This function stores the relation between CBR (first member of the tuple) and the number of bytes per square meter per second (second member of the tuple).
   */
  void channelOccupationBytesPerSecondsPerSquareMeter();
  /**
   * @brief This function updates the size fo received bytes
   */
   void updateBytesReceived(uint32_t bytesReceived)
   {
     m_total_bytes += bytesReceived;
   }
   /**
    * @brief This function gets all the CBR values (first member of each tuple) paired with the Channel Occupation Bytes per Second per Square meter (second member of each tuple)
    */
    std::vector<std::tuple<double, double>> getCBRValuesPerChannelOccupation() {return m_cbr_bytes_per_second_per_square_meter;};
    /**
    * @brief This function gets the Channel Occupation Bytes per Second per Square meter
    */
    std::vector<double> getBytesPerSecondPerSquareMeter() {return m_bytes_per_second_per_square_meter;};

    void setNodeContainer(NodeContainer nc) {m_node_container = nc;}

    void writeCBRtoCSV(std::string filename) {m_cbr_csv_log_file=filename;}
private:
  void computePRR(std::string buf);

  /**
   * @breif This function computes the CBR for each node..
   */
  void checkCBR();
  /**
   * @breif This function logs the last CBR values for each node and write the results into a file.
   */
  void logLastCBRs();

  std::unordered_map<std::string,baselineVehicleData_t> m_packetbuff_map; //! key: packet, value: list of vehicle IDs
  std::unordered_map<std::string,int64_t> m_latency_map; //! key: packet, value: latency
  std::unordered_map<std::string,uint64_t> m_id_map; //! key: packet, value: ID
  std::unordered_map<std::string,messageType_e> m_messagetype_map; //! key: packet, value: message type
  std::unordered_map<std::string,StationType_t> m_stationtype_map; //! key: packet, value: station type

  int m_count = 0;
  uint64_t m_count_latency = 0;
  uint64_t m_total_tx = 0.0;
  uint64_t m_total_rx = 0.0;
  double m_avg_PRR = 0.0;
  double m_avg_latency_ms = 0.0;
  std::unordered_map<uint64_t,double> m_avg_PRR_per_veh; //! key: vehicle ID, value: PRR
  std::unordered_map<uint64_t,double> m_avg_PRR_per_ped; //! key: pedestrian ID, value: PRR
  std::unordered_map<uint64_t,double> m_avg_PRR_per_rsu; //! key: RSU ID, value: PRR
  std::unordered_map<uint64_t,double> m_avg_latency_ms_per_veh;  //! key: vehicle ID, value: latency
  std::unordered_map<uint64_t,double> m_avg_latency_ms_per_ped;  //! key: pedestrian ID, value: latency
  std::unordered_map<uint64_t,double> m_avg_latency_ms_per_rsu;  //! key: RSU ID, value: latency
  std::unordered_map<uint64_t,uint64_t> m_ntx_per_veh;  //! key: vehicle ID, value: total number of packets transmitted per vehicle
  std::unordered_map<uint64_t,uint64_t> m_ntx_per_ped;  //! key: pedestrian ID, value: total number of packets transmitted per VRU
  std::unordered_map<uint64_t,uint64_t> m_ntx_per_rsu;  //! key: RSU ID, value:  total number of packets transmitted per RSU
  std::unordered_map<uint64_t,uint64_t> m_nrx_per_veh;  //! key: vehicle ID, value: total number of packets received per vehicle
  std::unordered_map<uint64_t,uint64_t> m_nrx_per_ped;  //! key: pedestrian ID, value: total number of packets received per VRU
  std::unordered_map<uint64_t,uint64_t> m_nrx_per_rsu;  //! key: RSU ID, value: total number of packets received per RSU
  std::unordered_map<uint64_t,double> m_avg_nvehbsln_per_veh;  //! key: vehicle ID, value: average number of road users within the baseline used for the PRR computation
  std::unordered_map<uint64_t,double> m_avg_nvehbsln_per_ped;  //! key: pedestrian ID, value: average number of road users within the baseline used for the PRR computation
  std::unordered_map<uint64_t,double> m_avg_nvehbsln_per_rsu;  //! key: RSU ID, value: average number of road users within the baseline used for the PRR computation

  std::unordered_map<uint64_t,int> m_count_per_veh; //! key: vehicle ID, value: count
  std::unordered_map<uint64_t,int> m_count_per_ped;  //! key: pedestrian ID, value: count
  std::unordered_map<uint64_t,int> m_count_per_rsu;  //! key: RSU ID, value: count
  std::unordered_map<uint64_t,uint64_t> m_count_latency_per_veh;    //! key: vehicle ID, value: count for latency computation
  std::unordered_map<uint64_t,uint64_t> m_count_latency_per_ped;    //! key: pedestrian ID, value: count for latency computation
  std::unordered_map<uint64_t,uint64_t> m_count_latency_per_rsu;    //! key: RSU ID, value: count for latency computation
  std::unordered_map<uint64_t,uint64_t> m_count_nvehbsln_per_veh;    //! key: vehicle ID, value: count for average number of vehicles within the baseline computation
  std::unordered_map<uint64_t,uint64_t> m_count_nvehbsln_per_ped;    //! key: pedestrian ID, value: count for average number of vehicles within the baseline computation
  std::unordered_map<uint64_t,uint64_t> m_count_nvehbsln_per_rsu;    //! key: RSU ID, value: count for average number of vehicles within the baseline computation

  std::unordered_map<messageType_e,double> m_avg_PRR_per_messagetype; //! key: message type, value: PRR
  std::unordered_map<messageType_e,double> m_avg_latency_ms_per_messagetype; //! key: message type, value: latency
  std::unordered_map<messageType_e,int> m_count_per_messagetype; //! key: message type, value: count
  std::unordered_map<messageType_e,uint64_t> m_count_latency_per_messagetype; //! key: message type, value: latency
  std::unordered_map<messageType_e,uint64_t> m_ntx_per_messagetype; //! key: message type, value: total number of packets transmitted per message type
  std::unordered_map<messageType_e,uint64_t> m_nrx_per_messagetype; //! key: message type, value: total number of packets received per message type
  std::unordered_map<messageType_e,uint64_t> m_count_nvehbsln_per_messagetype; //! key: message type, value: count for average number of vehicles within the baseline computation
  std::unordered_map<messageType_e,double> m_avg_nvehbsln_per_messagetype;  //! key: message type, value: average number of road users within the baseline used for the PRR computation for that message type

  std::unordered_map<uint64_t, std::vector<double>> m_sinr_per_veh; //! key: vehicle ID, value: SINR
  
  Ptr<TraciClient> m_traci_ptr = nullptr;
  Ptr<OpenCDAClient> m_carla_ptr = nullptr;
  std::vector<GPSTraceClient *> m_v_gpstc;
  double m_baseline_m = 150.0;

  bool m_prr_verbose_stdout = false;
  bool m_cbr_verbose_stdout = false;

  std::list<EventId> eventList;

  std::set<uint64_t> m_excluded_vehID_list;
  bool m_excluded_vehID_enabled = false;

  double m_pprcomp_timeout = 3.0;

  uint64_t m_stationId_baseline = 1000000;

  double m_cbr_window = -1; //!< The window for the CBR computation
  float m_cbr_alpha = -1; //!< The alpha parameter for the exponential moving average
  bool m_cbr_write_to_file = false; //!< True if the CBR values are written to a file, false otherwise
  std::string m_cbr_csv_log_file = ""; //!< Optional file name to which each CBR value should be written at each CBR computation window
  std::string m_channel_technology = ""; //!< The channel technology used
  NodeContainer m_node_container;
  float m_simulation_time = -1; //!< The simulation time
  std::unordered_map<std::string, std::vector<double>> m_average_cbr; //!< The exponential moving average CBR for each node

  uint32_t m_total_bytes = 0; //!< The total number of bytes received in the simulation
  double m_total_area = 0.0; //!< The total area for the simulation
  double m_channel_window = 2; //!< The time window (in Seconds) for the getChannelOccupationBytesPerSquareMeterPerSeconds function
  uint64_t m_received_bytes_checkpoint = 0; //!< The number of received bytes until the last time window checkpoint
  std::vector<double> m_bytes_per_second_per_square_meter; //!< The number of bytes per second per square meter
  std::vector<std::tuple<double, double>> m_cbr_bytes_per_second_per_square_meter; //!< The number of packet per second per square meter;
};
}


#endif // METRICSUPERVISOR_H
