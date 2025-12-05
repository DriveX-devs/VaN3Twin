#ifndef NS3_DCC_H
#define NS3_DCC_H

#include <string>
#include <vector>
#include <tuple>
#include "ns3/MetricSupervisor.h"
#include "ns3/wifi-net-device.h"
//#include "ns3/nr-net-device.h"
#include "ns3/net-device.h"
#include "ns3/wifi-phy.h"
//#include "ns3/nr-ue-phy.h"
#include "ns3/traci-client.h"
#include "ns3/basic-header.h"
#include "ns3/common-header.h"
#include "ns3/longpositionvector.h"
#include "ns3/btpdatarequest.h"
#include "ns3/MessageId.h"
// #include "ns3/BSMap.h"
// #include "ns3/nr-module.h"
// #include "ns3/geonet.h"

namespace ns3 {

/**
 * \ingroup automotive
 *
 * \brief This class implements the Decentralized Congestion Control (DCC) algorithm.
 *
 * This class provides capabilities for computing both the Reactive DCC and the Proactive DCC.
 */

typedef struct QueuePacket {
  int time;
  GNBasicHeader bh;
  GNCommonHeader ch;
  GNlpv_t long_PV;
  GNDataRequest_t dataRequest;
  MessageId_t message_id;
} QueuePacket;

class DCC : public Object
{
public:

  static TypeId GetTypeId(void);
  /**
   * \brief Default constructor
   *
   */
  DCC ();
  ~DCC();

  /**
    * \brief Setup DCC
    */
  void SetupDCC(std::string item_id, Ptr<MetricSupervisor> met_sup, Ptr<Node> node, std::string modality, uint32_t dcc_interval, float cbr_target=0.63, int queue_length=0, int max_lifetime=100, std::string log_file="");

  void StartDCC();

  void setMetricSupervisor(MetricSupervisor *met_sup_ptr) {m_metric_supervisor = met_sup_ptr;}
  void setLastTx(float t) {m_last_tx = t;}
  void cleanQueues(int now);
  void enqueue(int priority, QueuePacket p);
  std::tuple<bool, QueuePacket> dequeue(int priority);

  void updateTgoAfterStateCheck(uint32_t Toff);
  void updateTonpp(ssize_t pktSize);
  bool checkGateOpen(int64_t now);
  void updateTgoAfterDeltaUpdate();
  void updateTgoAfterTransmission();
  std::string getModality() {return m_modality;}
  void setBitRate(long bitrate) {m_bitrate_bps = bitrate;}
  void setSendCallback(std::function<void(const QueuePacket&)> cb);
  void setCBRGCallback(std::function<void()> cb);
  Ptr<WifiPhy> GetPhy();
  double getCBRTarget() const {return m_CBR_target;};
  void setCBRG(double cbr_g);
  void setNewCBRL0Hop (double cbr);
  double getCBRR0 () {return m_CBR_L0_Hop[0];}
  double getCBRL0Prev () {return m_CBR_L0_Hop[1];}
  void setCBRR1 (double cbr_r1) {m_CBR_R1_Hop = cbr_r1;};
  double getCBRR1 () {return m_CBR_R1_Hop;};

private:

  typedef enum ReactiveState
  {
    Relaxed,
    Active1,
    Active2,
    Active3,
    Restrictive
  } ReactiveState;

  typedef struct ReactiveParameters {
    double cbr_threshold;
    double tx_power;
    double data_rate;
    long tx_inter_packet_time;
    double sensitivity;
  } ReactiveParameters;

  const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_500_us =
      {
          {Relaxed,     {0.3, 30.0, -1, 50, -95.0}},
          {Active1,     {0.4, 30.0, -1, 100,  -95.0}},
          {Active2,     {0.5, 30.0, -1, 200, -95.0}},
          {Active3,     {0.65, 12.0, -1, 250, -95.0}},
          {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
      };

  const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_1ms =
      {
          // Values represent: CBR threshold, Tx Power [dBm], Data Rate [Mbit/s], Tx Inter Packet Time [ms], Rx Sensitivity [dBm]
          {Relaxed,     {0.3, 30.0, -1, 100, -95.0}},
          {Active1,     {0.4, 30.0, -1, 200,  -95.0}},
          {Active2,     {0.5, 18.0, -1, 400, -95.0}},
          {Active3,     {0.6, 12.0, -1, 500, -95.0}},
          {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
      };

  /**
   * \brief Start the reactive DCC mechanism
   *
   */
  void reactiveDCC();
  /**
   * \brief Start the adaptive DCC mechanism
   *
   */
  void adaptiveDCC();
  /**
   * \brief Start the CBR check for the adaptive DCC mechanism
   *
   */
  void adaptiveDCCcheckCBR();

  void DCCcheckCBRG();

  void checkQueue();

  std::unordered_map<ReactiveState, ReactiveParameters> getConfiguration(double Ton, double currentCBR);

  std::string m_item_id;
  Ptr<Node> m_node;
  std::string m_modality = ""; //!< Boolean to indicate if the DCC is reactive or adaptive
  uint32_t m_dcc_interval = -1; //!< Time interval for DCC
  Ptr<MetricSupervisor> m_metric_supervisor = NULL; //!< Pointer to the MetricSupervisor object
  ReactiveState m_current_state = ReactiveState::Relaxed;

  double m_CBR_its = -1;
  double m_alpha = 0.016;
  double m_beta = 0.0012;
  double m_CBR_target = 0.62;
  double m_delta_max = 0.03;
  double m_delta_min = 0.0006;
  double m_Gmax = 0.0005;
  double m_Gmin = -0.00025;
  uint32_t m_T_CBR = 100; // Check the CBR value each 100 ms for Adaptive DCC from standard suggestion
  double m_delta = 0;
  double m_previous_cbr = -1;

  float m_Tpg_ms = 0.0;
  float m_Tgo_ms = 0.0;
  float m_Ton_pp = 0.5;
  float m_Toff_ms = 0.0;
  float m_last_tx = 0.0;
  long m_bitrate_bps = 3;
  std::string m_dcc = "";
  float m_cbr = 0.0;
  uint8_t m_queue_length = 0;
  long m_lifetime{}; // ms
  struct GNDataIndication_t; // forward declaration to avoid circular import with geonet.h

  std::vector<QueuePacket> m_dcc_queue_dp0;
  std::vector<QueuePacket> m_dcc_queue_dp1;
  std::vector<QueuePacket> m_dcc_queue_dp2;
  std::vector<QueuePacket> m_dcc_queue_dp3;

  uint32_t m_dropped_by_gate = 0;
  std::string m_log_file = "";
  std::function<void(const QueuePacket&)> m_send_callback;
  std::function<void()> m_cbr_g_callback;

  double m_current_cbr = 0;

  uint8_t m_T_DCC_NET_Trig = 100;
  std::vector<double> m_CBR_G = {-1, -1};
  std::vector<double> m_CBR_L0_Hop = {0, 0};
  double m_CBR_R1_Hop = 0.0;
};

}



#endif //NS3_DCC_H
