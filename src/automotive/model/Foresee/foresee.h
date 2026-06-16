//
// Created by diego on 01/12/25.
//

#ifndef NS3_FORESEE_H
#define NS3_FORESEE_H

// #include "ns3/core-module.h"
// #include "ns3/LDM.h"
#include "ns3/asn_utils.h"
#include "ns3/event-id.h"
#include "ns3/int64x64.h"
#include "ns3/ldm-utils.h"
#include "ns3/mcBasicService.h"
#include "ns3/geonet.h"
#include <cstdint>
#include <unordered_map>

#define MAX_DIST_AHEAD_BEHIND 50
#define ACCELERATION_STEP 0.5
#define MIN_TTC 3
#define DEFAULT_ACC_VALUE 500
#define TRAJECTORY_PER_SUBM 10
#define MIN_DECELERATION -2
#define MAX_LOOPS 50
#define NO_SOLUTION 200
#define MAX_DISTANCE_TRAVELED_TO_COORDINATE 4850

namespace ns3
{
class foresee
{
public:
  enum PredictionType
  {
    UNKNOWN,
    CONSTANT_SPEED,
    CONSTANT_ACCELERATION,
  };

  typedef struct Strategy
  {
    bool accepted;
    double acceleration;
    double time;
    DeclineReason_t decline_reason;
  } Strategy;

  struct IDMParams { double v0, T, s0, a, b, d; };

  struct CoordinationLog {
      std::string coordination_id;
      int64_t sim_time_ms;
      double lane_speed_hv;
      double lane_speed_target;
      StationType_t type_hv;
      StationType_t type_rv;
      StationType_t type_rvahead;
      StationType_t type_rv1;
      StationType_t type_rv2;
      StationType_t type_rvahead1;
      StationType_t type_rvahead2;
      double gap_hv_rv;                       // pos_hv - pos_rv
      double gap_hv_rvahead;                  // pos_hv - pos_rvahead
      double gap_rv_rvahead;                  // pos_rv - pos_rvahead
      double gap_rvahead_rvahead1;            // pos_rvahead - pos_rvahead1
      double gap_rvahead1_rvahead2;           // pos_rvahead1 - pos_rvahead2
      double gap_rv_rv1;                      // pos_rv - pos_rv1
      double gap_rv1_rv2;                     // pos_rv1 - pos_rv2
      double desired_speed_hv;
      double desired_speed_rv;
      double desired_speed_rvahead;
      double rel_desired_speed_hv_rv;         // desired_speed_hv - desired_speed_rv
      double rel_desired_speed_hv_rvahead;    // desired_speed_hv - desired_speed_rvahead
      double rel_desired_speed_rv_rvahead;    // desired_speed_rv - desired_speed_rvahead
      double rel_speed_hv_rv;                 // speed_hv - speed_rv
      double rel_speed_hv_rvahead;            // speed_hv - speed_rvahead
      double rel_speed_rv_rvahead;            // speed_rv - speed_rvahead
      double rel_speed_rvahead_rvahead1;      // speed_rvahead - speed_rvahead1
      double rel_speed_rvahead1_rvahead2;     // speed_rvahead1 - speed_rvahead2
      double rel_speed_rv_rv1;                // speed_rv - speed_rv1
      double rel_speed_rv1_rv2;               // speed_rv1 - speed_rv2
      double rel_acc_hv_rv;                   // acc_hv - acc_rv
      double rel_acc_hv_rvahead;              // acc_hv - acc_rvahead
      double rel_acc_rv_rvahead;              // acc_rv - acc_rvahead
      double rel_acc_rvahead_rvahead1;        // acc_rvahead - acc_rvahead1
      double rel_acc_rvahead1_rvahead2;       // acc_rvahead1 - acc_rvahead2
      double rel_acc_rv_rv1;                  // acc_rv - acc_rv1
      double rel_acc_rv1_rv2;                 // acc_rv1 - acc_rv2
      double dec_rv_requested;
      double acc_rvahead_requested;
      double time_rv_requested;
      double time_rvahead_requested;
      // Outcome
      /*
        0 = failed after negotiation (execution failed)
        1 = failed during negotiation (refused)
        2 = success
      */
      int execution_success;
  };

  struct BlockedStruct {
    long time;
    std::set<StationId_t> participants;
  };

  foresee() = default;
  ~foresee() = default;
  IDMParams getIDMParams(StationType_t type, double desired_speed);
  double idmAcceleration(double v, double v_lead, double gap, double v0,
                          double T, double s0, double a, double b, double d);
  std::tuple<double, double> computeRequiredDeceleration(double speed_leader, double speed_follower,
                                               double current_gap, IDMParams p,
                                               double dt = 0.1, double horizon = 5.0);
  std::tuple<double, double> computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt = 0.1, double horizon = 5.0);
  void WrapperFORESEEMobilityModel(bool start_foresee);
  void FORESEEMobilityModel();
  void deleteEvents();
  void setStationType(StationType_t type) {m_station_type = type;};
  void setNode(Ptr<Node> node) {m_node = node;};
  void setLDM (Ptr<LDM> ldm) {m_LDM = ldm;};
  void setTraciAPI (Ptr<TraciClient> traci) {m_traci = traci;};
  void setNumberOfLanes ();
  void setVerbose(bool verbose) {m_verbose = verbose;};
  void setRegisterLog() {m_register_log = true;};
  void setLogFile(std::string log_file);
  void setSeed(int seed) {m_seed = seed;};
  void setVDP (VDP* vdp) {m_vdp = vdp;};
  void setManeuverHorizon(int horizon) {m_maneuver_horizon = horizon;};
  void setDesiredSpeed (double speed) {m_desired_speed = speed;};
  void setVehicleID (std::string vehicleID) {m_vehicle_id = vehicleID; m_vehicle_id_int = std::stol(vehicleID.substr (3));};
  void setCoordinationAvoidanceRange(double ca_range) {m_ca_range = ca_range;};
  void setMCBasicService(Ptr<MCBasicService> mcs_ptr) {m_mcs_ptr = mcs_ptr;};
  void setStartTime(int startTime) {m_start_time = startTime;};
  void setNegotiationTime(int negotiationTime) {m_negotiation_time = negotiationTime;}
  void terminateCoordination ();
  void startCoordination (long RV_id, long RVAhead_id, double dec_rv, double acc_rv_ahead, double time_rv, double time_rv_ahead, bool left_criterion, int target_lane);
  void addMCMRxCallback();
  void receiveMCM(const asn1cpp::Seq<MCM>& mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info);
  void negotiationPhase(bool left_criterion, int target_lane);
  void targetCheckACK();
  void executeManeuver();
  void continueWithConstantSpeed(StationId_t coordinator);
  void targetWatchdog(StationId_t coordinator);
  void cleanupBlockedCoordinations();
  void checkLane(int target_lane_id);
  const std::vector<CoordinationLog>& getCoordinationLog() const { return m_coordination_log; }
  void ScheduleNextCheck(Time delay);

private:
  std::string m_vehicle_id;
  uint64_t m_vehicle_id_int;
  Ptr<LDM> m_LDM;
  Ptr<TraciClient> m_traci;
  VDP* m_vdp;
  int m_FORESEE_check_ms = 5000;
  int m_cleanup_ms = 1000;
  int m_max_reception_mcs = 1000;
  double m_desired_speed = 0;
  double m_delta_ls = 0.5;
  double m_delta_ds = 0.5;
  double m_offset = 0.3;
  int m_num_lanes = 0;
  int m_time_to_lc = 0;

  std::unordered_map<StationId_t, struct BlockedStruct> m_blocked_by_other_coordinations; 
  double m_ca_range;
  Ptr<MCBasicService> m_mcs_ptr;
  int m_start_time;
  int m_step_time;
  int m_negotiation_time;
  int m_FORESEE_max_time = 10000;
  int m_maneuver_horizon = 5000;

  EventId m_foresee_event;
  EventId m_cleanup_event;
  EventId m_watchdog_event;
  EventId m_negotiation_event;
  EventId m_change_lane_event;
  EventId m_tx_mcm_event;
  EventId m_continue_constant_speed_event;
  Ptr<Node> m_node = nullptr;
  StationType_t m_station_type;
  std::function<void(const asn1cpp::Seq<MCM>& mcm, Address, StationID_t, StationType_t, SignalInfo)> m_MCMReceiveCallbackExtended = nullptr;
  bool m_real_time;
  Strategy m_strategy;
  bool m_coordinator = false;
  std::unordered_map<StationID_t, Strategy> m_acceptance_map;
  StationId_t m_my_coordinator = -1;
  bool m_my_coordinator_responded = false;
  EventId m_ack_event;
  bool m_busy_with_maneuver = false;
  int m_coordination_timeout_ms = 9000;
  std::tuple<double, double> m_required_acceleration_time;
  bool m_verbose = false;
  bool m_left_criterion;
  int m_target_lane;
  int m_seed;

  bool m_register_log = false;
  std::vector<struct CoordinationLog> m_coordination_log;
  uint64_t m_coordination_counter = 0;
  struct CoordinationLog m_coordination_structure;

  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_dist{1.0, 5.0};
  
  // Shared pointer for edge cases
  std::shared_ptr<bool> m_alive;
};
}


#endif //NS3_FORESEE_H
