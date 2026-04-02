//
// Created by diego on 01/12/25.
//

#ifndef NS3_FORESEE_H
#define NS3_FORESEE_H

// #include "ns3/core-module.h"
// #include "ns3/LDM.h"
#include "ns3/mcBasicService.h"
#include "ns3/trajectoryPrediction.h"
#include "ns3/geonet.h"

#define MAX_DIST_AHEAD_BEHIND 50
#define ACCELERATION_STEP 0.5
#define MIN_TTC 3
#define DEFAULT_ACC_VALUE 500
#define TRAJECTORY_PER_SUBM 10
#define MIN_DECELERATION -2
#define MAX_LOOPS 50
#define NO_SOLUTION 200

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
  } Strategy;

  struct IDMParams { double v0, T, s0, a, b; };

  foresee() = default;
  ~foresee() {
      delete m_traj_predictor;
  };
  IDMParams getIDMParams(StationType type);
  double idmAcceleration(double v, double v_lead, double gap, double v0,
                          double T, double s0, double a, double b);
  std::tuple<double, double> computeRequiredDeceleration(double speed_leader, double speed_follower,
                                               double current_gap, IDMParams p,
                                               double dt = 0.1, double horizon = 3.0);
  std::tuple<double, double> computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt = 0.1, double horizon = 3.0);
  void WrapperFORESEEMobilityModel();
  void FORESEEMobilityModel();
  void setStationType(StationType_t type) {m_station_type = type;};
  void setNode(Ptr<Node> node) {m_node = node;};
  void setLDM (Ptr<LDM> ldm) {m_LDM = ldm;};
  void setTraciAPI (Ptr<TraciClient> traci) {m_traci = traci;};
  void setNumberOfLanes ();
  void setVDP (VDP* vdp) {m_vdp = vdp;};
  void setDesiredSpeed (double speed) {m_desired_speed = speed;};
  void setVehicleID (std::string vehicleID) {m_vehicle_id = vehicleID; m_vehicle_id_int = std::stol(vehicleID.substr (3));};
  void setCurrentLCData(std::unordered_map<ulong, std::tuple<float, float, float>>* lc_data_structure) {m_lc_data_structure = lc_data_structure;};
  void setCoordinationAvoidanceRange(float ca_range) {m_ca_range = ca_range;};
  void setMCBasicService(Ptr<MCBasicService> mcs_ptr) {m_mcs_ptr = mcs_ptr;};
  void setStartTime(uint8_t startTime) {m_start_time = startTime;};
  void setTrajectoryPredictor(int horizon_time, int step_time, int negotiation_time, int deceleration_time, int lc_duration, PredictionType prediction_type);
  // static std::tuple<bool, double> trajectoryEvaluation(std::vector<trajectoryPrediction::TrajectoryItem> trajectory_HV, std::vector<trajectoryPrediction::TrajectoryItem> trajectory_other, double leader_length, int step_time, int negotiation_time, int lc_duration, trajectoryPrediction::ActorType type, int start_time);
  void terminateCoordination ();
  void startCoordination (long RV_id, long RVAhead_id, double dec_rv, double acc_rv_ahead, double time_rv, double time_rv_ahead, bool left_criterion);
  void addMCMRxCallback();
  void receiveMCM(asn1cpp::Seq<MCM> mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info);
  void negotiationPhase(bool left_criterion);


private:
  std::string m_vehicle_id;
  uint64_t m_vehicle_id_int;
  Ptr<LDM> m_LDM;
  Ptr<TraciClient> m_traci;
  VDP* m_vdp;
  int m_FORESEE_check_ms = 1000;
  int m_max_reception_mcs = 1000;
  double m_desired_speed = 0;
  double m_delta_ls = 0.5;
  double m_delta_ds = 0.5;
  double m_offset = 0.3;
  int m_num_lanes = 0;
  int m_time_to_lc;

  std::unordered_map<ulong, std::tuple<float, float, float>>* m_lc_data_structure;
  float m_ca_range;
  Ptr<MCBasicService> m_mcs_ptr;
  uint8_t m_start_time;
  trajectoryPrediction* m_traj_predictor;
  PredictionType m_prediction_type = UNKNOWN;
  int m_step_time;
  int m_negotiation_time;
  int m_FORESEE_max_time = 10000;
  bool m_busy_with_maneuver = false;

  EventId m_termination_event;
  Ptr<Node> m_node = nullptr;
  std::unordered_map<std::string, Ptr<Socket>> m_socket_map;
  StationType_t m_station_type;
  std::function<void(asn1cpp::Seq<MCM>, Address, StationID_t, StationType_t, SignalInfo)> m_MCMReceiveCallbackExtended = nullptr;
  bool m_real_time;
  Strategy m_strategy;
  bool m_coordinator = false;
  std::unordered_map<StationID_t, Strategy> m_acceptance_map;
  StationId_t m_target;

};
}


#endif //NS3_FORESEE_H
