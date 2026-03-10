//
// Created by diego on 01/12/25.
//

#ifndef NS3_FORESEE_H
#define NS3_FORESEE_H

// #include "ns3/core-module.h"
// #include "ns3/LDM.h"
#include "ns3/mcBasicService.h"

#define HORIZON_TIME 8
#define NEGOTIATION_TIME 1
#define DECELERATION_TIME 1
#define STEP_TIME .5
#define COMFORT_DECELERATION -2
#define SAFE_DECELERATION -4
#define MAX_DIST_AHEAD_BEHIND 50

namespace ns3
{
class foresee
{
public:
  typedef struct {
    std::string RV;
    std::string HVAhead;
    std::string RVAhead;
  }FORESEEActors;

  typedef struct {
    double x;
    double y;
    double speed;
  }TrajectoryItem;

  foresee() = default;
  ~foresee() = default;
  void WrapperFORESEEMobilityModel();
  void FORESEEMobilityModel();
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
  void terminateCoordination ();
  void doCoordination ();
  std::vector<TrajectoryItem> predictConstantSpeed(double x, double y, double speed, int8_t sign, bool is_RV = false);
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
  float m_time_to_lc = 1.5;
  std::unordered_map<ulong, std::tuple<float, float, float>>* m_lc_data_structure;
  float m_ca_range;
  Ptr<MCBasicService> m_mcs_ptr;
  FORESEEActors m_actors;
  uint8_t m_start_time;
};
}


#endif //NS3_FORESEE_H
