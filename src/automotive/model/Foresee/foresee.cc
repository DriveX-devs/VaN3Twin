//
// Created by diego on 01/12/25.
//

#include "foresee.h"
#include "ns3/DeclineReason.h"
#include "ns3/ForeseeIndication.h"
#include "ns3/LongitudinalAcceleration.h"
#include "ns3/asn_utils.h"
#include "ns3/assert.h"
#include "ns3/caBasicService_v1.h"
#include "ns3/event-id.h"
#include "ns3/foresee.h"
#include "ns3/ldm-utils.h"
#include "ns3/mcBasicService.h"
#include "ns3/nstime.h"
#include "ns3/phPoints.h"
#include "ns3/phy-entity.h"
#include "ns3/signalInfoUtils.h"
#include "ns3/simulator.h"
#include "ns3/sumo-TraCIDefs.h"
#include "ns3/traci-client.h"
#include "ns3/vdp.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <istream>
#include <memory>
#include <random>
#include <string>
#include <tuple>

#define DOUBLE_TOLERANCE 0.5 
#define NOT_PRESENT -2000
constexpr int N_SAMPLES = 25;

/*
TODO List:
- Spread MCMs during the execution phase (is it necessary?)
- Cancellation of the maneuver from one of the cooperands after the negotiation phase (while the maneuver is in execution) --> quiete difficult to design a refusal reason for the cooperand, and to control the message flow
- Be careful in not overwriting txMCM events (when busy with manouver)
*/

namespace ns3 {

void txMCM(Ptr<MCBasicService> mc, mcData mcmData) {
  mc->generateAndEncodeMCM(mcmData);
}

foresee::IDMParams foresee::getIDMParams(StationType_t type, double desired_speed) {
  switch (type) {
    case StationType::StationType_passengerCar:

      return {desired_speed, 0.8, 2.0, 2.5, 3.0, 4.0};

    case StationType::StationType_lightTruck:
      return {desired_speed, 1.1, 2.0, 1.5, 2.5, 4.0};

    // New types (consistent extensions)
    case StationType::StationType_heavyTruck:
      // Slower reaction, weaker accel, more cautious braking
      return {desired_speed, 1.3, 2.0, 1.0, 2.5, 4.0};

    case StationType::StationType_motorcycle:
      // Fast reaction, high accel, small gaps
      return {desired_speed, 0.6, 2.0, 3.0, 3.5, 4.0};

    case StationType::StationType_bus:
      // Heavy, smooth driving, large spacing
      return {desired_speed, 1.3, 2.0, 1.5, 2.5, 4.0};

    default:
      // Fallback: behave like a normal passenger car
      return {desired_speed, 0.8, 2.0, 2.5, 3.0, 4.0};
  }
}

double foresee::idmAcceleration(double v, double v_lead, double gap, double v0,
                          double T, double s0, double a, double b, double d) {
  // Desired minimum gap
  double s_star = s0 + std::max(0.0, v * T + (v * (v - v_lead)) / (2.0 * std::sqrt(a * b)));
  // IDM acceleration
  return a * (1.0 - std::pow(v / v0, d) - std::pow(s_star / std::max(gap, 0.1), 2));
}

/*std::tuple<double, double> foresee::computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon) {
  // Binary search on acceleration of leader in [0, a_max]
  double lo = 0.0;
  double hi = p.a; // max acceleration
  double best_delta_t = -1;
  for(int iter = 0; iter < MAX_LOOPS; iter++) {
      double a_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      double t = 0;
      double delta_t = -1;
      for(; t <= horizon; t += dt) {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b, p.d);
          if (a_f >= MIN_DECELERATION)
            {
              a_f_final = a_f;
              delta_t = t;
              break;
            }
          // Leader accelerates with candidate acceleration
          v_l  = std::min(v_l + a_candidate * dt, p.v0);
          // Update gap
          gap  += (v_l - v_f) * dt;
        }
      // Check if at end of horizon ego can merge comfortably
      if(a_f_final >= MIN_DECELERATION) {
          hi = a_candidate; // enough, try less acceleration (less positive)
          best_delta_t = delta_t;
          break;
        }
      else {
          lo = a_candidate; // not enough, need more acceleration
        }
    }
  if(std::abs(hi - p.a) < 0.1)
    return {NO_SOLUTION, -1};
  if (hi < 0.1) hi = 0.0;
  return {hi, best_delta_t}; // minimum acceleration RVAhead needs to apply
}

std::tuple<double, double> foresee::computeRequiredDeceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon) {
  // Binary search on deceleration of follower in [0, a_max]
  double lo = -p.b;
  double hi = 0.0; // max deceleration
  double best_delta_t = -1;
  for(int iter = 0; iter < MAX_LOOPS; iter++) {
      double d_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      double t = 0;
      double delta_t = -1;
      for(; t <= horizon; t += dt) {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b, p.d);
          if (a_f >= MIN_DECELERATION)
            {
              a_f_final = a_f;
              delta_t = t;
              break;
            }
          // Follower decelerates with candidate deceleration
          v_f = std::max(0.0, v_f + d_candidate * dt);
          // Update gap
          gap += (v_l - v_f) * dt;
        }
      // Check if at end of horizon ego can merge comfortably
      if(a_f_final >= MIN_DECELERATION) {
          lo = d_candidate; // enough, try less deceleration (less negative)
          best_delta_t = delta_t;
          break;
        }
      else {
          hi = d_candidate; // not enough, need more deceleration (more negative)
        }
    }
  if(lo <= -p.b + 0.01)
    return {NO_SOLUTION, -1};
  if (lo > -0.1) lo = 0.0;
  return {lo, best_delta_t}; // minimum deceleration
}*/

std::tuple<bool, double> foresee::simulateCandidate(
    double candidate, bool is_leader_case,
    double speed_leader, double speed_follower, double current_gap,
    const IDMParams& p, double dt, double horizon) {
 
  double gap = current_gap;
  double v_f = speed_follower;
  double v_l = speed_leader;
 
  for (double t = 0.0; t <= horizon + dt; t += dt) {
    double a_f = idmAcceleration(v_f, v_l, gap, p.v0, p.T, p.s0, p.a, p.b, p.d);
    if (a_f >= MIN_DECELERATION) {
      return {true, t};
    }
    if (is_leader_case) {
      // Leader accelera con il candidato
      v_l = std::min(v_l + candidate * dt, p.v0);
    } else {
      // Follower decelera con il candidato (candidate è negativo)
      v_f = std::max(0.0, v_f + candidate * dt);
    }
    gap += (v_l - v_f) * dt;
  }
  return {false, -1.0};
}

std::tuple<double, double> foresee::optimizeWeighted(
    double lo, double hi, bool is_leader_case,
    double speed_leader, double speed_follower, double current_gap,
    const IDMParams& p, double dt, double horizon, double lambda) {
 
  double range = std::abs(hi - lo);
  if (range < 1e-9) {
    return {NO_SOLUTION, -1.0};
  }
 
  double best_cost = std::numeric_limits<double>::infinity();
  double best_candidate = NO_SOLUTION;
  double best_delta_t = -1.0;
  bool found_any = false;
 
  for (int i = 0; i < N_SAMPLES; i++) {
    double candidate = lo + (hi - lo) * (static_cast<double>(i) / (N_SAMPLES - 1));
 
    auto [success, delta_t] = simulateCandidate(
        candidate, is_leader_case,
        speed_leader, speed_follower, current_gap, p, dt, horizon);
 
    if (!success) {
      continue; // candidato scartato: non porta a successo entro horizon
    }
 
    double norm_action = std::abs(candidate) / range;
    double norm_time    = delta_t / horizon;
    double cost = lambda * norm_action + (1.0 - lambda) * norm_time;
 
    if (cost < best_cost) {
      best_cost = cost;
      best_candidate = candidate;
      best_delta_t = delta_t;
      found_any = true;
    }
  }
 
  if (!found_any) {
    return {NO_SOLUTION, -1.0};
  }
  return {best_candidate, best_delta_t};
}

std::tuple<double, double> foresee::computeRequiredAcceleration(
    double speed_leader, double speed_follower, double current_gap,
    IDMParams p, double dt, double horizon, double lambda) {
 
  return optimizeWeighted(
      0.0, p.a, /*is_leader_case=*/true,
      speed_leader, speed_follower, current_gap, p, dt, horizon, lambda);
}

std::tuple<double, double> foresee::computeRequiredDeceleration(
    double speed_leader, double speed_follower, double current_gap,
    IDMParams p, double dt, double horizon, double lambda) {
 
  return optimizeWeighted(
      -p.b, 0.0, /*is_leader_case=*/false,
      speed_leader, speed_follower, current_gap, p, dt, horizon, lambda);
}

void
foresee::addMCMRxCallback () {
  std::function<void(const asn1cpp::Seq<MCM>&, Address, StationID_t, StationType_t, SignalInfo, mcData::mcDataForeseeIndication, mcData::mcDataForeseeIndication)>
  rx_callback = std::bind(&foresee::receiveMCM,
                 this,
                 std::placeholders::_1,
                 std::placeholders::_2,
                 std::placeholders::_3,
                 std::placeholders::_4,
                 std::placeholders::_5,
                 std::placeholders::_6,
                std::placeholders::_7);
  m_MCMReceiveCallbackForesee = rx_callback;
  m_mcs_ptr->addMCRxCallbackForesee (m_MCMReceiveCallbackForesee);
}

void
foresee::WrapperFORESEEMobilityModel(bool start_foresee)
{
  // Check if the number of lanes is valid
  if (m_num_lanes <= 0) {
      NS_FATAL_ERROR ("Set a number of lanes greater than 0 to use FORESEE Mobility Model.");
    }
  // Check if the LDM (Local Dynamic Map) is set
  if (m_LDM == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the LDM of the vehicle.");
    }
  // Check if TraCI (Traffic Control Interface) is set
  if (m_traci == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs TraCI.");
    }
  // Check if the desired speed is valid
  if (m_desired_speed <= 0) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs a Desired Speed greater than 0.");
    }
  // Check if the VDP (Vehicle Data Provider) is set
  if (m_vdp == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the VDP of the vehicle.");
    }
  // Check if the MCM (Maneuver Coordination Message) service is set
  if (m_mcs_ptr == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the MCM Basic Service of the vehicle.");
    }
  /*
  // Check for predictor
  if (m_prediction_type == UNKNOWN)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the prediction type.");
    }
  */
  if (m_node == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the pointer of the vehicle node.");
    }
  if (m_MCMReceiveCallbackForesee == nullptr) {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the callback for MCM.");
    }
  
  m_gen = std::mt19937(m_seed + m_vehicle_id_int);
  // Schedule the FORESEE Mobility Model to start at the specified time
  if (start_foresee) {
    m_alive = std::make_shared<bool>(true);
    m_coordination_log.reserve(25);
    ScheduleNextCheck(MilliSeconds(m_start_time));
    m_cleanup_event = Simulator::Schedule(MilliSeconds(m_cleanup_ms), &foresee::cleanupBlockedCoordinations, this);
  }
}

void
foresee::setNumberOfLanes () {
  // Retrieve the number of lanes for the current road using TraCI
  int lanes = m_traci->TraCIAPI::edge.getLaneNumber (m_traci->TraCIAPI::vehicle.getRoadID (m_vehicle_id));
  m_num_lanes = lanes;
}

void
foresee::FORESEEMobilityModel () {
  try {
    if (!*m_alive) return;
    double traveled_distance = m_vdp->getTravelledDistance();
    if (traveled_distance > MAX_DISTANCE_TRAVELED_TO_COORDINATE) {
      // It is not safe enough to start a maneuver since we are close to the end of the highway
      return;
    }
    double current_speed = m_vdp->getSpeedValue();
    if (std::abs(current_speed - m_desired_speed) <= DOUBLE_TOLERANCE) {
      // We are around the desired speed, schedule FORESEE in 10s to check again
      ScheduleNextCheck(MilliSeconds(2 * m_FORESEE_check_ms));
      return;
    }
    if (m_busy_with_maneuver) {
      // FORESEE cannot be activated in this case because we are involved in another maneuver
      // It will be scheduled after the maneuver
      return;
    }
    // Retrieve all connected vehicles (CVs) from the LDM
    std::vector<LDM::returnedVehicleData_t> vehicles;
    bool res = m_LDM->getAllCVs (vehicles);
    if (res == false) {
      // The route is empty (no perceived vehicles in the LDM)
      // FORESEE cannot be activated in this case, so we reschedule it in 5s
      ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
      return;
    }
    // Cleanup the blocked datasett in case there are old coordinations registered (e.g., the coordinator is out of range now)
    long now_ms = Simulator::Now().GetMilliSeconds();
    bool found_behind = false;
    std::set<StationId_t> found_participants;
    for (auto it = m_blocked_by_other_coordinations.begin(); it != m_blocked_by_other_coordinations.end();) {
        if (now_ms - it->second.time >= m_coordination_timeout_ms) {
            it = m_blocked_by_other_coordinations.erase(it);
        }
        else {
          found_behind = true;
          found_participants.insert(it->second.participants.begin(), it->second.participants.end());
          ++it;
        }
    }
    
    // Check whether there is another maneuver coordination that is happening in the CA range
    // If yes, ego vehicle cannot perform maneuver coordination
    if (found_behind) {
      // FORESEE cannot be activated in this case because the CA range is not free
      // Reschedule FORESEE in 5s
      ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
      return;
    }

    std::vector<std::string> active_vehicles = m_traci->vehicle.getIDList();
    
    // Data structures to store vehicle speeds and IDs per lane
    std::unordered_map<long, std::vector<double>> speeds_per_lane;
    std::unordered_map<long, std::vector<std::string>> veh_per_lane;

    // Retrieve ego vehicle's data
    double my_heading = m_vdp->getHeadingValue();
    double my_x = m_vdp->getPositionXY().x;
    double my_y = m_vdp->getPositionXY().y;
    double my_speed = m_vdp->getSpeedValue();
    std::string my_type = m_traci->vehicle.getTypeID (m_vehicle_id);
    std::string rv_type, rvahead_type, hvahead_type;
    // Lane normalized the lane in ETSI-based system --> 1 left-most lane, 2 center lane, 3 right-most lane
    VDPDataItem<int> my_lane = m_vdp->getLanePosition();

    // Process each vehicle in the LDM
    for(auto it = vehicles.begin(); it != vehicles.end(); ++it) {
        // Skip vehicles in different directions
        if ((it->vehData.heading - my_heading) > DOUBLE_TOLERANCE) continue;
        auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
        // Take the back bumper position
        double x = pos.x;
        // Skip vehicles behind the ego vehicle
        if (std::abs(my_heading - 90) < DOUBLE_TOLERANCE && x < my_x) continue;
        if (std::abs(my_heading - 270) < DOUBLE_TOLERANCE && x > my_x) continue;
        OptionalDataItem<long> lane = it->vehData.lanePosition;
        if (lane.isAvailable()) {
            // Store vehicle speed and ID in the corresponding lane
            speeds_per_lane[lane.getData()].push_back (it->vehData.speed_ms);
            veh_per_lane[lane.getData()].push_back (std::to_string (it->vehData.stationID));
          }
      }

    // Determine lane change possibilities and criteria
    bool right_has_veh = false, left_has_veh = false;
    bool can_turn_right = false, can_turn_left = false;
    if (my_lane.getData() == 1) {
        // Ego is in the leftmost lane, can only turn right
        right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
        can_turn_right = true;
        can_turn_left = false;
      }
    else if (my_lane.getData() == m_num_lanes) {
        // Ego is in the rightmost lane, can only turn left
        left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
        can_turn_left = true;
        can_turn_right = false;
      }
    else {
        // Ego can turn both left and right
        right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
        left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
        can_turn_left = true;
        can_turn_right = true;
      }

    // Check if the current lane has vehicles
    bool mine_has_veh = !speeds_per_lane[my_lane.getData()].empty();

    // Determine the minimum speed in each lane
    double min_speed_mine, min_speed_left, min_speed_right;
    if (mine_has_veh)
      min_speed_mine = *std::min_element(speeds_per_lane[my_lane.getData()].begin(), speeds_per_lane[my_lane.getData()].end());
    else
      min_speed_mine = m_desired_speed;  // Assume ego is driving at desired speed

    bool right_criterion = false;
    bool left_criterion = false;

    // Check left lane change incentive criterion
    if (can_turn_left && left_has_veh) {
        min_speed_left = *std::min_element(speeds_per_lane[my_lane.getData()-1].begin(), speeds_per_lane[my_lane.getData()-1].end());
        if (std::abs(min_speed_left - min_speed_mine) > m_delta_ls)
          {
            if (min_speed_left > min_speed_mine) {
                left_criterion = true;
              }
            else {
                double DSth_left = min_speed_mine * (1 - m_offset);
                if (m_desired_speed > DSth_left + m_delta_ds) {
                    left_criterion = true;
                  }
              }
          }
      }

    // Check right lane change incentive criterion
    if (can_turn_right && right_has_veh) {
        min_speed_right = *std::min_element(speeds_per_lane[my_lane.getData()+1].begin(), speeds_per_lane[my_lane.getData()+1].end());
        if (std::abs(min_speed_right - min_speed_mine) > m_delta_ls) {
            if (min_speed_right > min_speed_mine) {
                right_criterion = true;
              }
            else {
                double DSth_right = min_speed_right * (1 - m_offset);
                if (m_desired_speed < DSth_right - m_delta_ds) {
                    right_criterion = true;
                  }
              }
          }
      }

    // Determine if a lane change is possible and initiate coordination if needed
    // Direction for TraCI: {-1=right, 1=left}
    int8_t lc_direction = 0;
    if (left_criterion) lc_direction = 1;
    else if (right_criterion) lc_direction = -1;
    assert (lc_direction == 0 || lc_direction == 1 || lc_direction == -1);
    if (lc_direction != 0) {
      // At least one incentive criterion is satisfied
      // Check the comfort criterion
        // Take the four roles, target, ahead ego, ahead target
        std::string RV, HVAhead, RVAhead;
        long RV_id = -1, RVAhead_id = -1;
        StationType_t type_RV, type_RVAhead;
        // Check the comfort criterion
        double x_RV, x_RVAhead;
        double y_RV, y_RVAhead;
        double speed_RV, speed_RVAhead;
        double desired_speed_RV, desired_speed_RVAhead;
        double acc_RV, acc_RVAhead;
        double length_RV, length_RVAhead;
        double min_dist_rv = 10000;
        double min_dist_rv_ahead = 10000;
        std::map<uint64_t, PHData_t> ph_RV, ph_RVAhead;
        std::vector<vehicleData_t> ahead_vehicles;
        std::vector<vehicleData_t> behind_vehicles;
        // Target lane
        int target_lane = left_criterion ? my_lane.getData() - 1 : my_lane.getData() + 1;
        // Vehicles ahead of HV in the target lane
        auto& vec1 = veh_per_lane[target_lane];
        for(auto it = vehicles.begin(); it != vehicles.end(); ++it) {
            if ((it->vehData.heading - my_heading) > DOUBLE_TOLERANCE) continue;
            // if (std::find(active_vehicles.begin(), active_vehicles.end(), "veh" + std::to_string (it->vehData.stationID)) == active_vehicles.end()) continue;
            if (it->vehData.lanePosition.getData() == target_lane) {
                auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
                auto it_found = std::find (vec1.begin (), vec1.end (),
                                            std::to_string (it->vehData.stationID));
                if (it_found != vec1.end ()) {
                    // Vehicle is in the target lane ahead of ego, can be RVAhead
                    // Transform the distance into a front-back bumper distance
                    double leader_length = (double) it->vehData.vehicleLength.getData() / DECI;
                    double dist = std::abs(my_x - pos.x);
                    if (leader_length < dist) dist = dist - leader_length;
                    else dist = 0;
                    if (dist < min_dist_rv_ahead && dist < MAX_DIST_AHEAD_BEHIND) {
                      min_dist_rv_ahead = dist;
                      RVAhead = "veh" + std::to_string (it->vehData.stationID);
                      RVAhead_id = it->vehData.stationID;
                      length_RVAhead = (double) it->vehData.vehicleLength.getData() / DECI;
                      x_RVAhead = pos.x;
                      y_RVAhead = pos.y;
                      speed_RVAhead = it->vehData.speed_ms;
                      desired_speed_RVAhead = it->vehData.desired_speed.getData();
                      acc_RVAhead = it->vehData.accel_msquares;
                      type_RVAhead = static_cast<StationType_t> (it->vehData.stationType);
                      ph_RVAhead = it->phData.getPHpoints();
                    }
                    if (dist <= 2 * m_ca_range) {
                      // Select the vehicle for the group of the ahead vehicles
                      ahead_vehicles.push_back(it->vehData);
                    }
                  }
                else {
                    // Can be RV
                    // Transform the distance into a front-back bumper distance
                    double leader_length = m_vdp->getVehicleLength();
                    double dist = std::abs(my_x - pos.x);
                    if (leader_length < dist) dist = dist - leader_length;
                    else dist = 0;
                    OptionalDataItem<long> lane = it->vehData.lanePosition;
                    if (lane.isAvailable () && lane.getData () == target_lane) {
                        if (dist < min_dist_rv && dist < MAX_DIST_AHEAD_BEHIND) {
                            min_dist_rv = dist;
                            RV = "veh" + std::to_string (it->vehData.stationID);
                            RV_id = it->vehData.stationID;
                            length_RV = (double) it->vehData.vehicleLength.getData() / DECI;
                            x_RV = pos.x;
                            y_RV = pos.y;
                            speed_RV = it->vehData.speed_ms;
                            desired_speed_RV = it->vehData.desired_speed.getData();
                            acc_RV = it->vehData.accel_msquares;
                            type_RV = static_cast<StationType_t> (it->vehData.stationType);
                            ph_RV = it->phData.getPHpoints();
                          }
                        if (dist <= m_ca_range) {
                            // Select the vehicle for the group of the ahead vehicles
                            behind_vehicles.push_back(it->vehData);
                          }
                      }
                  }
              }
          }

        bool can_proceed = true;
        if (found_participants.find(RV_id) != found_participants.end() || found_participants.find(RVAhead_id) != found_participants.end()) {
          // One or more cooperands are currently busy, we need to reschedule
          can_proceed = false;
          if (m_verbose) {
            std::cout << "\n[NEGOTIATION CANNOT START]" << std::endl;
            std::cout << "Vehicle " << m_vehicle_id << " would like to coordinate but other actors are busy" << std::endl;
          }
        }

        if (can_proceed) {
          if (RV_id <= 0 && RVAhead_id <= 0) {
            // No RV and RVAhead present, we can directly change the lane
            target_lane = 3 - target_lane;
            // m_traci->vehicle.changeLane (m_vehicle_id, target_lane, m_time_to_lc);
            std::string edge_id = m_traci->vehicle.getRoadID(m_vehicle_id);
            double pos = m_traci->vehicle.getLanePosition(m_vehicle_id);
            std::string target_lane_id = edge_id + "_" + std::to_string(target_lane);
            m_traci->vehicle.moveTo(m_vehicle_id, target_lane_id, pos);
            // Since we have just changed the lane, we can schedule the future check after 10s
            ScheduleNextCheck(MilliSeconds(2 * m_FORESEE_check_ms));
          }
          else {
            // Check the comfort criterion
            double acc_rv_ahead = DEFAULT_ACC_VALUE, dec_rv = DEFAULT_ACC_VALUE;
            double time_rv_ahead, time_rv;
            bool possible_hv = true;
            bool possible_rv = true;

            if(RVAhead_id >= 0) {
              // Acceleration HV would experience with RVAhead as new leader
              IDMParams ego_params = getIDMParams(static_cast<StationType> (m_station_type), m_desired_speed);
              double a_ego_after = idmAcceleration(my_speed, speed_RVAhead, min_dist_rv_ahead,
                                                    ego_params.v0, ego_params.T,
                                                    ego_params.s0, ego_params.a, ego_params.b, ego_params.d);
              if(a_ego_after < MIN_DECELERATION) possible_hv = false;
              if(!possible_hv) {
                // It is needed to open the gap between RVAhead and HV
                // RVAhead needs to accelerate
                std::tuple<double, double> tuple = computeRequiredAcceleration (speed_RVAhead, my_speed, min_dist_rv_ahead, ego_params, 0.1, round(m_maneuver_horizon / 1e3), 0.7);
                double a = std::get<0>(tuple);
                double time = std::get<1>(tuple);
                NS_ASSERT((time >= 0.0 && time <= 5.0) || time == -1.0);
                if (std::abs(a - NO_SOLUTION) > DOUBLE_TOLERANCE){
                    acc_rv_ahead = a;
                    time_rv_ahead = time;
                    possible_hv = true;
                    // std::cout << "\n" << acc_rv_ahead << " " << time_rv_ahead << std::endl;
                  }
              }
            }

            if(RV_id >= 0) {
              // Acceleration RV would experience with HV as new leader
              IDMParams params = getIDMParams(type_RV, desired_speed_RV);
              double a_ego_after = idmAcceleration(speed_RV, my_speed, min_dist_rv,
                                                    params.v0, params.T,
                                                    params.s0, params.a, params.b, params.d);
              if(a_ego_after < MIN_DECELERATION) possible_rv = false;
              if(!possible_rv) {
                // It is needed to open the gap between RV and HV
                // RV needs to decelerate
                std::tuple<double, double> tuple = computeRequiredDeceleration (my_speed, speed_RV, min_dist_rv, params, 0.1, round(m_maneuver_horizon / 1e3), 0.7);
                double a = std::get<0>(tuple);
                double time = std::get<1>(tuple);
                NS_ASSERT((time >= 0.0 && time <= 5.0) || time == -1.0);
                if (std::abs(a - NO_SOLUTION) > DOUBLE_TOLERANCE) {
                    dec_rv = a;
                    time_rv = time;
                    possible_rv = true;
                    // std::cout << "\n" << dec_rv << " " << time_rv << std::endl;
                }
              }
            }
            
            if (possible_hv && possible_rv) {
              // If we have RV and/or RVAhead but they don't need to change their current gap with HV, we ask them to keep constant speed
              if (RV_id >= 0 && dec_rv == DEFAULT_ACC_VALUE) {dec_rv = 0; time_rv = 5;}
              if (RVAhead_id >= 0 && acc_rv_ahead == DEFAULT_ACC_VALUE) {acc_rv_ahead = 0; time_rv_ahead = 5;}
              // Need for a coordination
              int original_target_lane = target_lane;
              target_lane = 3 - target_lane;
              startCoordination(RV_id, RVAhead_id, dec_rv, acc_rv_ahead, time_rv, time_rv_ahead, left_criterion, target_lane);
              if (m_register_log) {
                // Register all the information for the csv
                m_coordination_structure.execution_success = 0;
                m_coordination_structure.coordination_id = m_vehicle_id + "_" + std::to_string(m_coordination_counter);
                m_coordination_structure.sim_time_ms = Simulator::Now().GetMilliSeconds();
                m_coordination_structure.desired_speed_hv = m_desired_speed;
                m_coordination_structure.desired_speed_rv = RV_id == -1 ? NOT_PRESENT : desired_speed_RV;
                m_coordination_structure.desired_speed_rvahead = RVAhead_id == -1 ? NOT_PRESENT : desired_speed_RVAhead;
                m_coordination_structure.lane_speed_hv = min_speed_mine;
                m_coordination_structure.lane_speed_target = left_criterion ? min_speed_left : min_speed_right;
                m_coordination_structure.type_hv = m_station_type;
                m_coordination_structure.type_rv = RV_id == -1 ? NOT_PRESENT : type_RV;
                m_coordination_structure.type_rvahead = RVAhead_id == -1 ? NOT_PRESENT : type_RVAhead;
                m_coordination_structure.gap_hv_rv = RV_id == -1 ? NOT_PRESENT : min_dist_rv;
                m_coordination_structure.gap_hv_rvahead = RVAhead_id == -1 ? NOT_PRESENT : min_dist_rv_ahead;
                if (RV_id >= 0 && RVAhead_id >= 0) {
                    double leader_length = length_RVAhead;
                    double dist = std::abs(x_RV - x_RVAhead);
                    if (dist > leader_length) dist -= leader_length; else dist = 0;
                    m_coordination_structure.gap_rv_rvahead = dist;
                  }
                else
                    m_coordination_structure.gap_rv_rvahead = NOT_PRESENT;
                m_coordination_structure.rel_desired_speed_hv_rv = RV_id == -1 ? NOT_PRESENT : m_desired_speed - desired_speed_RV;
                m_coordination_structure.rel_desired_speed_hv_rvahead = RVAhead_id == -1 ? NOT_PRESENT : m_desired_speed - desired_speed_RVAhead;
                m_coordination_structure.rel_desired_speed_rv_rvahead = (RV_id == -1 || RVAhead_id == -1) ? NOT_PRESENT : desired_speed_RV - desired_speed_RVAhead;
                m_coordination_structure.rel_speed_hv_rv = RV_id == -1 ? NOT_PRESENT : my_speed - speed_RV;
                m_coordination_structure.rel_speed_hv_rvahead = RVAhead_id == -1 ? NOT_PRESENT : my_speed - speed_RVAhead;
                m_coordination_structure.rel_speed_rv_rvahead = (RV_id == -1 || RVAhead_id == -1) ? NOT_PRESENT : speed_RV - speed_RVAhead;
                double my_acc = m_vdp->getAccelerationValue();
                m_coordination_structure.rel_acc_hv_rv = RV_id == -1 ? NOT_PRESENT : my_acc - acc_RV;
                m_coordination_structure.rel_acc_hv_rvahead = RVAhead_id == -1 ? NOT_PRESENT : my_acc - acc_RVAhead;
                m_coordination_structure.rel_acc_rv_rvahead = (RV_id == -1 || RVAhead_id == -1) ? NOT_PRESENT : acc_RV - acc_RVAhead;
                m_coordination_structure.dec_rv_requested = RV_id == -1 ? NOT_PRESENT : dec_rv;
                m_coordination_structure.acc_rvahead_requested = RVAhead_id == -1 ? NOT_PRESENT : acc_rv_ahead;
                m_coordination_structure.time_rv_requested = RV_id == -1 ? NOT_PRESENT : time_rv;
                m_coordination_structure.time_rvahead_requested = RVAhead_id == -1 ? NOT_PRESENT : time_rv_ahead;

                // --- Vehicles behind RV (rv1 = closest behind, rv2 = next) ---
                // Sort behind_vehicles by distance from RV, ascending
                std::vector<vehicleData_t> sorted_behind = behind_vehicles;
                if (RV_id >= 0) {
                  std::sort(sorted_behind.begin(), sorted_behind.end(), [&](const vehicleData_t& a, const vehicleData_t& b) {
                    auto pa = m_traci->simulation.convertLonLattoXY(a.lon, a.lat);
                    auto pb = m_traci->simulation.convertLonLattoXY(b.lon, b.lat);
                    return std::abs(x_RV - pa.x) < std::abs(x_RV - pb.x);
                  });
                }

                double speed_rv1 = NOT_PRESENT, acc_rv1 = NOT_PRESENT, x_rv1 = NOT_PRESENT;
                if (RV_id >= 0 && sorted_behind.size() >= 1) {
                  auto pos_rv1 = m_traci->simulation.convertLonLattoXY(sorted_behind[0].lon, sorted_behind[0].lat);
                  x_rv1 = pos_rv1.x;
                  speed_rv1 = sorted_behind[0].speed_ms;
                  acc_rv1 = sorted_behind[0].accel_msquares;
                  double len = length_RV;
                  double dist = std::abs(x_RV - x_rv1);
                  if (dist > len) dist -= len; else dist = 0;
                  m_coordination_structure.type_rv1 = sorted_behind[0].stationType;
                  m_coordination_structure.gap_rv_rv1 = dist;
                  m_coordination_structure.rel_speed_rv_rv1 = speed_RV - speed_rv1;
                  m_coordination_structure.rel_acc_rv_rv1 = acc_RV - acc_rv1;
                }
                else{
                  m_coordination_structure.type_rv1 = NOT_PRESENT;
                  m_coordination_structure.gap_rv_rv1 = NOT_PRESENT;
                  m_coordination_structure.rel_speed_rv_rv1 = NOT_PRESENT;
                  m_coordination_structure.rel_acc_rv_rv1 = NOT_PRESENT;
                }

                if (RV_id >= 0 && sorted_behind.size() >= 2){
                  auto pos_rv2 = m_traci->simulation.convertLonLattoXY(sorted_behind[1].lon, sorted_behind[1].lat);
                  double x_rv2 = pos_rv2.x;
                  double speed_rv2 = sorted_behind[1].speed_ms;
                  double acc_rv2 = sorted_behind[1].accel_msquares;
                  double len = (double) sorted_behind[1].vehicleLength.getData() / DECI;
                  double dist = std::abs(x_rv1 - x_rv2);
                  if (dist > len) dist -= len; else dist = 0;
                  m_coordination_structure.type_rv2 = sorted_behind[1].stationType;
                  m_coordination_structure.gap_rv1_rv2 = dist;
                  m_coordination_structure.rel_speed_rv1_rv2 = speed_rv1 - speed_rv2;
                  m_coordination_structure.rel_acc_rv1_rv2 = acc_rv1 - acc_rv2;
                }
                else{
                  m_coordination_structure.type_rv2 = NOT_PRESENT;
                  m_coordination_structure.gap_rv1_rv2 = NOT_PRESENT;
                  m_coordination_structure.rel_speed_rv1_rv2 = NOT_PRESENT;
                  m_coordination_structure.rel_acc_rv1_rv2 = NOT_PRESENT;
                }

                // --- Vehicles ahead of RVAhead (rvahead1 = closest ahead, rvahead2 = next) ---
                std::vector<vehicleData_t> sorted_ahead = ahead_vehicles;
                if (RVAhead_id >= 0){
                  std::sort(sorted_ahead.begin(), sorted_ahead.end(), [&](const vehicleData_t& a, const vehicleData_t& b) {
                    auto pa = m_traci->simulation.convertLonLattoXY(a.lon, a.lat);
                    auto pb = m_traci->simulation.convertLonLattoXY(b.lon, b.lat);
                    return std::abs(x_RVAhead - pa.x) < std::abs(x_RVAhead - pb.x);
                  });
                }

                double speed_rvahead1 = NOT_PRESENT, acc_rvahead1 = NOT_PRESENT, x_rvahead1 = NOT_PRESENT;
                if (RVAhead_id >= 0 && sorted_ahead.size() >= 1) {
                  auto pos_rvahead1 = m_traci->simulation.convertLonLattoXY(sorted_ahead[0].lon, sorted_ahead[0].lat);
                  x_rvahead1 = pos_rvahead1.x;
                  speed_rvahead1 = sorted_ahead[0].speed_ms;
                  acc_rvahead1 = sorted_ahead[0].accel_msquares;
                  double len = (double) sorted_ahead[0].vehicleLength.getData() / DECI;
                  double dist = std::abs(x_RVAhead - x_rvahead1);
                  if (dist > len) dist -= len; else dist = 0;
                  m_coordination_structure.type_rvahead1 = sorted_ahead[0].stationType;
                  m_coordination_structure.gap_rvahead_rvahead1 = dist;
                  m_coordination_structure.rel_speed_rvahead_rvahead1 = speed_RVAhead - speed_rvahead1;
                  m_coordination_structure.rel_acc_rvahead_rvahead1 = acc_RVAhead - acc_rvahead1;
                }
                else {
                  m_coordination_structure.type_rvahead1 = NOT_PRESENT;
                  m_coordination_structure.gap_rvahead_rvahead1 = NOT_PRESENT;
                  m_coordination_structure.rel_speed_rvahead_rvahead1 = NOT_PRESENT;
                  m_coordination_structure.rel_acc_rvahead_rvahead1 = NOT_PRESENT;
                }

                if (RVAhead_id >= 0 && sorted_ahead.size() >= 2) {
                  auto pos_rvahead2 = m_traci->simulation.convertLonLattoXY(sorted_ahead[1].lon, sorted_ahead[1].lat);
                  double x_rvahead2 = pos_rvahead2.x;
                  double speed_rvahead2 = sorted_ahead[1].speed_ms;
                  double acc_rvahead2 = sorted_ahead[1].accel_msquares;
                  double len = (double) sorted_ahead[1].vehicleLength.getData() / DECI;
                  double dist = std::abs(x_rvahead1 - x_rvahead2);
                  if (dist > len) dist -= len; else dist = 0;
                  m_coordination_structure.type_rvahead2 = sorted_ahead[1].stationType;
                  m_coordination_structure.gap_rvahead1_rvahead2 = dist;
                  m_coordination_structure.rel_speed_rvahead1_rvahead2 = speed_rvahead1 - speed_rvahead2;
                  m_coordination_structure.rel_acc_rvahead1_rvahead2 = acc_rvahead1 - acc_rvahead2;
                }
                else {
                  m_coordination_structure.type_rvahead2 = NOT_PRESENT;
                  m_coordination_structure.gap_rvahead1_rvahead2 = NOT_PRESENT;
                  m_coordination_structure.rel_speed_rvahead1_rvahead2 = NOT_PRESENT;
                  m_coordination_structure.rel_acc_rvahead1_rvahead2 = NOT_PRESENT;
                }
              }
            }
            else {
              // For one of the cooperand the maneuver requested is not possible, try again later
              ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
            }
          }
        }
        else {
          // One or more cooperands are currently busy, we need to reschedule
          ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
        }
      }
    else 
    {
      // No incentive found, try again later
      ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
    }
  }
  catch (...) {
    std::cout << "Error 404 FORESEE not usable for vehicle " << m_vehicle_id << " anymore!" << std::endl;
  }
}

void
foresee::startCoordination (long RV_id, long RVAhead_id, double dec_rv, double acc_rv_ahead, double time_rv, double time_rv_ahead, bool left_criterion, int target_lane) {
  // Cancel any ghost from a previous cycle
  Simulator::Cancel(m_negotiation_event);
  Simulator::Cancel(m_tx_mcm_event);
  // Choose the container
  mcData mcmData;
	mcData::mcBasicContainer mcBasicContainer{};
  mcBasicContainer.itsRole = McmItssRole_coordinatingItss; // HV is the coordinator
  mcBasicContainer.concept = 0; // MCM Goal will be set
  mcBasicContainer.cost = 0;
  mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
  mcBasicContainer.mcmType = McmType::McmType_request; // HV asks the others for a cooperation
  mcBasicContainer.maneuverID = left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane;
  // FORESEE is designed for passenger cars and trucks
  mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
  mcmData.setBasicContainer(mcBasicContainer);
  std::vector<mcData::mcDataManeuverAdvice> parsed_advices;
  if (RV_id >= 0) {
    mcData::mcDataManeuverAdvice adv;
    adv.executantID = static_cast<StationId_t>(RV_id);
    adv.currentStateAdvisedChange.setData(CurrentStateAdvisedChange_PR_stayInLane);

    mcData::mcDataAdvisedSubmaneuver subm;
    subm.submaneuverID = ManeuverID::Slowdown; // Always slow down for RV

    mcData::mcDataForeseeIndication foresee;
    foresee.longitudinalAccelerationValue = dec_rv;
    foresee.longitudinalAccelerationConfidence.setData(AccelerationConfidence::AccelerationConfidence_unavailable);
    foresee.duration = time_rv;
    mcmData.setForeseeIndicationRV(foresee);
    adv.submaneuvers.push_back(subm);
    parsed_advices.push_back(adv);
  }

  if (RVAhead_id >= 0) {
    mcData::mcDataManeuverAdvice adv;
    adv.executantID = static_cast<StationId_t>(RVAhead_id);
    adv.currentStateAdvisedChange.setData(CurrentStateAdvisedChange_PR_stayInLane);

    mcData::mcDataAdvisedSubmaneuver subm;
    subm.submaneuverID = ManeuverID::Accelerate; // Always accelerate for RVAhead

    mcData::mcDataForeseeIndication foresee;
    foresee.longitudinalAccelerationValue = acc_rv_ahead;
    foresee.longitudinalAccelerationConfidence.setData(AccelerationConfidence::AccelerationConfidence_unavailable);
    foresee.duration = time_rv_ahead;
    mcmData.setForeseeIndicationRVAhead(foresee);
    adv.submaneuvers.push_back(subm);
    parsed_advices.push_back(adv);
  }

  mcData::mcAdviceContainer advice_container{};
  advice_container.advices = parsed_advices;
  mcmData.setAdviceContainer(advice_container);

  if(RV_id >= 0) m_acceptance_map[RV_id] = {false, dec_rv, time_rv};
  if(RVAhead_id >= 0) m_acceptance_map[RVAhead_id] = {false, acc_rv_ahead, time_rv_ahead};
  m_coordinator = true;
  m_busy_with_maneuver = true;
  // Set constant speed for the negotiation time for HV
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
  m_traci->vehicle.setAcceleration(m_vehicle_id, 0, 1);
  double value = m_dist(m_gen);
  EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
  m_tx_mcm_event = e;
  m_left_criterion = left_criterion;
  m_target_lane = target_lane;
  m_negotiation_event = Simulator::Schedule(MilliSeconds(m_negotiation_time), &foresee::negotiationPhase, this, left_criterion, target_lane);
}

void foresee::negotiationPhase(bool left_criterion, int target_lane) {
  try {
    if (!m_coordinator || !m_busy_with_maneuver) {
      // State was cleared externally (e.g. cancellation request received between
      // startCoordination and this firing). FORESEE must be rescheduled here
      // only if no other handler already did it (i.e. we are still not busy).
      if (!m_busy_with_maneuver) ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
      return;
    }
    bool its_ok = true;
    std::string veh_refused;
    DeclineReason_t reason;
    std::vector<double> times;
    std::vector<std::string> coordinators;
    std::vector<double> accelerations;
    for (auto it : m_acceptance_map) {
        // Found an actor that doesn't want to coordinate
        if(!it.second.accepted) {its_ok = false; veh_refused = "veh" + std::to_string(it.first); reason = it.second.decline_reason; break;}
        times.push_back(it.second.time);
        coordinators.push_back("veh" + std::to_string(it.first));
        accelerations.push_back(it.second.acceleration);
        // Highlight vehicles involved
      }
    if(its_ok) {
        if (m_verbose) {
          double now_s = Simulator::Now().GetSeconds();
          std::cout << "\n[NEGOTIATION OK " << now_s << "]" << std::endl;
          std::cout << "Vehicle " << m_vehicle_id << " will change lane from " << m_traci->vehicle.getLaneIndex(m_vehicle_id) << " to " << target_lane << std::endl;
          int counter = 0;
          for (auto it = coordinators.begin(); it != coordinators.end(); ++it) {
            double speed = m_traci->vehicle.getSpeed(*it);
            double final_speed = speed + times[counter] / 1e3 * accelerations[counter];
            std::cout << "Vehicle " << *it << " is going at " << speed << " and after " << times[counter] / 1e3 << " seconds will be at " << final_speed << std::endl;
            counter ++;
          }
        }
        
        // Start the maneuver
        // First send the ACK to the others
        mcData mcmData;
        mcData::mcBasicContainer mcBasicContainer{};
        mcBasicContainer.itsRole = McmItssRole_coordinatingItss; // HV is the coordinator
        mcBasicContainer.concept = 0; // MCM Goal will be set
        mcBasicContainer.cost = 0;
        mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
        mcBasicContainer.mcmType = McmType::McmType_acknowledgment; // HV asks the others for a cooperation
        mcBasicContainer.maneuverID = left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane;
        mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
        mcmData.setBasicContainer(mcBasicContainer);
        mcData::mcAcknowledgeContainer ack{};
        mcmData.setAcknowledgmentContainer(ack);
        double value = m_dist(m_gen);
        EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
        m_tx_mcm_event = e;
        double time_to_wait = *std::max_element(times.begin(), times.end());
        // HV should keep constant speed until the time to change lane
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
        m_traci->vehicle.setAcceleration(m_vehicle_id, 0, time_to_wait / 1e3);
        
        // Lambda function to schedule the lane change after the coordinaiton
        std::shared_ptr<bool> alive = m_alive;
        m_change_lane_event = Simulator::Schedule(
          MilliSeconds(time_to_wait), 
          [this, alive, target_lane, coordinators]() {
            if (!*alive) return;  
            if (!m_busy_with_maneuver) {
                // Coordination was cancelled mid-wait, schedule again FORESEE
                ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
                return;
              }
              if (m_verbose) {
                double now_s = Simulator::Now().GetSeconds();
                std::cout << "\n[LANE CHANGE TIME " << now_s << "]" << std::endl;
                std::cout << "Vehicle " << m_vehicle_id << " is trying to coordinate" << std::endl;
                for (auto it = coordinators.begin(); it != coordinators.end(); ++it)
                {
                  double speed = m_traci->vehicle.getSpeed(*it);
                  std::cout << "Vehicle " << *it << " is going at " << speed << " after coordination" << std::endl;
                }
              }
              bool can_change = true;
              std::string veh_blocking;
              double my_heading = m_vdp->getHeadingValue();
              double my_x = m_vdp->getPositionXY().x;
              double my_speed = m_vdp->getSpeedValue();
              std::vector<LDM::returnedVehicleData_t> vehicles;
              m_LDM->getAllCVs (vehicles);
              // Check whether the comfort criteria is now respected for all the participants
              for (auto it = coordinators.begin(); it != coordinators.end(); ++it) {
                auto item = std::find_if(vehicles.begin(), vehicles.end(), 
                [&](const LDM::returnedVehicleData_t& veh){
                        return veh.vehData.stationID == std::stol((*it).substr(3));
                  }
                );

                if (item != vehicles.end()) {
                  // Check the position of HV w.r.t. the coordinator, to check which one is the leader and which is the follower
                  bool behind = false;
                  libsumo::TraCIPosition pos = m_traci->simulation.convertLonLattoXY (item->vehData.lon, item->vehData.lat);
                  double item_x = pos.x;
                  if ((std::abs(my_heading - 90) < DOUBLE_TOLERANCE && item_x <= my_x) || (std::abs(my_heading - 270) < DOUBLE_TOLERANCE && item_x >= my_x)) behind = true;
                  double a;
                  double gap = std::abs(my_x - item_x);
                  if (behind) {
                    // Coordinator is the vehicle ahead, so the vehicle in analysis is the RV
                    double leader_length = m_vdp->getVehicleLength();
                    if (gap > leader_length) gap -= leader_length;
                    else gap = 0;
                    IDMParams params = getIDMParams(static_cast<StationType> (item->vehData.stationType), item->vehData.desired_speed.getData());
                    a = idmAcceleration(item->vehData.speed_ms, my_speed, gap,
                                                        params.v0, params.T,
                                                        params.s0, params.a, params.b, params.d);
                  }
                  else {
                    // Coordinator is the vehicle behind, so the vehicle in analysis is the RVAhead
                    double leader_length = (double) item->vehData.vehicleLength.getData() / DECI;
                    if (gap > leader_length) gap -= leader_length;
                    else gap = 0;
                    IDMParams params = getIDMParams(static_cast<StationType> (m_station_type), m_desired_speed);
                    
                    a = idmAcceleration(my_speed, item->vehData.speed_ms, gap,
                                                        params.v0, params.T,
                                                        params.s0, params.a, params.b, params.d);
                  }
                  
                  if(a < MIN_DECELERATION) {
                    // Comfort criterion if not reached for this item, coordination failed
                    can_change = false;
                    veh_blocking = behind ? "RV" : "RVAhead";
                    break;
                  }
                }
              }
              if (can_change) {
                // For all the coordinators the comfort has been reached, we can coordinate
                std::string edge_id = m_traci->vehicle.getRoadID(m_vehicle_id);
                double pos = m_traci->vehicle.getLanePosition(m_vehicle_id);
                std::string target_lane_id = edge_id + "_" + std::to_string(target_lane);
                m_traci->vehicle.moveTo(m_vehicle_id, target_lane_id, pos);
                if (m_register_log) {
                  m_coordination_structure.execution_success = 2;
                  m_coordination_log.push_back(m_coordination_structure);
                  m_coordination_counter ++;
                }
                Simulator::Schedule(MilliSeconds(150), &foresee::checkLane, this, target_lane);
                ScheduleNextCheck(MilliSeconds(2 * m_FORESEE_check_ms));
              }
              else {
                if (m_verbose) {
                  double now_s = Simulator::Now().GetSeconds();
                  std::cout << "\n[COORDINATION FAILED " << now_s << "]" << std::endl;
                  std::cout << "Vehicle " << m_vehicle_id << " cannot complete the coordination due to comfort criteria not reached by " << veh_blocking << std::endl;
                }
                if (m_register_log) {
                  m_coordination_structure.execution_success = 0;
                  m_coordination_log.push_back(m_coordination_structure);
                  m_coordination_counter ++;
                }
                // Failed: retry sooner
                ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
              }
              
              // Send termination to free all targets
              mcData mcmData;
              mcData::mcBasicContainer mcBasicContainer{};
              mcBasicContainer.itsRole = McmItssRole_coordinatingItss; // HV is the coordinator
              mcBasicContainer.concept = 0; // MCM Goal will be set
              mcBasicContainer.cost = 0;
              mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
              mcBasicContainer.mcmType = McmType::McmType_termination; // HV asks the others for a cooperation
              mcBasicContainer.maneuverID = ManeuverID::Undefined;
              mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
              mcmData.setBasicContainer(mcBasicContainer);
              mcData::mcTerminationContainer term{};
              mcmData.setTerminationContainer(term);
              double value = m_dist(m_gen);
              EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
              m_tx_mcm_event = e;

              m_coordinator = false;
              m_busy_with_maneuver = false;
              m_acceptance_map.clear();
              m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
          }
        );
      } else {
        if (m_verbose) {
          double now_s = Simulator::Now().GetSeconds();
          if (reason == DeclineReason_agreementSeekingUnwanted) {
            std::cout << "\n[NEGOTIATION FAILED " << now_s << "]" << std::endl;
            std::cout << "Vehicle " << m_vehicle_id << " will not change lane due to a refusal from " << veh_refused << " (not convenient)" << std::endl;
          }
          else if (reason == DeclineReason_unableToConform) {
            std::cout << "\n[NEGOTIATION FAILED " << now_s << "]" << std::endl;
            std::cout << "Vehicle " << m_vehicle_id << " will not change lane due to a refusal from " << veh_refused << " (busy with another maneuver)" << std::endl;
          }
          else  {
            std::cout << "\n[NEGOTIATION FAILED " << now_s << "]" << std::endl;
            std::cout << "Vehicle " << m_vehicle_id << " will not change lane due to a refusal from " << veh_refused << " (probably lost message)" << std::endl;
          }
        }
        if (m_register_log) {
          if (reason == DeclineReason_agreementSeekingUnwanted) {
            // Register only refusals due to acceleration/deceleration not convenient, not the other ones
            m_coordination_structure.execution_success = 1;
            m_coordination_log.push_back(m_coordination_structure);
          }
        }
        // Cancel the maneuver
        mcData mcmData;
        mcData::mcBasicContainer mcBasicContainer{};
        mcBasicContainer.itsRole = McmItssRole_coordinatingItss; // HV is the coordinator
        mcBasicContainer.concept = 0; // MCM Goal will be set
        mcBasicContainer.cost = 0;
        mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
        mcBasicContainer.mcmType = McmType::McmType_termination; // HV asks the others for a cooperation
        mcBasicContainer.maneuverID = left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane;
        mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
        mcmData.setBasicContainer(mcBasicContainer);
        mcData::mcTerminationContainer term{};
        mcmData.setTerminationContainer(term);
        double value = m_dist(m_gen);
        EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
        m_tx_mcm_event = e;
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
        m_busy_with_maneuver = false;
        m_coordinator = false;
        m_acceptance_map.clear();
        // Schedule again FORESEE in 10s
        ScheduleNextCheck(MilliSeconds(2 * m_FORESEE_check_ms));
      }
  } catch (...) {
    std::cout << "Error 404 FORESEE not usable for vehicle " << m_vehicle_id << " anymore!" << std::endl;
  }
  
}

void foresee::targetCheckACK() {
  if (!*m_alive) return;
  // Guard: if we are no longer in a coordination or already responded, do nothing.
  // Safety note: if !m_busy_with_maneuver, the termination handler has already
  // rescheduled FORESEE, so we must NOT reschedule again here to avoid double-firing.
  // If m_busy_with_maneuver is still true but m_my_coordinator == -1 or already
  // responded, something inconsistent happened — reschedule defensively.
  if (m_my_coordinator == -1 || m_my_coordinator_responded) {
    if (m_busy_with_maneuver) {
      // Inconsistent state: coordinator cleared but still marked busy.
      // Free the vehicle and reschedule to avoid a silent stall.
      m_busy_with_maneuver = false;
      m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
      ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
    }
    // else: termination handler already rescheduled, nothing to do.
    return;
  }
  if (!m_busy_with_maneuver) {
    // Termination arrived just before this callback fired.
    // FORESEE already rescheduled by termination handler.
    return;
  }
  mcData mcmData;
  mcData::mcBasicContainer mcBasicContainer{};
  mcBasicContainer.itsRole = McmItssRole_targetVehicle;
  mcBasicContainer.concept = 0; // MCM Goal will be set
  mcBasicContainer.cost = 0;
  mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
  mcBasicContainer.mcmType = McmType::McmType_termination; // HV asks the others for a cooperation
  mcBasicContainer.maneuverID = ManeuverID::Undefined;
  mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
  mcmData.setBasicContainer(mcBasicContainer);
  mcData::mcTerminationContainer term{};
  mcmData.setTerminationContainer(term);
  double value = m_dist(m_gen);
  EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
  m_tx_mcm_event = e;
  m_my_coordinator = -1;
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
  m_my_coordinator_responded = false;
  m_busy_with_maneuver = false;
  if (m_verbose) {
    double now_s = Simulator::Now().GetSeconds();
    std::cout << "\n[MANEUVER REFUSED " << now_s << "]" << std::endl;
    std::cout << "Vehicle " << m_vehicle_id << " is refusing the maneuver due to ACK not received " << std::endl;
  }
  ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
}

void foresee::receiveMCM(const asn1cpp::Seq<MCM>& mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info, mcData::mcDataForeseeIndication foresee_indication_rv, mcData::mcDataForeseeIndication foresee_indication_rvahead) {
  StationId_t sender = mcm->header.stationId;
  long now = compute_timestampIts (m_real_time) % 65536;
  mcm->payload.basicContainer.generationDeltaTime;
  McmType_t type = mcm->payload.basicContainer.mcmType;
  McmContainer_PR present_container = mcm->payload.mcmContainer.present;
  McmItssRole_t sender_role = mcm->payload.basicContainer.itssRole;

  // Container extraction
  bool no_containers = false;
  bool advice_container = false;
  ManoeuvreAdviceContainer_t adc;
  bool maneuver_container = false;
  VehicleManoeuvreContainer_t man;
  bool ack_container = false;
  AcknowledgmentContainer_t ack;
  bool resp_container = false;
  ResponseContainer_t resp;
  bool term_container = false;
  TerminationContainer_t term;

  switch (present_container) {
      case McmContainer_PR_NOTHING:
        no_containers = true;
        break;
      case McmContainer_PR_advisedManoeuvreContainer:
        adc = mcm->payload.mcmContainer.choice.advisedManoeuvreContainer;
        advice_container = true;
        break;
      case McmContainer_PR_vehicleManoeuvreContainer:
        man = mcm->payload.mcmContainer.choice.vehicleManoeuvreContainer;
        maneuver_container = true;
        break;
      case McmContainer_PR_acknowledgmentContainer:
        ack = mcm->payload.mcmContainer.choice.acknowledgmentContainer;
        ack_container = true;
        break;
      case McmContainer_PR_responseContainer:
        resp = mcm->payload.mcmContainer.choice.responseContainer;
        resp_container = true;
        break;
      case McmContainer_PR_terminationContainer:
        term = mcm->payload.mcmContainer.choice.terminationContainer;
        term_container = true;
        break;
      default:
        break;
    }

  if (no_containers) {
      // If no container present, directly return
      return;
    }

  // MCM type identification
  // FORESEE takes into account: Request, Response, Acknowledgment, Execution Status, Terminator
  switch (type) {
    case McmType::McmType_request:
      // Accept the coordination if the request is for us
      if (advice_container) {
        long sub_id_for_response;
        bool accept = false;
        bool speed_exceeds_desired = false;
        bool msg_for_me = false;
        int my_id = std::stol(m_vehicle_id.substr(3));
        int adv_size = asn1cpp::sequenceof::getSize(adc);
        for(int i = 0; i < adv_size; ++i) {
            auto adv = adc.list.array[i];
            StationId_t id = adv->executantID;
            if (id == my_id) {
                msg_for_me = true;
                if (m_busy_with_maneuver) {
                    if (m_verbose) {
                        double now_s = Simulator::Now().GetSeconds();
                        std::cout << "\n[MANEUVER REFUSED " << now_s << "]" << std::endl;
                        std::cout << "Vehicle " << m_vehicle_id << " cannot accept maneuver from veh" << sender << " because it is busy with another" << std::endl;
                      }
                    break;
                  }
                else {
                    // In FORESEE we have just one submaneuver per each executant
                    auto subm = adv->submaneuvres.list.array[0];
                    auto sub_id = asn1cpp::getField(subm->submanoeuvreId, Identifier1B_t);
                    sub_id_for_response = sub_id;
                    double acc_value, time_s;
                    if (sub_id == ManeuverID::Accelerate) {
                      // I am the RVAhead in this situation, I need to take the acceleration
                      acc_value = foresee_indication_rvahead.longitudinalAccelerationValue;
                      time_s = foresee_indication_rvahead.duration;
                    } else if (sub_id == ManeuverID::Slowdown) {
                      acc_value = foresee_indication_rv.longitudinalAccelerationValue;
                      time_s = foresee_indication_rv.duration;
                    }
                    double current_speed = m_vdp->getSpeedValue();
                    double future_speed = current_speed + acc_value * time_s;
                    if ((future_speed - m_desired_speed) > DOUBLE_TOLERANCE) {
                      speed_exceeds_desired = true;
                      if (m_verbose) {
                        double now_s = Simulator::Now().GetSeconds();
                        std::cout << "\n[MANEUVER REFUSED " << now_s << "]" <<std::endl;
                        std::cout << "For vehicle " << m_vehicle_id << " the final speed would exceed the desired one: " << m_desired_speed << " < " << future_speed << std::endl;
                      }
                      break;
                    }
                    else if (m_vdp->getTravelledDistance() > MAX_DISTANCE_TRAVELED_TO_COORDINATE) {
                      // The vehicle that is requested to coordinate is too close to the end of the road segment
                      // It is not safe to coordinate, we refuse the coordination
                      if (m_verbose) {
                        double now_s = Simulator::Now().GetSeconds();
                        std::cout << "\n[MANEUVER REFUSED " << now_s << "]" <<std::endl;
                        std::cout << "Vehicle " << m_vehicle_id << " is too close to the end of the road segment" << std::endl;
                      }
                      break;
                    }
                    m_required_acceleration_time = std::make_tuple(acc_value, time_s);
                    accept = true;
                    break;
                  }
              }
            }
          if (accept && msg_for_me) {
              if (m_verbose) {
                double now_s = Simulator::Now().GetSeconds();
                std::cout << "\n[MANEUVER ACCEPTED " << now_s << "]" <<std::endl;
                std::cout << "For vehicle " << m_vehicle_id << " the maneuver requested by veh" << sender << " is feasible" << std::endl;
              }
              // Keep constant speed until the HV will start the maneuver
              m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
              m_traci->vehicle.setAcceleration(m_vehicle_id, 0, 1.5);
              // Accept the coordination
              mcData mcmData;
              mcData::mcBasicContainer mcBasicContainer{};
              mcBasicContainer.itsRole = McmItssRole_targetVehicle; // HV is the coordinator
              mcBasicContainer.concept = 0; // MCM Goal will be set
              mcBasicContainer.cost = 0;
              mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
              mcBasicContainer.mcmType = McmType::McmType_response; // HV asks the others for a cooperation
              mcBasicContainer.maneuverID = sub_id_for_response;
              mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
              mcmData.setBasicContainer(mcBasicContainer);
              mcData::mcResponseContainer resp{};
              resp.response = 0;
              resp.coordinator = sender;
              mcmData.setResponseContainer(resp);
              m_busy_with_maneuver = true;
              m_my_coordinator = sender;
              m_my_coordinator_responded = false;
              double value = m_dist(m_gen);
              EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
              m_tx_mcm_event = e;
              m_ack_event = Simulator::Schedule(MilliSeconds(m_negotiation_time + 200), &foresee::targetCheckACK, this);
            }
          else if (!accept && msg_for_me) {
              // Refuse the coordination
              mcData mcmData;
              mcData::mcBasicContainer mcBasicContainer{};
              mcBasicContainer.itsRole = McmItssRole_targetVehicle; // HV is the coordinator
              mcBasicContainer.concept = 0; // MCM Goal will be set
              mcBasicContainer.cost = 0;
              mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
              mcBasicContainer.mcmType = McmType::McmType_response; // HV asks the others for a cooperation
              mcBasicContainer.maneuverID = sub_id_for_response;
              mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
              mcmData.setBasicContainer(mcBasicContainer);
              mcData::mcResponseContainer resp{};
              resp.response = 1;
              if (speed_exceeds_desired) resp.declineReason.setData(DeclineReason_agreementSeekingUnwanted);
              else resp.declineReason.setData(DeclineReason_unableToConform);
              mcmData.setResponseContainer(resp);
              double value = m_dist(m_gen);
              EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
              m_tx_mcm_event = e;
            }
          else if (!msg_for_me) {
            // The message was not for me but I received it
            // If the direction is the same and the distance is within the coordination avoidance range, we populate the blocked list
            double latitude = (double) mcm->payload.basicContainer.position.latitude / DOT_ONE_MICRO;
            double longitude = (double) mcm->payload.basicContainer.position.longitude / DOT_ONE_MICRO;
            libsumo::TraCIPosition pos = m_traci->simulation.convertLonLattoXY(longitude, latitude);
            int direction = mcm->payload.basicContainer.direction;
            // Check if direction is the same
            double my_heading = m_vdp->getHeadingValue();
            bool same_direction = false;
            if ((std::abs(my_heading - 90) < DOUBLE_TOLERANCE && direction == 0) || (std::abs(my_heading - 270) < DOUBLE_TOLERANCE && direction == 1)) same_direction = true;
            if (same_direction) {
              double my_x = m_vdp->getPositionXY().x;
              bool behind = false;
              if ((direction == 0 && pos.x <= my_x) || (direction == 1 && pos.x >= my_x)) behind = true;
              if (behind) {
                // The HV that is cooperating id behind us, we need to check whether we are inside the CA Range
                double gap = std::abs(pos.x - my_x);
                // Check whether the coordination is at a reasonable distance (within the Coordination Avoidance range)
                if (gap <= m_ca_range) {
                  // Coordinator is behind ego and within CA range — register and mark accordingly
                  m_blocked_by_other_coordinations[sender] = {Simulator::Now().GetMilliSeconds(), {}};
                  int adv_size = asn1cpp::sequenceof::getSize(adc);
                  for (int i = 0; i < adv_size; ++i) {
                    auto adv = adc.list.array[i];
                    m_blocked_by_other_coordinations[sender].participants.emplace(adv->executantID);
                  }
                }
              }     
            }
          }
        }
      break;

    case McmType::McmType_response:
      if (resp_container) {
          // Checke whether I am currently a coordinator and I am waiting for the response of the sender
          if (m_coordinator && sender_role == McmItssRole::McmItssRole_targetVehicle && m_acceptance_map.find(sender) != m_acceptance_map.end()) {
              // Response received from the sender, but it could be for another coordinator
              // We need to ensure the response is for us
              StationID_t coordinator = asn1cpp::getField(resp.coordinatorID, StationId_t);
              ManouevreResponse_t response = resp.manouevreResponse;
              if (response == ManouevreResponse::ManouevreResponse_accept && coordinator == m_vehicle_id_int) {
                  // One of the vehicles accepted to be involved in the coordination
                  // Update the map
                  m_acceptance_map[sender].accepted = true;
                  if (m_verbose) {
                    double now_s = Simulator::Now().GetSeconds();
                    std::cout << "\n[ACCEPTANCE RECEIVED " << now_s << "]" << std::endl;
                    std::cout << "Vehicle " << m_vehicle_id << " has received the acceptance from veh" << sender << std::endl;
                  }
                  // Check if we have received all the acceptance, we can anticipate the negotiation end
                  bool received_all = true;
                  for (auto it = m_acceptance_map.begin(); it != m_acceptance_map.end(); ++it) {
                    if (!it->second.accepted) {received_all = false; break;}
                  }
                  // If we have already received all the acceptances requested, we can cancel the scheduled negotiation and instead start one immediately 
                  if (received_all) {
                    Simulator::Cancel(m_negotiation_event);
                    negotiationPhase(m_left_criterion, m_target_lane);
                  }
                }
              // Response is for us, but it is a refusal
              else if (response == ManouevreResponse::ManouevreResponse_decline && coordinator == m_vehicle_id_int) {
                // One of the vehicles declined the coordination
                // We need to check whether it has already accepted the maneuver and cancel its acceptance
                // THIS CASE HAS NOT BEEN IMPLEMENTED YET IN FORESEE --> added for completeness
                DeclineReason_t reason = asn1cpp::getField(resp.declineReason, DeclineReason_t);
                m_acceptance_map[sender].accepted = false;
                m_acceptance_map[sender].decline_reason = reason;
                if (m_verbose) {
                  double now_s = Simulator::Now().GetSeconds();
                  std::cout << "\n[REFUSAL RECEIVED " << now_s << "]" << std::endl;
                  std::cout << "Vehicle " << m_vehicle_id << " has received the refusal from veh" << sender << std::endl;
                }
                // One of the vehicles refused, the maneuver should be suppressed, we can directly call the negotiation phase (which will fail) 
                // Simulator::Cancel(m_negotiation_event);
                // negotiationPhase(m_left_criterion, m_target_lane);
              }
              // Else: the sender is replying to another coordinator
            }
        }
      break;

    case McmType::McmType_termination:
      // Termination reception, the vehicle is now free to do other coordinations
      if (sender_role == McmItssRole_coordinatingItss && sender != m_my_coordinator && m_blocked_by_other_coordinations.find(sender) != m_blocked_by_other_coordinations.end()) {
        // A maneuver is terminated, erase it from the set if it is not my maneuver
        // Must be the coordinator to stop the menauver
        m_blocked_by_other_coordinations.erase(sender);
      }
      else if (sender_role == McmItssRole_coordinatingItss && sender == m_my_coordinator) {
        // If it is my maneuver, now I am free for others
        // We can cancel the check for the ACK
        Simulator::Cancel(m_ack_event);
        Simulator::Cancel(m_continue_constant_speed_event);
        Simulator::Cancel(m_tx_mcm_event);
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
        m_busy_with_maneuver = false;
        m_my_coordinator_responded = false;
        m_my_coordinator = -1;
        // Schedule again FORESEE
        ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
      }
      break;

    case McmType::McmType_acknowledgment:
      if (sender_role == McmItssRole_coordinatingItss && sender == m_my_coordinator) {
        // We can start the maneuver
        m_my_coordinator_responded = true;
        // We can cancel the check for the ACK
        Simulator::Cancel(m_ack_event);
        // Execute the acceleration/deceleration
        executeManeuver();
      }
      break;

    case McmType::McmType_cancellationRequest:
      // One target asked for cancellation, approve
      if (m_coordinator && m_acceptance_map.find(sender) != m_acceptance_map.end()) {
        // Cancel the other events if any
        Simulator::Cancel(m_negotiation_event);
        Simulator::Cancel(m_change_lane_event);
        Simulator::Cancel(m_tx_mcm_event);
        mcData mcmData;
        mcData::mcBasicContainer mcBasicContainer{};
        mcBasicContainer.itsRole = McmItssRole_coordinatingItss;
        mcBasicContainer.concept = 0; // MCM Goal will be set
        mcBasicContainer.cost = 0;
        mcBasicContainer.goal = ManoeuvreCooperationGoal_localTrafficManagement; // FORESEE manages local traffic interactions
        mcBasicContainer.mcmType = McmType::McmType_termination; // HV asks the others for a cooperation
        mcBasicContainer.maneuverID = ManeuverID::Undefined;
        mcBasicContainer.stationType = m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon;
        mcmData.setBasicContainer(mcBasicContainer);
        mcData::mcTerminationContainer term{};
        mcmData.setTerminationContainer(term);
        double value = m_dist(m_gen);
        EventId e = Simulator::Schedule(NanoSeconds(value * 1e6), &txMCM, m_mcs_ptr, mcmData);
        m_tx_mcm_event = e;
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
        m_coordinator = false;
        m_busy_with_maneuver = false;
        m_acceptance_map.clear();
        // Schedule again FORESEE
        ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
      }
      break;
    default:
      break;
    }
}

void foresee::executeManeuver() {
  double acc = std::get<0>(m_required_acceleration_time);
  double duration = std::get<1>(m_required_acceleration_time);
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
  m_traci->vehicle.setAcceleration(m_vehicle_id, acc, duration);
  m_continue_constant_speed_event = Simulator::Schedule(MilliSeconds(duration*1e3), &foresee::continueWithConstantSpeed, this, m_my_coordinator);
}

void foresee::continueWithConstantSpeed(StationId_t coordinator) {
  if (!*m_alive) return;
  // Check if the maneuver has been finished thanks between the old coordinator and the current one
  if(coordinator == m_my_coordinator) {
    // The termination has not been received yet from the coordinator
    // The vehicle continues with constant speed
    if (m_verbose) {
      double now_s = Simulator::Now().GetSeconds();
      std::cout << "\n[TERMINATION NOT YET RECEIVED " << now_s << "]" << std::endl;
      std::cout << "Vehicle " << m_vehicle_id << " will continue with constant speed" << std::endl;
    }
    m_traci->vehicle.setAcceleration(m_vehicle_id, 0, m_maneuver_horizon / 1e3);
    // Watchdog: if termination still not received after the horizon, free ourselves
    m_watchdog_event = Simulator::Schedule(MilliSeconds(m_maneuver_horizon + 500), &foresee::targetWatchdog, this, coordinator);
  }
  else {
    // The termination has already been received, we are in normal speed mode, so we can avoid to set a specific speed
    if (m_verbose) {
      double now_s = Simulator::Now().GetSeconds();
      std::cout << "\n[TERMINATION ALREADY RECEIVED " << now_s << "]" << std::endl;
      std::cout << "Vehicle " << m_vehicle_id << " will not change its current motion" << std::endl;
    }
  }
}

void foresee::targetWatchdog(StationId_t coordinator) {
  if (!*m_alive) return;
  if (m_my_coordinator == coordinator && m_busy_with_maneuver) {
    // Termination was never received — free the vehicle
    m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
    m_busy_with_maneuver = false;
    m_my_coordinator = -1;
    m_my_coordinator_responded = false;
    ScheduleNextCheck(MilliSeconds(m_FORESEE_check_ms));
  }
  // else: termination already arrived cleanly, nothing to do
}

void foresee::checkLane(int target_lane_id) {
  if (!*m_alive) return;
  if (m_verbose) {
    int new_lane = m_traci->vehicle.getLaneIndex(m_vehicle_id);
    double now_s = Simulator::Now().GetSeconds();
    if (new_lane == target_lane_id) {
      std::cout << "\n[COORDINATION SUCCESS " << now_s << "]" << std::endl;
      std::cout << "Vehicle " << m_vehicle_id << " changed lane" << std::endl;
    }
    else {
      std::cout << "\n[COORDINATION FAILED " << now_s << "]" << std::endl;
      std::cout << "Vehicle " << m_vehicle_id << " didn't change lane" << std::endl;
    }
  }
}

void foresee::cleanupBlockedCoordinations() {
  if (!*m_alive) return;
  if (m_vdp->getTravelledDistance() > MAX_DISTANCE_TRAVELED_TO_COORDINATE)
    return; // Vehicle is done, stop the periodic cleanup
  long now_ms = Simulator::Now().GetMilliSeconds();
  for (auto it = m_blocked_by_other_coordinations.begin(); it != m_blocked_by_other_coordinations.end();) {
    if (now_ms - it->second.time >= m_coordination_timeout_ms)
      it = m_blocked_by_other_coordinations.erase(it);
    else
      ++it;
  }
  m_cleanup_event = Simulator::Schedule(MilliSeconds(m_cleanup_ms), &foresee::cleanupBlockedCoordinations, this);
}

void foresee::ScheduleNextCheck(Time delay) {
  // Check if an event is already scheduled and still 'alive'
  if (!m_foresee_event.IsExpired()) {
      Simulator::Cancel(m_foresee_event);
    }
  
  // Schedule the next execution
  m_foresee_event = Simulator::Schedule(delay, &foresee::FORESEEMobilityModel, this);
}

 void foresee::deleteEvents() {
  *m_alive = false;
  Simulator::Cancel(m_foresee_event);
  Simulator::Cancel(m_negotiation_event);
  Simulator::Cancel(m_change_lane_event);
  Simulator::Cancel(m_continue_constant_speed_event);
  Simulator::Cancel(m_tx_mcm_event);
  Simulator::Cancel(m_cleanup_event);
  Simulator::Cancel(m_watchdog_event);
 }

}

