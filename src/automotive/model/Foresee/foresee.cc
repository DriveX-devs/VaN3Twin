//
// Created by diego on 01/12/25.
//

#include "foresee.h"
#include "ns3/ForeseeIndication.h"
#include "ns3/LongitudinalAcceleration.h"
#include "ns3/asn_utils.h"
#include "ns3/assert.h"
#include "ns3/caBasicService_v1.h"
#include "ns3/foresee.h"
#include "ns3/mcBasicService.h"
#include "ns3/nstime.h"
#include "ns3/signalInfoUtils.h"
#include "ns3/simulator.h"
#include "ns3/sumo-TraCIDefs.h"
#include "ns3/vdp.h"
#include <cassert>
#include <cstddef>
#include <random>
#include <string>
#include <tuple>

#define DOUBLE_TOLERANCE 0.5
#define SEED 42

std::uniform_real_distribution<double> dist(1, 5);
std::mt19937 gen (SEED);

/*
TODO List:
- Spread MCMs during the execution phase (?)
- Collect data for the csv
*/

namespace ns3
{

void txMCM(Ptr<MCBasicService> mc, Ptr<MCSpecification> specs)
{
  mc->generateAndEncodeMCM(specs);
}

foresee::IDMParams foresee::getIDMParams(StationType type) {
  if (type == StationType::StationType_passengerCar)
  {
    return {m_desired_speed, 0.8, 2.0, 1.5, 2.0, 1.5};
  } 
  else if (type == StationType::StationType_lightTruck)
  {
    return {m_desired_speed, 1.0, 2.0, 1.5, 2.0, 2.0};
  }
  else
  {
    return {m_desired_speed, 1.0, 2.0, 1.5, 2.0, 2.0};
  }
}

double foresee::idmAcceleration(double v, double v_lead, double gap, double v0,
                          double T, double s0, double a, double b) {
  // Desired minimum gap
  double s_star = s0 + std::max(0.0, v * T + (v * (v - v_lead)) / (2.0 * std::sqrt(a * b)));
  // IDM acceleration
  return a * (1.0 - std::pow(v / v0, 4) - std::pow(s_star / std::max(gap, 0.1), 2));
}

std::tuple<double, double> foresee::computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on acceleration of leader in [0, a_max]
  double lo = 0.0;
  double hi = p.a; // max acceleration
  double delta_t = -1;
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double a_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      double t = 0;
      for(; t <= horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
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
      if(a_f_final >= MIN_DECELERATION)
        {
          hi = a_candidate; // enough, try less
        }
      else
        {
          lo = a_candidate; // not enough, need more
        }
    }
  if(std::abs(hi - p.a) < 0.01)
    return {NO_SOLUTION, -1};
  if (hi < 0.1) hi = 0.0;
  return {hi, delta_t}; // minimum acceleration RVAhead needs to apply
}

std::tuple<double, double> foresee::computeRequiredDeceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on deceleration of follower in [0, a_max]
  double lo = -p.d;
  double hi = 0.0; // max deceleration
  double delta_t = -1;
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double d_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      double t = 0;
      for(; t <= horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
          if (a_f >= MIN_DECELERATION)
            {
              a_f_final = a_f;
              delta_t = t;
              break;
            }
          // Follower decelerates with candidate deceleration
          v_f = v_f + d_candidate * dt;
          // Update gap
          gap += (v_l - v_f) * dt;
        }
      // Check if at end of horizon ego can merge comfortably
      if(a_f_final >= MIN_DECELERATION)
        {
          lo = d_candidate; // enough, try less deceleration (less negative)
        }
      else
        {
          hi = d_candidate; // not enough, need more deceleration (more negative)
        }
    }
  if(std::abs(lo) - p.d < 0.01)
    return {NO_SOLUTION, -1};
  if (lo < 0.1) lo = 0.0;
  return {lo, delta_t}; // minimum deceleration
}

void
foresee::addMCMRxCallback ()
{
  std::function<void(const asn1cpp::Seq<MCM>&, Address, StationID_t, StationType_t, SignalInfo)> rx_callback =
      std::bind(&foresee::receiveMCM,
                 this,
                 std::placeholders::_1,
                 std::placeholders::_2,
                 std::placeholders::_3,
                 std::placeholders::_4,
                 std::placeholders::_5);
  m_MCMReceiveCallbackExtended = rx_callback;
  m_mcs_ptr->addMCRxCallbackExtended (m_MCMReceiveCallbackExtended);
}

void
foresee::WrapperFORESEEMobilityModel()
{
  // Check if the number of lanes is valid
  if (m_num_lanes <= 0)
    {
      NS_FATAL_ERROR ("Set a number of lanes greater than 0 to use FORESEE Mobility Model.");
    }
  // Check if the LDM (Local Dynamic Map) is set
  if (m_LDM == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the LDM of the vehicle.");
    }
  // Check if TraCI (Traffic Control Interface) is set
  if (m_traci == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs TraCI.");
    }
  // Check if the desired speed is valid
  if (m_desired_speed <= 0)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs a Desired Speed greater than 0.");
    }
  // Check if the VDP (Vehicle Data Provider) is set
  if (m_vdp == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the VDP of the vehicle.");
    }
  // Check if the MCM (Maneuver Coordination Message) service is set
  if (m_mcs_ptr == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the MCM Basic Service of the vehicle.");
    }
  /*
  // Check for predictor
  if (m_prediction_type == UNKNOWN)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the prediction type.");
    }
  */
  if (m_node == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the pointer of the vehicle node.");
    }
  if (m_station_type != StationType_passengerCar && m_station_type != StationType_lightTruck)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the station type ['StationType_passengerCar', 'StationType_lightTruck'].");
    }
  if (m_MCMReceiveCallbackExtended == nullptr)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the callback for MCM.");
    }
  // Schedule the FORESEE Mobility Model to start at the specified time
  Simulator::Schedule (MilliSeconds(m_start_time), &foresee::FORESEEMobilityModel, this);
}

void
foresee::setNumberOfLanes ()
{
  // Retrieve the number of lanes for the current road using TraCI
  int lanes = m_traci->TraCIAPI::edge.getLaneNumber (m_traci->TraCIAPI::vehicle.getRoadID (m_vehicle_id));
  m_num_lanes = lanes;
}

void
foresee::FORESEEMobilityModel ()
{
  // Retrieve all connected vehicles (CVs) from the LDM
  std::vector<LDM::returnedVehicleData_t> vehicles;
  bool res = m_LDM->getAllCVs (vehicles);
  if (res == false)
    {
      // The route is empty (no perceived vehicles in the LDM)
      // FORESEE cannot be activated in this case, so we reschedule it
      Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
      return;
    }
  if (m_busy_with_maneuver)
    {
      // FORESEE cannot be activated in this case because we are involved in another maneuver
      Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
      return;
    }
  // Cleanup the blocked datasett in case there are old coordinations registered (e.g., the coordinator is out of range now)
  long now_ms = Simulator::Now().GetMilliSeconds();
  for (auto it = m_blocked_by_other_coordinations.begin(); it != m_blocked_by_other_coordinations.end();)
  {
      if (now_ms - it->second >= m_coordination_timeout_ms)
          it = m_blocked_by_other_coordinations.erase(it);
      else
          ++it;
  }
  // Check whether there is another maneuver coordination that is happening within the ahead range
  // If yes, ego vehicle cannot perform maneuver coordination
  bool found_coordination = m_blocked_by_other_coordinations.empty() ? false : true;
  if (found_coordination)
  {
    // FORESEE cannot be activated in this case because the CA range is not free
    Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
    return;
  }
  
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
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
      // Skip vehicles in different directions
      if ((it->vehData.heading - my_heading) > DOUBLE_TOLERANCE) continue;
      auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
      // Take the back bumper position
      double x = pos.x;
      // Skip vehicles behind the ego vehicle
      if (std::abs(my_heading - 90) < DOUBLE_TOLERANCE && x < my_x) continue;
      if (std::abs(my_heading - 270) < DOUBLE_TOLERANCE && x > my_x) continue;
      OptionalDataItem<long> lane = it->vehData.lanePosition;
      if (lane.isAvailable())
        {
          // Store vehicle speed and ID in the corresponding lane
          speeds_per_lane[lane.getData()].push_back (it->vehData.speed_ms);
          veh_per_lane[lane.getData()].push_back (std::to_string (it->vehData.stationID));
        }
    }

  // Determine lane change possibilities and criteria
  bool right_has_veh = false, left_has_veh = false;
  bool can_turn_right = false, can_turn_left = false;
  if (my_lane.getData() == 1)
    {
      // Ego is in the leftmost lane, can only turn right
      right_has_veh = !speeds_per_lane[my_lane.getData()+1].empty();
      can_turn_right = true;
      can_turn_left = false;
    }
  else if (my_lane.getData() == m_num_lanes)
    {
      // Ego is in the rightmost lane, can only turn left
      left_has_veh = !speeds_per_lane[my_lane.getData()-1].empty();
      can_turn_left = true;
      can_turn_right = false;
    }
  else
    {
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
  if (can_turn_left && left_has_veh)
    {
      min_speed_left = *std::min_element(speeds_per_lane[my_lane.getData()-1].begin(), speeds_per_lane[my_lane.getData()-1].end());
      if (std::abs(min_speed_left - min_speed_mine) > m_delta_ls)
        {
          if (min_speed_left > min_speed_mine)
            {
              left_criterion = true;
            }
          else
            {
              double DSth_left = min_speed_mine * (1 - m_offset);
              if (m_desired_speed > DSth_left + m_delta_ds)
                {
                  left_criterion = true;
                }
            }
        }
    }

  // Check right lane change incentive criterion
  if (can_turn_right && right_has_veh)
    {
      min_speed_right = *std::min_element(speeds_per_lane[my_lane.getData()+1].begin(), speeds_per_lane[my_lane.getData()+1].end());
      if (std::abs(min_speed_right - min_speed_mine) > m_delta_ls)
        {
          if (min_speed_right > min_speed_mine)
            {
              right_criterion = true;
            }
          else
            {
              double DSth_right = min_speed_right * (1 - m_offset);
              if (m_desired_speed < DSth_right - m_delta_ds)
                {
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
  if (lc_direction != 0)
    // At least one incentive criterion is satisfied
    // Check the comfort criterion
    {
      // Take the four roles, target, ahead ego, ahead target
      std::string RV, HVAhead, RVAhead;
      long RV_id = -1, RVAhead_id = -1;
      StationType RV_type, RVAhead_type;
      // Check the comfort criterion
      double x_RV, x_RVAhead;
      double y_RV, y_RVAhead;
      double speed_RV, speed_RVAhead;
      double min_dist_rv_ahead = 10000;
      double min_dist_rv = 10000;
      // Target lane
      int target_lane = left_criterion ? my_lane.getData() - 1 : my_lane.getData() + 1;
      // Vehicles ahead of HV in the target lane
      auto& vec1 = veh_per_lane[target_lane];
      for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
        {
          if ((it->vehData.heading - my_heading) > DOUBLE_TOLERANCE) continue;
          if (it->vehData.lanePosition.getData() == target_lane)
            {
              auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
              auto it_found = std::find (vec1.begin (), vec1.end (),
                                          std::to_string (it->vehData.stationID));
              if (it_found != vec1.end ())
                {
                  // Vehicle is in the target lane ahead of ego, can be RVAhead
                  // Transform the distance into a front-back bumper distance
                  double leader_length = (double) it->vehData.vehicleLength.getData() / DECI;
                  double dist = std::abs(my_x - pos.x);
                  if (leader_length < dist) dist = dist - leader_length;
                  else dist = 0;
                  if (dist < min_dist_rv_ahead && dist < MAX_DIST_AHEAD_BEHIND)
                    {
                      min_dist_rv_ahead = dist;
                      RVAhead = "veh" + std::to_string (it->vehData.stationID);
                      RVAhead_id = it->vehData.stationID;
                      x_RVAhead = pos.x;
                      y_RVAhead = pos.y;
                      speed_RVAhead = it->vehData.speed_ms;
                    }
                }
              else
                {
                  // Can be RV
                  // Transform the distance into a front-back bumper distance
                  double leader_length = m_vdp->getVehicleLength();
                  double dist = std::abs(my_x - pos.x);
                  if (leader_length < dist) dist = dist - leader_length;
                  else dist = 0;
                  OptionalDataItem<long> lane = it->vehData.lanePosition;
                  if (lane.isAvailable () && lane.getData () == target_lane)
                    {
                      if (dist < min_dist_rv && dist < MAX_DIST_AHEAD_BEHIND)
                        {
                          min_dist_rv = dist;
                          RV = "veh" + std::to_string (it->vehData.stationID);
                          RV_id = it->vehData.stationID;
                          x_RV = pos.x;
                          y_RV = pos.y;
                          speed_RV = it->vehData.speed_ms;
                          RV_type = static_cast<StationType> (it->vehData.stationType);
                        }
                    }
                }
            }
        }

      if (RV_id <= 0 && RVAhead_id <= 0)
        {
          // No RV and RVAhead present, we can directly change the lane
          target_lane = 3 - target_lane;
          // m_traci->vehicle.changeLane (m_vehicle_id, target_lane, m_time_to_lc);
          std::string edge_id = m_traci->vehicle.getRoadID(m_vehicle_id);
          double pos = m_traci->vehicle.getLanePosition(m_vehicle_id);
          std::string target_lane_id = edge_id + "_" + std::to_string(target_lane);
          m_traci->vehicle.moveTo(m_vehicle_id, target_lane_id, pos);
        }
      else
        {
          // Check the comfort criterion
          double acc_rv_ahead = DEFAULT_ACC_VALUE, dec_rv = DEFAULT_ACC_VALUE;
          double time_rv_ahead, time_rv;
          bool possible_hv = true;
          bool possible_rv = true;

          if(RVAhead_id >= 0)
            {
              IDMParams ego_params = getIDMParams(static_cast<StationType> (m_station_type));
              // Acceleration HV would experience with RVAhead as new leader
              double a_ego_after = idmAcceleration(my_speed, speed_RVAhead, min_dist_rv_ahead,
                                                    ego_params.v0, ego_params.T,
                                                    ego_params.s0, ego_params.a, ego_params.b);
              if(a_ego_after < MIN_DECELERATION) possible_hv = false;
              if(!possible_hv)
                {
                  // It is needed to open the gap between RVAhead and HV
                  // RVAhead needs to accelerate
                  std::tuple<double, double> tuple = computeRequiredAcceleration (speed_RVAhead, my_speed, min_dist_rv_ahead, ego_params, 0.1, round(m_maneuver_horizon / 1e3));
                  double a = std::get<0>(tuple);
                  double time = std::get<1>(tuple);
                  NS_ASSERT((time >= 0.0 && time <= 5.0) || time == -1.0);
                  if (std::abs(a - NO_SOLUTION) > DOUBLE_TOLERANCE)
                    {
                      acc_rv_ahead = a;
                      time_rv_ahead = time;
                      possible_hv = true;
                    }
                }
            }

          if(RV_id >= 0)
            {
              IDMParams params = getIDMParams(RV_type);
              // Acceleration HV would experience with RVAhead as new leader
              double a_ego_after = idmAcceleration(speed_RV, my_speed, min_dist_rv,
                                                    params.v0, params.T,
                                                    params.s0, params.a, params.b);
              if(a_ego_after < MIN_DECELERATION) possible_rv = false;
              if(!possible_rv)
                {
                  // It is needed to open the gap between RVAhead and HV
                  // RVAhead needs to accelerate
                  std::tuple<double, double> tuple = computeRequiredDeceleration (my_speed, speed_RV, min_dist_rv, params, 0.1, round(m_maneuver_horizon / 1e3));
                  double a = std::get<0>(tuple);
                  double time = std::get<1>(tuple);
                  NS_ASSERT((time >= 0.0 && time <= 5.0) || time == -1.0);
                  if (std::abs(a - NO_SOLUTION) > DOUBLE_TOLERANCE)
                    {
                      dec_rv = a;
                      time_rv = time;
                      possible_rv = true;
                    }
                }
            }
          
          if (possible_hv && possible_rv)
          {
            // If we have RV and/or RVAhead but they don't need to change their current gap with HV, we ask them to keep constant speed
            if (RV_id >= 0 && dec_rv == DEFAULT_ACC_VALUE) {dec_rv = 0; time_rv = 5;}
            if (RVAhead_id >= 0 && acc_rv_ahead == DEFAULT_ACC_VALUE) {acc_rv_ahead = 0; time_rv_ahead = 5;}
            // Need for a coordination
            target_lane = 3 - target_lane;
            startCoordination(RV_id, RVAhead_id, dec_rv, acc_rv_ahead, time_rv, time_rv_ahead, left_criterion, target_lane);
          }
        }
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

void
foresee::startCoordination (long RV_id, long RVAhead_id, double dec_rv, double acc_rv_ahead, double time_rv, double time_rv_ahead, bool left_criterion, int target_lane)
{
  Ptr<MCSpecification> specification = Create<MCSpecification>();
  specification->setForesee();
  // Choose the container
  specification->setAdviseContainer();
  specification->setMCMItsRole (McmItssRole_coordinatingItss); // HV is the coordinator
  if (RV_id >= 0)
    {
      ManoeuvreAdvice* adv = specification->create<ManoeuvreAdvice>();
      adv->executantID = static_cast<StationId_t>(RV_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification->create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv->currentStateAdvisedChange = csac;

      Submanoeuvre_t* subm = specification->create<Submanoeuvre_t>();
      if (dec_rv < 0) asn1cpp::setField(subm->submanoeuvreId, ManeuverID::Slowdown);
      else asn1cpp::setField(subm->submanoeuvreId, ManeuverID::StayInLane);

      subm->foreseeIndication = specification->create<ForeseeIndication_t>();
      asn1cpp::setField(subm->foreseeIndication->acceleration.longitudinalAccelerationValue, dec_rv * CENTI);
      asn1cpp::setField(subm->foreseeIndication->acceleration.longitudinalAccelerationConfidence, AccelerationConfidence::AccelerationConfidence_unavailable);
      // Transform into Milliseconds
      time_rv = time_rv * 1e3;
      asn1cpp::setField(subm->foreseeIndication->duration, time_rv);
      subm->advisedTrajectory = nullptr;
      subm->advisedTargetRoadResource = nullptr;
      specification->add(asn_DEF_Submanoeuvre, &adv->submaneuvres, subm);
      specification->pushManeuverAdvice(adv);
    }

  if (RVAhead_id >= 0)
    {
      ManoeuvreAdvice* adv = specification->create<ManoeuvreAdvice>();
      adv->executantID = static_cast<StationId_t>(RVAhead_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification->create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv->currentStateAdvisedChange = csac;

      Submanoeuvre_t* subm = specification->create<Submanoeuvre_t>();
      if (acc_rv_ahead > 0) asn1cpp::setField(subm->submanoeuvreId, ManeuverID::Accelerate);
      else asn1cpp::setField(subm->submanoeuvreId, ManeuverID::StayInLane);

      subm->foreseeIndication = specification->create<ForeseeIndication_t>();
      asn1cpp::setField(subm->foreseeIndication->acceleration.longitudinalAccelerationValue, acc_rv_ahead * CENTI);
      asn1cpp::setField(subm->foreseeIndication->acceleration.longitudinalAccelerationConfidence, AccelerationConfidence::AccelerationConfidence_unavailable);
      // Transform into Milliseconds
      time_rv_ahead = time_rv_ahead * 1e3;
      asn1cpp::setField(subm->foreseeIndication->duration, time_rv_ahead);
      subm->advisedTrajectory = nullptr;
      subm->advisedTargetRoadResource = nullptr;
      specification->add(asn_DEF_Submanoeuvre, &adv->submaneuvres, subm);
      specification->pushManeuverAdvice(adv);
    }

  specification->setMCMConcept (0); // MCM Goal will be set
  specification->setMCMCost (0);
  specification->setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
  specification->setMCMType (McmType::McmType_request); // HV asks the others for a cooperation
  specification->setManeuverID (left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
  // FORESEE is designed for passenger cars and trucks
  specification->setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
  if(RV_id >= 0) m_acceptance_map[RV_id] = {false, dec_rv, time_rv};
  if(RVAhead_id >= 0) m_acceptance_map[RVAhead_id] = {false, acc_rv_ahead, time_rv_ahead};
  m_coordinator = true;
  m_busy_with_maneuver = true;
  // Set constant speed for the negotiation time for HV
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
  m_traci->vehicle.setAcceleration(m_vehicle_id, 0, 1);
  double value = dist(gen);
  m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
  m_left_criterion = left_criterion;
  m_target_lane = target_lane;
  m_negotiation_event = Simulator::Schedule(MilliSeconds(m_negotiation_time), &foresee::negotiationPhase, this, left_criterion, target_lane);
}

void foresee::negotiationPhase(bool left_criterion, int target_lane)
{
  if (!m_coordinator || !m_busy_with_maneuver) return;
  bool its_ok = true;
  std::string veh_refused;
  std::vector<double> times;
  std::vector<std::string> coordinators;
  std::vector<double> accelerations;
  for (auto it : m_acceptance_map)
    {
      // Found an actor that doesn't want to coordinate
      if(!it.second.accepted) {its_ok = false; veh_refused = "veh" + std::to_string(it.first); break;}
      times.push_back(it.second.time);
      coordinators.push_back("veh" + std::to_string(it.first));
      accelerations.push_back(it.second.acceleration);
      // Highlight vehicles involved
    }
  if(its_ok)
    {
      if (m_verbose)
      {
        std::cout << "\n[NEGOTIATION OK]" << std::endl;
        std::cout << m_vehicle_id << " will change lane from " << m_traci->vehicle.getLaneIndex(m_vehicle_id) << " to " << target_lane << std::endl;
        int counter = 0;
        for (auto it = coordinators.begin(); it != coordinators.end(); ++it)
        {
          double speed = m_traci->vehicle.getSpeed(*it);
          double final_speed = speed + times[counter] / 1e3 * accelerations[counter];
          std::cout << "Vehicle " << *it << " is going at " << speed << " and after " << times[counter] / 1e3 << " seconds will be at " << final_speed << std::endl;
          counter ++;
        }
      }
      // Start the maneuver
      // First send the ACK to the others
      Ptr<MCSpecification> specification = Create<MCSpecification>();
      specification->setForesee();
      specification->setAcknowledgmentContainer();
      specification->setMCMItsRole (McmItssRole_coordinatingItss);
      specification->setMCMType (McmType::McmType_acknowledgment);
      specification->setMCMConcept (0); // MCM Goal will be set
      specification->setMCMCost (0); // Default, it is not used for this use case
      specification->setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
      specification->setManeuverID(left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
      specification->setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
      double value = dist(gen);
      m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
      double time_to_wait = *std::max_element(times.begin(), times.end());
      // HV should keep constant speed until the time to change lane
      m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
      m_traci->vehicle.setAcceleration(m_vehicle_id, 0, time_to_wait / 1e3);
      m_change_lane_event = Simulator::Schedule(MilliSeconds(time_to_wait), [this, target_lane, coordinators]() {
          if (!m_busy_with_maneuver) return; // coordination was cancelled mid-wait
          if (m_verbose)
          {
            std::cout << "\n[LANE CHANGE TIME]" << std::endl;
            std::cout << "Vehicle " << m_vehicle_id << " is trying to coordinate" << std::endl;
            for (auto it = coordinators.begin(); it != coordinators.end(); ++it)
            {
              double speed = m_traci->vehicle.getSpeed(*it);
              std::cout << "Vehicle " << *it << " is going at " << speed << " after coordination" << std::endl;
            }
          }
          double my_heading = m_vdp->getHeadingValue();
          double my_x = m_vdp->getPositionXY().x;
          double my_speed = m_vdp->getSpeedValue();
          std::vector<LDM::returnedVehicleData_t> vehicles;
          m_LDM->getAllCVs (vehicles);
          // Check whether the comfort criteria is now respected for all the participants
          for (auto it = coordinators.begin(); it != coordinators.end(); ++it)
          {
            auto item = std::find_if(vehicles.begin(), vehicles.end(), 
            [&](const LDM::returnedVehicleData_t& veh)
                {
                    return veh.vehData.stationID == std::stol((*it).substr(3));
                });

            if (item != vehicles.end())
            {
              // Check the position of HV w.r.t. the coordinator, to check which one is the leader and which is the follower
              bool behind = false;
              if ((std::abs(my_heading - 90) < DOUBLE_TOLERANCE && item->vehData.x < my_x) || (std::abs(my_heading - 270) < DOUBLE_TOLERANCE && item->vehData.x > my_x)) behind = true;
              double a;
              auto pos = m_traci->simulation.convertLonLattoXY (item->vehData.lon, item->vehData.lat);
              double x = pos.x;
              double gap = std::abs(my_x - x);
              if (behind)
              {
                // Coordinator is behind of the HV
                double leader_length = m_vdp->getVehicleLength();
                gap -= leader_length;
                IDMParams params = getIDMParams(static_cast<StationType> (item->vehData.stationType));
                a = idmAcceleration(item->vehData.speed_ms, my_speed, gap,
                                                    params.v0, params.T,
                                                    params.s0, params.a, params.b);
              }
              else 
              {
                // Coordinator is ahead of the HV
                double leader_length = (double) item->vehData.vehicleLength.getData() / DECI;
                gap -= leader_length;
                IDMParams params = getIDMParams(static_cast<StationType> (m_station_type));
                
                a = idmAcceleration(my_speed, item->vehData.speed_ms, gap,
                                                    params.v0, params.T,
                                                    params.s0, params.a, params.b);
              }
              
              if(a < MIN_DECELERATION)
              {
                // Comfort criterion if not reached for this item, coordination failed
                return;
              }
            }
          }
          // For all the coordinators the comfort has been reached, we can coordinate
          std::string edge_id = m_traci->vehicle.getRoadID(m_vehicle_id);
          double pos = m_traci->vehicle.getLanePosition(m_vehicle_id);
          std::string target_lane_id = edge_id + "_" + std::to_string(target_lane);
          m_traci->vehicle.moveTo(m_vehicle_id, target_lane_id, pos);
          // Send termination to free all targets
          Ptr<MCSpecification> specification = Create<MCSpecification>();
          specification->setForesee();
          specification->setTerminatorContainer();
          specification->setMCMItsRole(McmItssRole_coordinatingItss);
          specification->setMCMType(McmType::McmType_termination);
          specification->setMCMConcept(0);
          specification->setMCMCost(0);
          specification->setMCMGoal(ManoeuvreCooperationGoal_localTrafficManagement);
          double value = dist(gen);
          m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);

          m_coordinator = false;
          m_busy_with_maneuver = false;
          m_acceptance_map.clear();
          m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
          Simulator::Schedule(MilliSeconds(500), &foresee::checkLane, this);
      });
    }
  else
    {
      if (m_verbose)
      {
        std::cout << "\n[NEGOTIATION FAILED]" << std::endl;
        std::cout << "Vehicle " << m_vehicle_id << " will not change lane due to a refusal from " << veh_refused << std::endl;
      }
      // Cancel the maneuver
      Ptr<MCSpecification> specification = Create<MCSpecification>();
      specification->setForesee();
      specification->setTerminatorContainer();
      specification->setMCMItsRole (McmItssRole_coordinatingItss);
      specification->setMCMType (McmType::McmType_termination);
      specification->setMCMConcept (0); // MCM Goal will be set
      specification->setMCMCost (0); // Default, it is not used for this use case
      specification->setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
      specification->setManeuverID(left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
      specification->setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
      double value = dist(gen);
      m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
      m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
      m_busy_with_maneuver = false;
      m_coordinator = false;
      m_acceptance_map.clear();
    }
}

void foresee::targetCheckACK()
{
  // Guard: if we are no longer in a coordination or already responded, do nothing
  if (m_my_coordinator == -1 || m_my_coordinator_responded || !m_busy_with_maneuver)
  {
    if (m_verbose)
      {
        std::cout << "\n[TARGET CHECK]" << std::endl;
        std::cout << "Vehicle " << m_vehicle_id << " doesn't need to check" << std::endl;
      }
    return;
  }
  Ptr<MCSpecification> specification = Create<MCSpecification>();
  specification->setForesee();
  specification->setTerminatorContainer();
  specification->setMCMItsRole (McmItssRole_targetVehicle);
  specification->setMCMType (McmType::McmType_cancellationRequest);
  specification->setMCMConcept (0); // MCM Goal will be set
  specification->setMCMCost (0); // Default, it is not used for this use case
  specification->setMCMGoal(ManoeuvreCooperationGoal_localTrafficManagement);
  double value = dist(gen);
  m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
  m_my_coordinator = -1;
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
  m_my_coordinator_responded = false;
  m_busy_with_maneuver = false;
  if (m_verbose)
  {
    std::cout << "\n[MANEUVER REFUSED]" << std::endl;
    std::cout << "Vehicle " << m_vehicle_id << " is refusing the maneuver due to ACK not received " << std::endl;
  }
}

void foresee::receiveMCM(const asn1cpp::Seq<MCM>& mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
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

  switch (present_container)
    {
      case McmContainer_PR_NOTHING:
        no_containers = true;
        break;
      case McmContainer_PR_advisedManoeuvreContainer:
      {
        adc = mcm->payload.mcmContainer.choice.advisedManoeuvreContainer;
        advice_container = true;
        break;
      }
      case McmContainer_PR_vehicleManoeuvreContainer:
      {
        man = mcm->payload.mcmContainer.choice.vehicleManoeuvreContainer;
        maneuver_container = true;
        break;
      }
      case McmContainer_PR_acknowledgmentContainer:
      {
        ack = mcm->payload.mcmContainer.choice.acknowledgmentContainer;
        ack_container = true;
        break;
      }
      case McmContainer_PR_responseContainer:
      {
        resp = mcm->payload.mcmContainer.choice.responseContainer;
        resp_container = true;
        break;
      }
      case McmContainer_PR_terminationContainer:
      {
        term = mcm->payload.mcmContainer.choice.terminationContainer;
        term_container = true;
        break;
      }
      default:
        break;
    }

  if (no_containers)
    {
      // If no container present, directly return
      return;
    }

  // MCM type identification
  // FORESEE takes into account: Request, Response, Acknowledgment, Execution Status, Terminator
  switch (type)
    {
    case McmType::McmType_request:
      // Accept the coordination if the request is for us
      if (advice_container)
        {
          bool accept = false;
          bool msg_for_me = false;
          int my_id = std::stol(m_vehicle_id.substr(3));
          int adv_size = asn1cpp::sequenceof::getSize(adc);
          for(int i = 0; i < adv_size; ++i)
            {
              auto adv = adc.list.array[i];
              StationId_t id = adv->executantID;
              if (id == my_id)
                {
                  msg_for_me = true;
                  if (m_busy_with_maneuver)
                    {
                      if (m_verbose)
                        {
                          std::cout << "\n[MANEUVER REFUSED]" <<std::endl;
                          std::cout << "Vehicle " << m_vehicle_id << " cannot accept maneuver from veh" << sender << " because it is busy with another" << std::endl;
                        }
                      break;
                    }
                  else
                    {
                      // In FORESEE we have just one submaneuver per each executant
                      auto subm = adv->submaneuvres.list.array[0];
                      auto sub_id = asn1cpp::getField(subm->submanoeuvreId, Identifier1B_t);
                      auto acc_value = (double) asn1cpp::getField(subm->foreseeIndication->acceleration.longitudinalAccelerationValue, LongitudinalAccelerationValue) / CENTI;
                      auto time_s = (double) asn1cpp::getField(subm->foreseeIndication->duration, DeltaTimeMilliSecondPositive_t) / MILLI;
                      if ((sub_id == ManeuverID::Accelerate && acc_value < 0) || (sub_id == ManeuverID::Slowdown && acc_value > 0) || (sub_id == ManeuverID::StayInLane && acc_value != 0))
                      {
                        // Error in message generation: acceleration/deceleration required but acceleration sign is not consistent
                        if (m_verbose)
                        {
                          std::cout << "\n[ERROR IN MESSAGE]" << std::endl;
                          std::cout << "Acceleration or deceleration not right" << std::endl;
                        }
                        break;
                      }
                      double current_speed = m_vdp->getSpeedValue();
                      double future_speed = current_speed + acc_value * time_s;
                      if ((future_speed - m_desired_speed) > DOUBLE_TOLERANCE)
                      {
                        if (m_verbose)
                        {
                          std::cout << "\n[MANEUVER REFUSED]" <<std::endl;
                          std::cout << "For vehicle " << m_vehicle_id << " the final speed would exceed the desired one: " << m_desired_speed << " < " << future_speed << std::endl;
                        }
                        break;
                      }
                      m_required_acceleration_time = std::make_tuple(acc_value, time_s);
                      accept = true;
                      break;
                    }
                }
            }
          if (accept && msg_for_me)
            {
              if (m_verbose)
              {
                std::cout << "\n[MANEUVER ACCEPTED]" <<std::endl;
                std::cout << "For vehicle " << m_vehicle_id << " the maneuver requested by veh" << sender << " is feasible" << std::endl;
              }
              // Keep constant speed until the HV will start the maneuver
              m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
              m_traci->vehicle.setAcceleration(m_vehicle_id, 0, 1.5);
              // Accept the coordination
              Ptr<MCSpecification> specification = Create<MCSpecification>();
              specification->setForesee();
              specification->setResponseContainer();
              specification->setMCMItsRole (McmItssRole_targetVehicle);
              specification->setMCMType(McmType::McmType_response);
              specification->setMCMResponse (0);
              specification->setMCMConcept(0);
              specification->setMCMCost(0);
              specification->setMCMGoal(ManoeuvreCooperationGoal_localTrafficManagement);
              m_busy_with_maneuver = true;
              m_my_coordinator = sender;
              m_my_coordinator_responded = false;
              double value = dist(gen);
              m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
              m_ack_event = Simulator::Schedule(MilliSeconds(1500), &foresee::targetCheckACK, this);
            }
          else if (!accept && msg_for_me)
            {
              // Refuse the coordination
              Ptr<MCSpecification> specification = Create<MCSpecification>();
              specification->setForesee();
              specification->setResponseContainer();
              specification->setMCMItsRole (McmItssRole_targetVehicle);
              specification->setMCMResponse (1);
              specification->setMCMConcept(0);
              specification->setMCMCost(0);
              specification->setMCMGoal(ManoeuvreCooperationGoal_localTrafficManagement);
              specification->setMCMType(McmType::McmType_response);
              double value = dist(gen);
              m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
            }
          else if (!msg_for_me)
            {
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
              if (same_direction)
              {
                // I register the coordination if within my Coordination Avoidance range
                double my_x = m_vdp->getPositionXY().x;
                // Check that the coordinator is haed
                if ((direction == 0 && pos.x >= my_x) || (direction == 1 && pos.x <= my_x)) 
                {
                  double gap = std::abs(pos.x - my_x);
                  if (gap <= m_ca_range)
                  {
                    m_blocked_by_other_coordinations[sender] = Simulator::Now().GetMilliSeconds();
                  }
                }
              }
            }
        }
      break;

    case McmType::McmType_response:
      if (resp_container)
        {
          if (m_coordinator && sender_role == McmItssRole::McmItssRole_targetVehicle && m_acceptance_map.find(sender) != m_acceptance_map.end())
            {
              // Response received
              // The coordinator is waiting the ACK from the others
              ManouevreResponse_t response = resp.manouevreResponse;
              if (response == ManouevreResponse::ManouevreResponse_accept)
                {
                  // One of the vehicles accepted to be involved in the coordination
                  // Update the map
                  m_acceptance_map[sender].accepted = true;
                  if (m_verbose)
                  {
                    std::cout << "\n[ACCEPTANCE RECEIVED]" << std::endl;
                    std::cout << "Vehicle " << m_vehicle_id << " has received the acceptance from veh" << sender << std::endl;
                  }
                  // Check if we have received all the acceptance, we can anticipate the negotiation end
                  bool received_all = true;
                  for (auto it = m_acceptance_map.begin(); it != m_acceptance_map.end(); ++it)
                  {
                    if (!it->second.accepted) {received_all = false; break;}
                  }
                  if (received_all)
                  {
                    Simulator::Cancel(m_negotiation_event);
                    negotiationPhase(m_left_criterion, m_target_lane);
                  }
                }
              else if (response == ManouevreResponse::ManouevreResponse_decline)
              {
                // One of the vehicles declined the coordination
                // We need to check whether this refusal is for another maneuver request by simply check the acceptance map
                // In case the vehicle has already accepted, the rejection is not for us
                if (m_acceptance_map[sender].accepted != true)
                {
                  m_acceptance_map[sender].accepted = false;
                  if (m_verbose)
                  {
                    std::cout << "\n[REFUSAL RECEIVED]" << std::endl;
                    std::cout << "Vehicle " << m_vehicle_id << " has received the refusal from veh" << sender << std::endl;
                  }
                  Simulator::Cancel(m_negotiation_event);
                  negotiationPhase(m_left_criterion, m_target_lane);
                }
              }
            }
        }
      break;

    case McmType::McmType_termination:
      // Termination reception, the vehicle is now free to do other coordinations
      if (sender_role == McmItssRole_coordinatingItss && sender != m_my_coordinator && m_blocked_by_other_coordinations.find(sender) != m_blocked_by_other_coordinations.end())
      {
        // A maneuver is terminated, erase it from the set if it is not my maneuver
        // Must be the coordinator to stop the menauver
        m_blocked_by_other_coordinations.erase(sender);
      }
      else if (sender_role == McmItssRole_coordinatingItss && sender == m_my_coordinator)
      {
        // If it is my maneuver, now I am free for others
        // We can cancel the check for the ACK
        Simulator::Cancel(m_ack_event);
        Simulator::Cancel(m_tx_mcm_event);
        Simulator::Cancel(m_continue_constant_speed_event);
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
        m_busy_with_maneuver = false;
        m_my_coordinator_responded = false;
        m_my_coordinator = -1;
      }
      break;

    case McmType::McmType_acknowledgment:
      if (sender_role == McmItssRole_coordinatingItss && sender == m_my_coordinator)
      {
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
      if (m_coordinator && m_acceptance_map.find(sender) != m_acceptance_map.end())
      {
        // Cancel the other events if any
        Simulator::Cancel(m_negotiation_event);
        Simulator::Cancel(m_change_lane_event);
        Simulator::Cancel(m_tx_mcm_event);
        Ptr<MCSpecification> specification = Create<MCSpecification>();
        specification->setForesee();
        specification->setTerminatorContainer();
        specification->setMCMItsRole (McmItssRole_coordinatingItss);
        specification->setMCMType (McmType::McmType_termination);
        specification->setMCMConcept (0); // MCM Goal will be set
        specification->setMCMCost (0); // Default, it is not used for this use case
        specification->setMCMGoal(ManoeuvreCooperationGoal_localTrafficManagement);
        double value = dist(gen);
        m_tx_mcm_event = Simulator::Schedule(MilliSeconds(value), &txMCM, m_mcs_ptr, specification);
        m_traci->vehicle.setSpeedMode(m_vehicle_id, 31);
        m_coordinator = false;
        m_busy_with_maneuver = false;
        m_acceptance_map.clear();
      }
      break;
    default:
      break;
    }
}

void foresee::executeManeuver()
{
  double acc = std::get<0>(m_required_acceleration_time);
  double duration = std::get<1>(m_required_acceleration_time);
  m_traci->vehicle.setSpeedMode(m_vehicle_id, 0);
  m_traci->vehicle.setAcceleration(m_vehicle_id, acc, duration);
  m_continue_constant_speed_event = Simulator::Schedule(MilliSeconds(duration*1e3), &foresee::continueWithConstantSpeed, this, m_my_coordinator);
}

void foresee::continueWithConstantSpeed(StationId_t coordinator)
{
  // Check if the maneuver has been finished thanks between the old coordinator and the current one
  if(coordinator == m_my_coordinator)
  {
    // The termination has not been received yet from the coordinator
    // The vehicle continues with constant speed
    if (m_verbose)
    {
      std::cout << "\n[TERMINATION NOT YET RECEIVED]" << std::endl;
      std::cout << m_vehicle_id << " will continue with constant speed" << std::endl;
    }
    m_traci->vehicle.setAcceleration(m_vehicle_id, 0, m_maneuver_horizon / 1e3);
  }
  else 
  {
    // The termination has already been received, we are in normal speed mode, so we can avoid to set a specific speed
    if (m_verbose)
    {
      std::cout << "\n[TERMINATION ALREADY RECEIVED]" << std::endl;
      std::cout << "Vehicle " << m_vehicle_id << " will not change its current motion" << std::endl;
    }
  }
  

}

void foresee::checkLane()
{
  std::cout << "\n[CHANGE LANE CHECK]" << std::endl;
  std::cout << "New lane for vehicle " << m_vehicle_id << " is " << m_traci->vehicle.getLaneIndex(m_vehicle_id) << std::endl;
}

}

