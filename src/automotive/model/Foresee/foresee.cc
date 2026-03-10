//
// Created by diego on 01/12/25.
//

#include "foresee.h"

namespace ns3
{
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
  // Schedule the FORESEE Mobility Model to start at the specified time
  Simulator::Schedule (Seconds(m_start_time), &foresee::FORESEEMobilityModel, this);
}

void
foresee::setNumberOfLanes ()
{
  // Retrieve the number of lanes for the current road using TraCI
  int lanes = m_traci->TraCIAPI::edge.getLaneNumber (m_traci->TraCIAPI::vehicle.getRoadID (m_vehicle_id));
  m_num_lanes = lanes;
}

double IDMAcceleration(
               double v,
               double v_lead,
               double gap,
               double v0,
               double a_max,
               double b,
               double s0,
               double T,
               double delta = 4.0
)
{
  // Ensure the gap is not too small to avoid singularities
  gap = std::max(gap, 0.1);

  // Calculate the relative speed and desired gap
  double delta_v = v - v_lead;
  double s_star = s0 + v * T + (v * delta_v) / (2.0 * std::sqrt(a_max * b));

  // Compute the acceleration based on the IDM formula
  double a_exp = a_max * (1.0 - std::pow(v / v0, delta) - std::pow(s_star / gap, 2.0));

  return a_exp;
}

double
estimateTimeFromPredictionIDM(
    std::vector<foresee::TrajectoryItem> leader,
    std::vector<foresee::TrajectoryItem> follower,
    double comfort_decel,
    double a_max,
    double desired_speed,
    double b,
    double s0,
    double T)
{
  // Determine the minimum size of the leader and follower trajectories
  size_t N = std::min(leader.size(), follower.size());

  for (size_t k = 0; k < N; ++k)
    {
      auto lead = leader[k];
      auto foll = follower[k];

      // Calculate the gap between the leader and follower at this timestep
      double gap = std::abs(lead.x - foll.x);

      // Calculate the required braking for the follower
      double a_required_RV = IDMAcceleration(
          foll.speed,   // follower
          lead.speed,   // leader
          gap,
          desired_speed,
          a_max,
          b,
          s0,
          T
      );

      // Check if the required braking is within comfort limits
      if (a_required_RV >= -comfort_decel)
        {
          return (k + 1) * STEP_TIME; // EstimatedTime found
        }
    }

  return -1.0; // No feasible coordination in time horizon
}

void
foresee::FORESEEMobilityModel ()
{
  // Retrieve all connected vehicles (CVs) from the LDM
  std::vector<LDM::returnedVehicleData_t> vehicles;
  bool res = m_LDM->getAllCVs (vehicles);
  if (res == false)
    {
      // TODO automatically move vehicle since it can change the lane without any issues
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
  // Lane normalized the lane in ETSI-based system --> 1 left-most lane, 2 center lane, 3 right-most lane
  VDPDataItem<int> my_lane = m_vdp->getLanePosition();

  // Process each vehicle in the LDM
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
      // Skip vehicles in different directions
      if (it->vehData.heading != my_heading) continue;
      auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
      double x = pos.x;

      // Skip vehicles behind the ego vehicle
      if (my_heading == 90 && x < my_x) continue;
      if (my_heading == 270 && x > my_x) continue;
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
  bool could_change_lane;
  // Direction for TraCI: {-1=right, 1=left}
  int8_t lc_direction = 0;
  if (left_criterion) lc_direction = 1;
  else if (right_criterion) lc_direction = -1;
  assert (lc_direction == 0 || lc_direction == 1 || lc_direction == -1);
  bool startManeuver = false;
  if (lc_direction != 0)
    // At least one incentive criterion is satisfied
    // Check the comfort criterion
    {
      // Check the coordination avoidance range
      bool found_coordination = false;
      // Take the four roles, target, ahead ego, ahead target
      std::string RV, HVAhead, RVAhead;
      // Check whether there is another maneuver coordination that is happening within the ahead range
      // If yes, ego vehicle cannot perform maneuver coordination
      for (auto it = (*m_lc_data_structure).begin(); it != (*m_lc_data_structure).end(); ++it)
        {
          auto v = it->second;
          float heading = std::get<0>(v);
          if (heading != my_heading) continue;
          float x = std::get<1>(v);
          // Filter behind vehicles
          if (my_heading == 90 && x < my_x) continue;
          if (my_heading == 270 && x > my_x) continue;
          float y = std::get<2>(v);
          float dist = std::sqrt (std::pow(my_x - x, 2) + std::pow(my_y - y, 2));
          if (dist <= m_ca_range)
            {
              found_coordination = true;
              break;
            }
        }
      if (!found_coordination)
        {
          // No other coordination in progress, check the comfort criterion
          startManeuver = true;
          double x_RV, x_RVAhead, x_HVAhead;
          double y_RV, y_RVAhead, y_HVAhead;
          double speed_RV, speed_RVAhead, speed_HVAhead;
          double min_dist_hv_ahead = 10000;
          double min_dist_rv_ahead = 10000;
          double min_dist_rv = 10000;
          // Vehicles ahead of HV in the same lane
          auto& vec1 = veh_per_lane[my_lane.getData()];
          // Target lane
          int target_lane = left_criterion ? my_lane.getData() - 1 : my_lane.getData() + 1;
          // Vehicles ahead of HV in the target lane
          auto& vec2 = veh_per_lane[target_lane];
          for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
            {
              auto it_found = std::find(
                  vec1.begin(),
                  vec1.end(),
                  std::to_string(it->vehData.stationID)
              );
              auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
              double dist = std::sqrt (std::pow(my_x - pos.x, 2) + std::pow(my_y - pos.y, 2));
              if (it_found != vec1.end())
                {
                  // Vehicle is in the same lane of HV ahead of ego, can be HVAhead
                  if (dist < min_dist_hv_ahead && dist < MAX_DIST_AHEAD_BEHIND)
                    {
                      min_dist_hv_ahead = dist;
                      HVAhead = "veh" + std::to_string (it->vehData.stationID);
                      x_HVAhead = pos.x;
                      y_HVAhead = pos.y;
                      speed_HVAhead = it->vehData.speed_ms;
                    }
                }
              else
                {
                  // Check for RV and RVAhead
                  it_found = std::find(
                      vec2.begin(),
                      vec2.end(),
                      std::to_string(it->vehData.stationID)
                  );
                  if (it_found != vec2.end())
                    {
                      // Vehicle is in the target lane ahead of ego, can be RVAhead
                      if (dist < min_dist_rv_ahead && dist < MAX_DIST_AHEAD_BEHIND)
                        {
                          min_dist_rv_ahead = dist;
                          RVAhead = "veh" + std::to_string (it->vehData.stationID);
                          x_RVAhead = pos.x;
                          y_RVAhead = pos.y;
                          speed_RVAhead = it->vehData.speed_ms;
                        }
                    }
                  else
                    {
                      // Can be RV
                      OptionalDataItem<long> lane = it->vehData.lanePosition;
                      if(lane.isAvailable() && lane.getData() == target_lane)
                        {
                          if (dist < min_dist_rv && dist < MAX_DIST_AHEAD_BEHIND)
                            {
                              min_dist_rv = dist;
                              RV = "veh" + std::to_string (it->vehData.stationID);
                              x_RV = pos.x;
                              y_RV = pos.y;
                              speed_RV = it->vehData.speed_ms;
                            }
                        }
                    }
                }
            }
          m_actors = {RV, HVAhead, RVAhead};
          std::vector<TrajectoryItem> mp_RV;
          std::vector<TrajectoryItem> mp_RVAhead;
          std::vector<TrajectoryItem> mp_HVAhead;
          int8_t sign = my_heading == 270 ? -1 : 1;
          // Do prediction for each actor, if present
          if(!RV.empty()) mp_RV = predictConstantSpeed (x_RV, y_RV, speed_RV, sign, true);
          if(!RVAhead.empty()) mp_RVAhead = predictConstantSpeed (x_RVAhead, y_RVAhead, speed_RVAhead, sign);
          if(!HVAhead.empty()) mp_HVAhead = predictConstantSpeed (x_HVAhead, y_HVAhead, speed_HVAhead, sign);
          std::vector<TrajectoryItem> mp_HV = predictConstantSpeed (my_x, my_y, my_speed, sign);
          double estimated_time_hv_rv = 0, estimated_time_hv_rvahead = 0, estimated_time_hv_hvahead = 0;
          if(!mp_RV.empty())
            {
              estimated_time_hv_rv = estimateTimeFromPredictionIDM (
                  mp_HV,
                  mp_RV,
                  std::abs(COMFORT_DECELERATION), // comfort threshold
                  m_traci->vehicletype.getAccel (my_type),
                  m_desired_speed,
                  std::abs(SAFE_DECELERATION),
                  m_traci->vehicletype.getMinGap (my_type),
                  m_traci->vehicletype.getTau (my_type)
              );
            }
          if (!mp_RVAhead.empty())
            {
              estimated_time_hv_rvahead = estimateTimeFromPredictionIDM (
                  mp_RVAhead,
                  mp_HV,
                  std::abs(COMFORT_DECELERATION), // comfort threshold
                  m_desired_speed,
                  m_traci->vehicletype.getAccel (my_type),
                  std::abs(SAFE_DECELERATION),
                  m_traci->vehicletype.getMinGap (my_type),
                  m_traci->vehicletype.getTau (my_type)
              );
            }
          if (!mp_HVAhead.empty())
            {
              estimated_time_hv_hvahead = estimateTimeFromPredictionIDM (
                  mp_HVAhead,
                  mp_HV,
                  std::abs(COMFORT_DECELERATION), // comfort threshold
                  m_desired_speed,
                  m_traci->vehicletype.getAccel (my_type),
                  std::abs(SAFE_DECELERATION),
                  m_traci->vehicletype.getMinGap (my_type),
                  m_traci->vehicletype.getTau (my_type)
              );
            }

          if (estimated_time_hv_rv == -1){
              std::cout << "Not found for HV-RV" << std::endl;
            }
          if (estimated_time_hv_hvahead == -1){
              std::cout << "Not found for HV-HVAhead" << std::endl;
            }
          if (estimated_time_hv_rvahead == -1){
              std::cout << "Not found for HV-RVAhead" << std::endl;
            }
          // Insert in the data structure the new coordination event that is going to happen
          // (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
          // Simulator::Schedule (MilliSeconds(0), &foresee::doCoordination, this);
          if (false)
            {
              // TraCI utility to determine whether the lane change in a certain direction is possible
              could_change_lane = m_traci->vehicle.couldChangeLane (m_vehicle_id, lc_direction);
              if (could_change_lane)
                {
                  // There are not any obstacles to change lane, so do it directly
                  // Need to convert my_lane into SUMO-based lane system
                  int target_lane;
                  if (left_criterion)
                    {
                      target_lane = my_lane.getData() - 1;
                    }
                  else if (right_criterion)
                    {
                      target_lane = my_lane.getData() + 1;
                    }
                  target_lane = m_num_lanes - target_lane;
                  m_traci->vehicle.changeLane (m_vehicle_id, target_lane, m_time_to_lc);
                }
              else
                {
                  // Need for a maneuver coordination
                  auto [state, stateTraCI] = m_traci->vehicle.getLaneChangeState(m_vehicle_id, lc_direction);
                  std::vector<uint8_t> reasons = m_traci->vehicle.getLaneChangeFailureReasons (state, stateTraCI, lc_direction);
                  // TODO maybe use the reason in some way (sometimes is UNKNOWN)
                }
            }
        }
    }
  if (!startManeuver) Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

void
foresee::doCoordination ()
{
  MCSpecification specification = {};
  m_mcs_ptr->generateAndEncodeMCM (specification);
}

void
foresee::terminateCoordination ()
{
  // Clear the data structure
  (*m_lc_data_structure).erase(m_vehicle_id_int);
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

std::vector<foresee::TrajectoryItem>
foresee::predictConstantSpeed(double x, double y, double speed, int8_t sign, bool is_RV)
{
  std::vector<foresee::TrajectoryItem> motion_plan;
  if (!is_RV)
    {
      float t = 0.5;
      while (t <= HORIZON_TIME)
      {
        double delta = sign * (speed * STEP_TIME);
        x += delta;
        TrajectoryItem item {x, y, speed};
        motion_plan.push_back (item);
        t += STEP_TIME;
      }
    }
  else
    {
      float t = 0.5;
      while (t <= HORIZON_TIME)
        {
          if (t <= NEGOTIATION_TIME || t > NEGOTIATION_TIME + DECELERATION_TIME)
            {
              double delta = sign * (speed * STEP_TIME);
              x += delta;
              TrajectoryItem item {x, y, speed};
              motion_plan.push_back (item);
              t += STEP_TIME;
            }
          else if (t > NEGOTIATION_TIME && t <= NEGOTIATION_TIME + DECELERATION_TIME)
            {
              double delta = sign * (speed * STEP_TIME + 0.5 * COMFORT_DECELERATION * std::pow(STEP_TIME, 2));
              x += delta;
              double delta_speed = COMFORT_DECELERATION * STEP_TIME;
              speed += delta_speed;
              TrajectoryItem item {x, y, speed};
              motion_plan.push_back (item);
              t += STEP_TIME;
            }
        }
    }
  return motion_plan;
}

}
