//
// Created by diego on 01/12/25.
//

#include "foresee.h"
#include "ns3/foresee.h"

namespace ns3
{

foresee::IDMParams foresee::getIDMParams(StationType type) {
  if (type == StationType::StationType_passengerCar) {
      return {m_desired_speed, 0.8, 2.0, 1.5, 1.5};
    } else if (type == StationType::StationType_lightTruck) { // passenger car
      return {m_desired_speed, 1.0, 2.0, 1.5, 2.0};
    }
}

double foresee::idmAcceleration(double v, double v_lead, double gap, double v0,
                          double T, double s0, double a, double b) {
  // Desired minimum gap
  double s_star = s0 + std::max(0.0, v * T + (v * (v - v_lead)) / (2.0 * std::sqrt(a * b)));
  // IDM acceleration
  return a * (1.0 - std::pow(v / v0, 4) - std::pow(s_star / std::max(gap, 0.1), 2));
}

double foresee::computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on acceleration of leader in [0, a_max]
  double lo = 0.0;
  double hi = p.a; // max acceleration
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double a_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap   = current_gap;
      double v_f = speed_follower;
      double v_l  = speed_leader;
      double a_f_final = -200;
      for(double t = 0; t < horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
          if (a_f >= MIN_DECELERATION)
            {
              break;
            }
          // Leader accelerates with candidate acceleration
          v_l  = std::min(v_l + a_candidate * dt, p.v0);
          // Update gap
          gap  += (v_l - v_f) * dt;
        }
      // Check if at end of horizon ego can merge comfortably
      if(a_f_final >= MIN_DECELERATION)
        hi = a_candidate; // enough, try less
      else
        lo = a_candidate; // not enough, need more
    }
  if(std::abs(hi - (-p.a)) < 0.01)
    return NO_SOLUTION;
  return hi; // minimum acceleration RVAhead needs to apply
}

double foresee::computeRequiredDeceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on deceleration of follower in [0, a_max]
  double lo = -p.a;
  double hi = 0.0; // max deceleration
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double d_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      for(double t = 0; t < horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
          if (a_f >= MIN_DECELERATION)
            {
              break;
            }
          // Follower decelerates with candidate deceleration
          v_f  = std::min(v_f + d_candidate * dt, 0.0);
          // Update gap
          gap  += (v_l - v_f) * dt;
        }
      // Check if at end of horizon ego can merge comfortably
      if(a_f_final >= MIN_DECELERATION)
        lo = d_candidate; // enough, try less deceleration (less negative)
      else
        hi = d_candidate; // not enough, need more deceleration (more negative)
    }
  if(std::abs(lo - (-p.a)) < 0.01)
    return NO_SOLUTION;
  return lo; // minimum deceleration
}

void
foresee::setTrajectoryPredictor (int horizon_time, int step_time, int negotiation_time,
                                 int deceleration_time, int lc_duration, PredictionType prediction_type)
{
  m_traj_predictor = new trajectoryPrediction(horizon_time, step_time, negotiation_time, deceleration_time);
  m_step_time = step_time;
  m_negotiation_time = negotiation_time;
  m_time_to_lc = lc_duration;
  m_prediction_type = prediction_type;
}

void
foresee::addMCMRxCallback ()
{
  std::function<void(asn1cpp::Seq<MCM>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback =
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
  // Check for predictor
  if (m_prediction_type == UNKNOWN)
    {
      NS_FATAL_ERROR ("FORESEE Mobility Model needs the prediction type.");
    }
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
  Simulator::Schedule (Seconds(m_start_time), &foresee::FORESEEMobilityModel, this);
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
      // The route is empty (no perceived vehicles in the LDM
      // FORESEE cannot be activated in this case, so we reschedule it
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
  // Direction for TraCI: {-1=right, 1=left}
  int8_t lc_direction = 0;
  if (left_criterion) lc_direction = 1;
  else if (right_criterion) lc_direction = -1;
  assert (lc_direction == 0 || lc_direction == 1 || lc_direction == -1);
  if (lc_direction != 0)
    // At least one incentive criterion is satisfied
    // Check the comfort criterion
    {
      // Check the coordination avoidance range
      bool found_coordination = false;
      // Take the four roles, target, ahead ego, ahead target
      std::string RV, HVAhead, RVAhead;
      long RV_id = -1, RVAhead_id = -1, HVAhead_id = -1;
      StationType RV_type;
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
          double x_RV, x_RVAhead, x_HVAhead;
          double y_RV, y_RVAhead, y_HVAhead;
          double speed_RV, speed_RVAhead, speed_HVAhead;
          double min_dist_rv_ahead = 10000;
          double min_dist_rv = 10000;
          double min_dist_hv_ahead = 10000;
          // Target lane
          int target_lane = left_criterion ? my_lane.getData() - 1 : my_lane.getData() + 1;
          // Vehicles ahead of HV in the target lane
          auto& vec1 = veh_per_lane[target_lane];
          // Vehicles ahead of HV in the same lane
          auto& vec2 = veh_per_lane[my_lane.getData()];
          for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
            {
              if (it->vehData.lanePosition.getData() == target_lane)
                {
                  auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
                  double dist = std::sqrt (std::pow (my_x - pos.x, 2) + std::pow (my_y - pos.y, 2));
                  auto it_found = std::find (vec1.begin (), vec1.end (),
                                             std::to_string (it->vehData.stationID));
                  if (it_found != vec1.end ())
                    {
                      // Vehicle is in the target lane ahead of ego, can be RVAhead
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
              else if (it->vehData.lanePosition.getData() == my_lane.getData())
                {
                  auto pos = m_traci->simulation.convertLonLattoXY (it->vehData.lon, it->vehData.lat);
                  double dist = std::sqrt (std::pow (my_x - pos.x, 2) + std::pow (my_y - pos.y, 2));
                  auto it_found = std::find (vec2.begin (), vec2.end (),
                                             std::to_string (it->vehData.stationID));
                  if (it_found != vec1.end ())
                    {
                      // Can be HVAhead
                      if (dist < min_dist_rv_ahead && dist < MAX_DIST_AHEAD_BEHIND)
                        {
                          min_dist_hv_ahead = dist;
                          HVAhead = "veh" + std::to_string (it->vehData.stationID);
                          HVAhead_id = it->vehData.stationID;
                          x_HVAhead = pos.x;
                          y_HVAhead = pos.y;
                          speed_HVAhead = it->vehData.speed_ms;
                        }
                    }
                }
            }

          if(RVAhead_id >= 0)
            {
              bool possible_hv = true;
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
                  double a = computeRequiredAcceleration (speed_RVAhead, my_speed, min_dist_rv_ahead, ego_params);
                  if (a == NO_SOLUTION) std::cout << "Not Possible" << std::endl;
                  std::cout << a << std::endl;
                }
            }

          if(RV_id >= 0)
            {
              bool possible_rv = true;
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
                  double a = computeRequiredDeceleration (my_speed, speed_RV, min_dist_rv, params);
                  if (a == NO_SOLUTION) std::cout << "Not Possible" << std::endl;
                  std::cout << a << std::endl;
                }
            }
          /*
          int start_time = -1;
          int8_t sign = my_heading == 270 ? -1 : 1;
          bool feasible_for_RV = false, feasible_for_RVAhead = false, feasible_for_HVAhead = false;
          double deceleration_RV, acceleration_RVAhead, acceleration_HVAhead;
          trajectoryPrediction::TrajectoryItem ref_HV, ref_RV, ref_RVAhead, ref_HVAhead;
          std::vector<trajectoryPrediction::TrajectoryItem> trajectory_HV, trajectory_RV, trajectory_RVAhead, trajectory_HVAhead;
          // Do prediction for each actor, if present
          // Prediction for HV, considering a constant speed prediction model (minimum effort for HV)
          std::tuple<trajectoryPrediction::TrajectoryItem, std::vector<trajectoryPrediction::TrajectoryItem>> item =
              m_traj_predictor->predictConstantSpeed (my_x, my_y, my_speed, -ACCEL_DECEL, sign, trajectoryPrediction::ActorType::HV);
          trajectory_HV = std::get<1>(item);

          // Prediction for RV, considering a constant deceleration during the deceleration time, then constant speed
          // Note that a deceleration of 0 is first used to consider the case in which no motion changes are required for RV
          if(!RV.empty())
            {
              double deceleration_supported = -ACCEL_DECEL;
              double d = 0;
              // The leader in this case is HV
              double leader_length = m_traci->vehicle.getLength (m_vehicle_id);
              // Starting with the least invasive deceleration for RV (= 0)
              // Do multiple trial until we find a possible safe deceleration to apply
              while (d >= deceleration_supported)
                {
                  item = m_traj_predictor->predictConstantSpeed (x_RV, y_RV, speed_RV, d, sign, trajectoryPrediction::ActorType::RV);
                  std::vector<trajectoryPrediction::TrajectoryItem> trajectory = std::get<1>(item);
                  std::tuple<bool, double> ret = trajectoryEvaluation (trajectory_HV, trajectory, leader_length, m_step_time, m_negotiation_time, m_time_to_lc, trajectoryPrediction::ActorType::RV, 0);
                  bool evaluation = std::get<0>(ret);
                  bool start = std::get<1>(ret);
                  if (evaluation)
                    {
                      feasible_for_RV = true;
                      deceleration_RV = d;
                      trajectory_RV = std::move(trajectory);
                      ref_RV = std::get<0>(item);
                      start_time = start;
                      break;
                    }
                  d -= ACCELERATION_STEP;
                }
            }
          else
            {
              feasible_for_RV = true;
              deceleration_RV = DEFAULT_ACC_VALUE;
            }

          // Prediction for RV Ahead, considering a constant acceleration during the deceleration time, then constant speed
          // Note that an acceleration of 0 is first used to consider the case in which no motion changes are required for RV Ahead
          // If present, the maneuver must be feasible for RV
          if(!RVAhead.empty() && feasible_for_RV)
            {
              double acceleration_supported = ACCEL_DECEL;
              double a = 0.0;
              // The leader in this case is RVAhead
              double leader_length = std::find_if(vehicles.begin(), vehicles.end(), [&id = RVAhead_id] (const LDM::returnedVehicleData_t& ldm_item) -> bool {return  ldm_item.vehData.stationID == id;})->vehData.vehicleLength.getData();
              leader_length /= DECI;
              // Starting with the least invasive acceleration for RVAhead (= 0)
              // Do multiple trial until we find a possible safe acceleration to apply
              while (a <= acceleration_supported)
                {
                  item = m_traj_predictor->predictConstantSpeed (x_RVAhead, y_RVAhead, speed_RVAhead, a, sign, trajectoryPrediction::ActorType::RVAhead);
                  std::vector<trajectoryPrediction::TrajectoryItem> trajectory = std::get<1>(item);
                  std::tuple<bool, double> ret = trajectoryEvaluation (trajectory_HV, trajectory, leader_length, m_step_time, m_negotiation_time, m_time_to_lc, trajectoryPrediction::ActorType::RVAhead, start_time);
                  bool evaluation = std::get<0>(ret);
                  double start = std::get<1>(ret);
                  if (evaluation)
                    {
                      feasible_for_RVAhead = true;
                      acceleration_RVAhead = a;
                      trajectory_RVAhead = std::move(trajectory);
                      ref_RVAhead = std::get<0>(item);
                      if (start_time == -1)
                        {
                          // If there is not RV, store the start_time
                          start_time = start;
                        }
                      break;
                    }
                  a += ACCELERATION_STEP;
                }
            }
          else
            {
              feasible_for_RVAhead = true;
              acceleration_RVAhead = DEFAULT_ACC_VALUE;
            }

          // Prediction for HV Ahead, considering a constant acceleration during the deceleration time, then constant speed
          // Note that an acceleration of 0 is first used to consider the case in which no motion changes are required for RV Ahead
          // If present, the maneuver must be feasible for both RV and RVAhead first
          if(!HVAhead.empty() && feasible_for_RV && feasible_for_RVAhead)
            {
              double acceleration_supported = ACCEL_DECEL;
              double a = 0.0;
              // The leader in this case is HVAhead
              double leader_length = std::find_if(vehicles.begin(), vehicles.end(), [&id = HVAhead_id] (const LDM::returnedVehicleData_t& ldm_item) -> bool {return  ldm_item.vehData.stationID == id;})->vehData.vehicleLength.getData();
              leader_length /= DECI;
              // Starting with the least invasive acceleration for RVAhead (= 0)
              // Do multiple trial until we find a possible safe acceleration to apply
              while (a <= acceleration_supported)
                {
                  item = m_traj_predictor->predictConstantSpeed (x_HVAhead, y_HVAhead, speed_HVAhead, a, sign, trajectoryPrediction::ActorType::HVAhead);
                  std::vector<trajectoryPrediction::TrajectoryItem> trajectory = std::get<1>(item);
                  std::tuple<bool, double> ret = trajectoryEvaluation (trajectory_HV, trajectory, leader_length, m_step_time, m_negotiation_time, m_time_to_lc, trajectoryPrediction::ActorType::HVAhead, start_time);
                  bool evaluation = std::get<0>(ret);
                  double start = std::get<1>(ret);
                  if (evaluation)
                    {
                      feasible_for_HVAhead = true;
                      acceleration_HVAhead = a;
                      trajectory_HVAhead = std::move(trajectory);
                      ref_HVAhead = std::get<0>(item);
                      if (start_time == -1)
                        {
                          // If there are not RV and RVAhead, store the start_time
                          start_time = start;
                        }
                      break;
                    }
                  a += ACCELERATION_STEP;
                }
            }
          else
            {
              feasible_for_HVAhead = true;
              acceleration_HVAhead = DEFAULT_ACC_VALUE;
            }

          // If this condition is not verified, one or both the actors cannot perform the requested maneuver (it is not feasible for all of them)
          // The maneuver must not be executed
          if (feasible_for_RV && feasible_for_RVAhead && feasible_for_HVAhead)
            {
              if (deceleration_RV != DEFAULT_ACC_VALUE || acceleration_RVAhead != DEFAULT_ACC_VALUE || acceleration_HVAhead != DEFAULT_ACC_VALUE)
                {
                  // At least RV or RVAhead are present
                  // Insert in the data structure the new coordination event that is going to happen
                  (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
                  m_strategy.RV_acceleration = deceleration_RV;
                  m_strategy.RVAhead_acceleration = acceleration_RVAhead;
                  m_strategy.HVAhead_acceleration = acceleration_HVAhead;
                  Simulator::Schedule (MilliSeconds(0), std::bind(&foresee::startCoordination, this, RV_id, trajectory_RV, ref_RV, RVAhead_id, trajectory_RVAhead, ref_RVAhead, HVAhead_id, trajectory_HVAhead, ref_HVAhead, left_criterion));
                }
              else
                {
                  // The maneuver is feasible but there are not RV, neither RVAhead,neither HVAhead
                  // The coordination is not needed, manually change lane
                  target_lane = 3 - target_lane;
                  m_traci->vehicle.changeLane (m_vehicle_id, target_lane, m_time_to_lc);
                }
            }*/
        }
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

std::tuple<bool, double>
foresee::trajectoryEvaluation (std::vector<trajectoryPrediction::TrajectoryItem> trajectory_HV,
                               std::vector<trajectoryPrediction::TrajectoryItem> trajectory_other,
                               double leader_length,
                               int step_time,
                               int negotiation_time,
                               int lc_duration,
                               trajectoryPrediction::ActorType type,
                               int start_time)
{
  // The extensive evaluation must be done by RV ( to decide the start time)
  // In case the RV doesn't exist, the start_time would be -1
  // The evaluation will be performed for RVAhead/HVAhead
  if (type == trajectoryPrediction::ActorType::RV || start_time == -1)
    {
      size_t length = trajectory_HV.size();
      int i = 0;
      std::vector<std::tuple<double, bool>> ttc_over_time;
      while (i < length)
        {
          // Exclude the negotiation time from the evaluation
          int t = trajectory_HV[i].time.GetMilliSeconds();
          if (t >= negotiation_time)
            {
              // Remember: SUMO positions refer always to the front bumper of the vehicles
              // To get the distance between the leader back bumper and the follower front bumper, we remove the length of the leader
              double gap = std::max (std::abs (trajectory_HV[i].x - trajectory_other[i].x) - leader_length, 0.1);
              double delta_v = std::max (std::abs (trajectory_HV[i].speed - trajectory_other[i].speed), 0.1);
              double ttc = gap / delta_v;
              // Store the TTC condition predicted for the moment t
              ttc_over_time.push_back({t, ttc >= MIN_TTC});
            }
          t += step_time;
          i += 1;
        }
      double start_time = -1;
      bool possible = false;
      // We need to check whether there is a long enough window (based on lane change duration) to do the coordination safely
      for (auto it = ttc_over_time.begin(); it != ttc_over_time.end(); ++it)
        {
          double t = std::get<0> (*it);
          bool ttc = std::get<1> (*it);
          if (start_time == -1 && ttc)
            {
              // Store the start time of the window
              start_time = t;
            }
          else if (start_time != -1 && ttc)
            {
              // Start time is already present and the situation is safe
              // We need to check the length of the window
              double delta_t = t - start_time;
              if (delta_t >= lc_duration)
                {
                  // If the time window is exceeded, we found a favorable window for the lane change
                  possible = true;
                  break;
                }
            }
          else if (start_time != -1 && !ttc)
            {
              // The window is broken, we need to start again the window computation
              // Set again the start_time
              start_time = -1;
            }
          else if (start_time == -1 && !ttc)
            {
              continue;
            }
        }
      return {possible, start_time};
    }
  // In case the start_time has already been computed, do just a simple check on TTC at the estimated coordination time (i.e., start_time)
  else if (type == trajectoryPrediction::ActorType::RVAhead || type == trajectoryPrediction::ActorType::HVAhead)
    {
      trajectoryPrediction::TrajectoryItem HV = *std::find_if(trajectory_HV.begin(), trajectory_HV.end(), [&time = start_time] (const trajectoryPrediction::TrajectoryItem item) {return item.time.GetMilliSeconds() == time;});
      trajectoryPrediction::TrajectoryItem other = *std::find_if(trajectory_other.begin(), trajectory_other.end(), [&time = start_time] (const trajectoryPrediction::TrajectoryItem item) {return item.time.GetMilliSeconds() == time;});
      double gap = std::max (std::abs (HV.x - other.x) - leader_length, 0.1);
      double delta_v = std::max (std::abs (HV.speed - other.speed), 0.1);
      double ttc = gap / delta_v;
      if (ttc >= MIN_TTC)
        {
          return {true, 0};
        }
      else
        {
          return {false, 0};
        }
    }
}

void
foresee::startCoordination (long RV_id, std::vector<trajectoryPrediction::TrajectoryItem> trajectory_RV, trajectoryPrediction::TrajectoryItem ref_RV, long RVAhead_id, std::vector<trajectoryPrediction::TrajectoryItem> trajectory_RVAhead, trajectoryPrediction::TrajectoryItem ref_RVAhead, long HVAhead_id, std::vector<trajectoryPrediction::TrajectoryItem> trajectory_HVAhead, trajectoryPrediction::TrajectoryItem ref_HVAhead, bool left_criterion)
{
  MCSpecification specification;
  // Choose the container
  specification.setAdviseContainer();
  specification.setMCMItsRole (McmItssRole_coordinatingItss); // HV is the coordinator
  if (RV_id >= 0)
    {
      ManoeuvreAdvice adv = {};
      adv.executantID = static_cast<StationId_t>(RV_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification.create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv.currentStateAdvisedChange = csac;

      uint8_t total = trajectory_RV.size();
      libsumo::TraCIPosition ref = m_traci->simulation.convertXYtoLonLat(ref_RV.x, ref_RV.y);
      double prev_lat = ref.y, prev_lon = ref.x;
      for (uint8_t i = 0; i < total; i += TRAJECTORY_PER_SUBM)
        {
          Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
          subm->submanoeuvreId = ManeuverID::Slowdown;
          subm->advisedTrajectory = specification.create<Trajectory>();
          subm->advisedTrajectory->wayPointType = WayPointType_intermediateWayPoint;
          subm->advisedTargetRoadResource = nullptr;

          uint8_t end = std::min((uint8_t)(i + TRAJECTORY_PER_SUBM), total);
          for (uint8_t j = i; j < end; ++j)
            {
              // Speed
              Speed* sp = specification.create<Speed>();
              sp->speedValue = trajectory_RV[j].speed * CENTI;
              sp->speedConfidence = SpeedConfidence_unavailable;
              specification.add(asn_DEF_Speed, &subm->advisedTrajectory->speed, sp);

              // WayPoint
              PathPoint_t* wp = (PathPoint_t*) calloc(1, sizeof(PathPoint_t));
              wp->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
              libsumo::TraCIPosition pos = m_traci->simulation.convertXYtoLonLat(trajectory_RV[j].x, trajectory_RV[j].y);
              double wp_lon = pos.x;
              double wp_lat = pos.y;
              wp->pathPosition.deltaLatitude  = (long)((wp_lat - prev_lat) * 1e7);
              wp->pathPosition.deltaLongitude  = (long)((wp_lon - prev_lon) * 1e7);
              prev_lat = wp_lat;
              prev_lon = wp_lon;
              specification.add(asn_DEF_WayPoint, &subm->advisedTrajectory->wayPoints, wp);
            }
          specification.add(asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);
        }

      specification.pushManeuverAdvice(adv);

    }

  if (RVAhead_id >= 0)
    {
      ManoeuvreAdvice adv = {};
      adv.executantID = static_cast<StationId_t>(RVAhead_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification.create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv.currentStateAdvisedChange = csac;

      uint8_t total = trajectory_RV.size();
      libsumo::TraCIPosition ref = m_traci->simulation.convertXYtoLonLat(ref_RVAhead.x, ref_RVAhead.y);
      double prev_lat = ref.y, prev_lon = ref.x;
      for (uint8_t i = 0; i < total; i += TRAJECTORY_PER_SUBM)
        {
          Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
          subm->submanoeuvreId = ManeuverID::Slowdown;
          subm->advisedTrajectory = specification.create<Trajectory>();
          subm->advisedTrajectory->wayPointType = WayPointType_intermediateWayPoint;
          subm->advisedTargetRoadResource = nullptr;

          uint8_t end = std::min((uint8_t)(i + TRAJECTORY_PER_SUBM), total);
          for (uint8_t j = i; j < end; ++j)
            {
              // Speed
              Speed* sp = specification.create<Speed>();
              sp->speedValue = trajectory_RV[j].speed * CENTI;
              sp->speedConfidence = SpeedConfidence_unavailable;
              specification.add(asn_DEF_Speed, &subm->advisedTrajectory->speed, sp);

              // WayPoint
              PathPoint_t* wp = (PathPoint_t*) calloc(1, sizeof(PathPoint_t));
              wp->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
              libsumo::TraCIPosition pos = m_traci->simulation.convertXYtoLonLat(trajectory_RV[j].x, trajectory_RV[j].y);
              double wp_lon = pos.x;
              double wp_lat = pos.y;
              wp->pathPosition.deltaLatitude  = (long)((wp_lat - prev_lat) * 1e7);
              wp->pathPosition.deltaLongitude  = (long)((wp_lon - prev_lon) * 1e7);
              prev_lat = wp_lat;
              prev_lon = wp_lon;
              specification.add(asn_DEF_WayPoint, &subm->advisedTrajectory->wayPoints, wp);
            }
          specification.add(asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);
        }
      specification.pushManeuverAdvice(adv);
    }

  if (HVAhead_id >= 0)
    {
      ManoeuvreAdvice adv = {};
      adv.executantID = static_cast<StationId_t>(HVAhead_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification.create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv.currentStateAdvisedChange = csac;

      uint8_t total = trajectory_RV.size();
      libsumo::TraCIPosition ref = m_traci->simulation.convertXYtoLonLat(ref_HVAhead.x, ref_HVAhead.y);
      double prev_lat = ref.y, prev_lon = ref.x;
      for (uint8_t i = 0; i < total; i += TRAJECTORY_PER_SUBM)
        {
          Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
          subm->submanoeuvreId = ManeuverID::Slowdown;
          subm->advisedTrajectory = specification.create<Trajectory>();
          subm->advisedTrajectory->wayPointType = WayPointType_intermediateWayPoint;
          subm->advisedTargetRoadResource = nullptr;

          uint8_t end = std::min((uint8_t)(i + TRAJECTORY_PER_SUBM), total);
          for (uint8_t j = i; j < end; ++j)
            {
              // Speed
              Speed* sp = specification.create<Speed>();
              sp->speedValue = trajectory_RV[j].speed * CENTI;
              sp->speedConfidence = SpeedConfidence_unavailable;
              specification.add(asn_DEF_Speed, &subm->advisedTrajectory->speed, sp);

              // WayPoint
              PathPoint_t* wp = (PathPoint_t*) calloc(1, sizeof(PathPoint_t));
              wp->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
              libsumo::TraCIPosition pos = m_traci->simulation.convertXYtoLonLat(trajectory_RV[j].x, trajectory_RV[j].y);
              double wp_lon = pos.x;
              double wp_lat = pos.y;
              wp->pathPosition.deltaLatitude  = (long)((wp_lat - prev_lat) * 1e7);
              wp->pathPosition.deltaLongitude  = (long)((wp_lon - prev_lon) * 1e7);
              prev_lat = wp_lat;
              prev_lon = wp_lon;
              specification.add(asn_DEF_WayPoint, &subm->advisedTrajectory->wayPoints, wp);
            }
          specification.add(asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);
        }
      specification.pushManeuverAdvice(adv);
    }

  specification.setMCMConcept (0); // MCM Goal will be set
  specification.setMCMCost (0); // Default, it is not used for this use case
  specification.setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
  specification.setMCMType (McmType::McmType_request); // HV asks the others for a cooperation
  specification.setManeuverID (left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
  // FORESEE is designed for passenger cars and trucks
  specification.setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
  // The logic for the coordination process will be: Request -> ACK -> SYN ACK
  // TODO add a call at the end of negotiation time to check that all the map members answered affirmatively
  if(RV_id >= 0) m_acceptance_map[RV_id] = false;
  if(RVAhead_id >= 0) m_acceptance_map[RVAhead_id] = false;
  if(HVAhead_id >= 0) m_acceptance_map[HVAhead_id] = false;
  m_coordinator = true;
  m_mcs_ptr->generateAndEncodeMCM (&specification);
  // Free the CurrentStateAdvisedChange we allocated above
  // Simulator::Schedule(MilliSeconds(m_negotiation_time), &foresee::startCoordination, this);
  m_termination_event = Simulator::Schedule(MilliSeconds(m_FORESEE_max_time), &foresee::terminateCoordination, this);
  m_busy_with_maneuver = true;
}

void foresee::receiveMCM(asn1cpp::Seq<MCM> mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
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
          int subms_size = asn1cpp::sequenceof::getSize(adc);
          for(int i = 0; i < subms_size; ++i)
            {
              auto subms = asn1cpp::sequenceof::getSeq(adc, ManoeuvreAdvice, i);
              StationId_t id = subms->executantID;
              if (id == std::stol(m_vehicle_id.substr(3)))
                {
                  if (m_busy_with_maneuver)
                    {
                      break;
                    }
                  else
                    {
                      // TODO calculate the acceleration/deceleration based on trajectory
                      accept = true;
                    }
                }
            }
          if (accept)
            {
              // Accept the coordination
              MCSpecification specification;
              specification.setResponseContainer();
              specification.setMCMItsRole (McmItssRole_targetVehicle);
              specification.setMCMType(McmType::McmType_response);
              specification.setMCMResponse (0);
              m_busy_with_maneuver = true;
              m_mcs_ptr->generateAndEncodeMCM (&specification);
            }
          else
            {
              // Refuse the coordination
              MCSpecification specification;
              specification.setResponseContainer();
              specification.setMCMItsRole (McmItssRole_targetVehicle);
              specification.setMCMResponse (1);
              specification.setMCMType(McmType::McmType_response);
              m_mcs_ptr->generateAndEncodeMCM (&specification);
            }
        }
      break;
    case McmType::McmType_termination:
      // Termination reception, the vehicle is now free to do other coordinations
      m_busy_with_maneuver = false;
      break;
    case McmType::McmType_acknowledgment:
      // TODO
      break;
    case McmType::McmType_response:
      if (resp_container)
        {
          if (m_coordinator && sender_role == McmItssRole::McmItssRole_targetVehicle)
            {
              // The coordinator is waiting the ACK from the others
              ManouevreResponse_t response = resp.manouevreResponse;
              if (response == ManouevreResponse::ManouevreResponse_accept)
                {
                  // ACK received
                  // One of the vehicles accepted to be involved in the coordination
                  m_acceptance_map[sender] = true;
                  // Send a SYN-ACK
                  MCSpecification specification;
                  specification.setAcknowledgmentContainer();
                  specification.setMCMItsRole (McmItssRole_coordinatingItss);
                  specification.setMCMType (McmType::McmType_acknowledgment);
                  m_mcs_ptr->generateAndEncodeMCM (&specification);
                }
              else
                {
                  // Terminate the coordination in case of refuse
                  MCSpecification specification;
                  specification.setTerminatorContainer();
                  specification.setMCMItsRole (McmItssRole_coordinatingItss);
                  specification.setMCMType (McmType::McmType_cancellationRequest);
                  m_mcs_ptr->generateAndEncodeMCM (&specification);
                }
            }
        }
      break;
    case McmType::McmType_cancellationRequest:
      // TODO
      break;
    default:
      break;
    }
}

void
foresee::terminateCoordination ()
{
  // Clear the data structure after terminating the coordination
  (*m_lc_data_structure).erase(m_vehicle_id_int);
  // TODO send a coordination message to terminate
  m_busy_with_maneuver = false;
  // Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

}

