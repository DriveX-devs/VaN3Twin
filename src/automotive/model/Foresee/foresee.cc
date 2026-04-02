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

std::tuple<double, double> foresee::computeRequiredAcceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on acceleration of leader in [0, a_max]
  double lo = 0.0;
  double hi = p.a; // max acceleration
  double delta_t;
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double a_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap   = current_gap;
      double v_f = speed_follower;
      double v_l  = speed_leader;
      double a_f_final = -200;
      double t = 0;
      for(; t < horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
          if (a_f >= MIN_DECELERATION)
            {
              a_f_final = a_f;
              delta_t = t - dt;
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
  if(std::abs(hi - (-p.a)) < 0.01)
    return {NO_SOLUTION, -1};
  return {hi, delta_t}; // minimum acceleration RVAhead needs to apply
}

std::tuple<double, double> foresee::computeRequiredDeceleration(double speed_leader, double speed_follower,
                                      double current_gap, IDMParams p,
                                      double dt, double horizon)
{
  // Binary search on deceleration of follower in [0, a_max]
  double lo = -p.a;
  double hi = 0.0; // max deceleration
  double delta_t;
  for(int iter = 0; iter < MAX_LOOPS; iter++)
    {
      double d_candidate = (lo + hi) / 2.0;
      // Simulate gap evolution over horizon
      double gap = current_gap;
      double v_f = speed_follower;
      double v_l = speed_leader;
      double a_f_final = -200;
      double t = 0;
      for(; t < horizon; t += dt)
        {
          // Follower IDM behind the leader
          double a_f = idmAcceleration(v_f, v_l, gap,
                                        p.v0, p.T, p.s0, p.a, p.b);
          if (a_f >= MIN_DECELERATION)
            {
              a_f_final = a_f;
              delta_t = t - dt;
              break;
            }
          // Follower decelerates with candidate deceleration
          v_f  = std::min(v_f + d_candidate * dt, 0.0);
          // Update gap
          gap  += (v_l - v_f) * dt;
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
  if(std::abs(lo - (-p.a)) < 0.01)
    return {NO_SOLUTION, -1};
  return {lo, delta_t}; // minimum deceleration
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
foresee::setTrajectoryPredictor (double horizon_time, double step_time, double negotiation_time,
                                 double deceleration_time, double lc_duration, PredictionType prediction_type)
{
  m_traj_predictor = new trajectoryPrediction(horizon_time, step_time, negotiation_time, deceleration_time);
  m_step_time = step_time;
  m_negotiation_time = negotiation_time;
  m_time_to_lc = lc_duration;
  m_prediction_type = prediction_type;
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

          if (RV_id <= 0 && RVAhead_id <= 0)
            {
              // No RV and RVAhead present, we can directly change the lane
              target_lane = 3 - target_lane;
              m_traci->vehicle.changeLane (m_vehicle_id, target_lane, 0.5);
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
                      std::tuple<double, double> tuple = computeRequiredAcceleration (speed_RVAhead, my_speed, min_dist_rv_ahead, ego_params);
                      double a = std::get<0>(tuple);
                      double time = std::get<1>(tuple);
                      if (a != NO_SOLUTION)
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
                      std::tuple<double, double> tuple = computeRequiredDeceleration (my_speed, speed_RV, min_dist_rv, params);
                      double a = std::get<0>(tuple);
                      double time = std::get<1>(tuple);
                      if (a != NO_SOLUTION)
                        {
                          dec_rv = a;
                          time_rv = time;
                          possible_rv = true;
                        }
                    }
                }
              if ((possible_hv && acc_rv_ahead == DEFAULT_ACC_VALUE) && (possible_rv && dec_rv == DEFAULT_ACC_VALUE))
                {
                  // No need to change the motion of RV and RVAhead, we can directly change lane
                  target_lane = 3 - target_lane;
                  m_traci->vehicle.changeLane (m_vehicle_id, target_lane, 0.5);
                }
              else
                {
                  // Need a coordination
                  (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
                  startCoordination(RV_id, RVAhead_id, dec_rv, acc_rv_ahead, time_rv, time_rv_ahead, left_criterion);
                }

            }
        }
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

void
foresee::startCoordination (long RV_id, long RVAhead_id, double dec_rv, double acc_rv_ahead, double time_rv, double time_rv_ahead, bool left_criterion)
{
  MCSpecification specification;
  // Choose the container
  specification.setAdviseContainer();
  specification.setMCMItsRole (McmItssRole_coordinatingItss); // HV is the coordinator
  if (RV_id >= 0)
    {
      ManoeuvreAdvice* adv = specification.create<ManoeuvreAdvice>();
      adv->executantID = static_cast<StationId_t>(RV_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification.create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv->currentStateAdvisedChange = csac;

      Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
      asn1cpp::setField(subm->submanoeuvreId, ManeuverID::Slowdown);
      asn1cpp::setField(subm->acceleration.longitudinalAccelerationValue, dec_rv * DECI);
      asn1cpp::setField(subm->acceleration.longitudinalAccelerationConfidence, AccelerationConfidence::AccelerationConfidence_unavailable);
      asn1cpp::setField(subm->durationDeltaTime, time_rv);
      subm->advisedTrajectory = nullptr;
      subm->advisedTargetRoadResource = nullptr;
      specification.add(asn_DEF_Submanoeuvre, &adv->submaneuvres, subm);
      specification.pushManeuverAdvice(adv);

    }

  if (RVAhead_id >= 0)
    {
      ManoeuvreAdvice* adv = specification.create<ManoeuvreAdvice>();
      adv->executantID = static_cast<StationId_t>(RVAhead_id);

      // Allocate the CurrentStateAdvisedChange before filling it
      CurrentStateAdvisedChange* csac = specification.create<CurrentStateAdvisedChange> ();
      csac->present = CurrentStateAdvisedChange_PR_stayInLane;
      csac->choice.stayInLane = 1;
      adv->currentStateAdvisedChange = csac;

      Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
      asn1cpp::setField(subm->submanoeuvreId, ManeuverID::Accelerate);
      asn1cpp::setField(subm->acceleration.longitudinalAccelerationValue, acc_rv_ahead * CENTI);
      asn1cpp::setField(subm->acceleration.longitudinalAccelerationConfidence, AccelerationConfidence::AccelerationConfidence_unavailable);
      asn1cpp::setField(subm->durationDeltaTime, time_rv_ahead * CENTI);
      subm->advisedTrajectory = nullptr;
      subm->advisedTargetRoadResource = nullptr;
      specification.add(asn_DEF_Submanoeuvre, &adv->submaneuvres, subm);
      specification.pushManeuverAdvice(adv);
    }

  specification.setMCMConcept (0); // MCM Goal will be set
  specification.setMCMCost (0); // Default, it is not used for this use case
  specification.setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
  specification.setMCMType (McmType::McmType_request); // HV asks the others for a cooperation
  specification.setManeuverID (left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
  // FORESEE is designed for passenger cars and trucks
  specification.setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
  if(RV_id >= 0) m_acceptance_map[RV_id] = {false, dec_rv, time_rv};
  if(RVAhead_id >= 0) m_acceptance_map[RVAhead_id] = {false, acc_rv_ahead, time_rv_ahead};
  m_coordinator = true;
  m_busy_with_maneuver = true;
  m_mcs_ptr->generateAndEncodeMCM (&specification);
  Simulator::Schedule(MilliSeconds(m_negotiation_time), &foresee::negotiationPhase, this, left_criterion);
  // m_termination_event = Simulator::Schedule(MilliSeconds(m_FORESEE_max_time), &foresee::terminateCoordination, this);
}

void foresee::negotiationPhase(bool left_criterion)
{
  bool its_ok = true;
  std::vector<double> times;
  for (auto it : m_acceptance_map)
    {
      // Found an actor that doesn't want to coordinate
      if(!it.second.accepted) {its_ok = false; break;}
      times.push_back(it.second.time);
    }
  if(its_ok)
    {
      // Start the maneuver
      // First send the ACK to the others
      MCSpecification specification;
      specification.setAcknowledgmentContainer();
      specification.setMCMItsRole (McmItssRole_coordinatingItss);
      specification.setMCMType (McmType::McmType_acknowledgment);
      specification.setMCMConcept (0); // MCM Goal will be set
      specification.setMCMCost (0); // Default, it is not used for this use case
      specification.setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
      specification.setManeuverID(left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
      specification.setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
      m_mcs_ptr->generateAndEncodeMCM (&specification);
      // TODO here coordinate by looking the time needed
      // TODO start the execution phase from here
      double time_to_wait = *std::max_element(times.begin(), times.end());
    }
  else
    {
      // Cancel the maneuver
      MCSpecification specification;
      specification.setTerminatorContainer();
      specification.setMCMItsRole (McmItssRole_coordinatingItss);
      specification.setMCMType (McmType::McmType_termination);
      specification.setMCMConcept (0); // MCM Goal will be set
      specification.setMCMCost (0); // Default, it is not used for this use case
      specification.setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
      specification.setManeuverID(left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
      specification.setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
      m_mcs_ptr->generateAndEncodeMCM (&specification);
      m_busy_with_maneuver = false;
      m_coordinator = false;
      (*m_lc_data_structure).erase(m_vehicle_id_int);
    }
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
                  // Response received
                  // One of the vehicles accepted to be involved in the coordination
                  m_acceptance_map[sender].accepted = true;
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

}

