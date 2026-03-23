//
// Created by diego on 01/12/25.
//

#include "foresee.h"
#include "ns3/foresee.h"

namespace ns3
{
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
      long RV_id = -1, RVAhead_id = -1, HVAhead_id = -1;
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
          std::vector<trajectoryPrediction::TrajectoryItem> mp_HV;
          std::vector<trajectoryPrediction::TrajectoryItem> mp_RV;
          std::vector<trajectoryPrediction::TrajectoryItem> mp_RVAhead;
          int8_t sign = my_heading == 270 ? -1 : 1;
          bool feasible_for_RV = false, feasible_for_RVAhead = false, feasible_for_HVAhead = false;
          double deceleration_RV, acceleration_RVAhead;
          std::vector<trajectoryPrediction::TrajectoryItem> trajectory_RV, trajectory_RVAhead;
          // Do prediction for each actor, if present
          // Prediction for HV, considering a constant speed prediction model (minimum effort for HV)
          mp_HV = m_traj_predictor->predictConstantSpeed (my_x, my_speed, -m_traci->vehicletype.getDecel (my_type), sign, trajectoryPrediction::ActorType::HV);

          // Prediction for RV, considering a constant deceleration during the deceleration time, then constant speed
          // Note that a deceleration of 0 is first used to consider the case in which no motion changes are required for RV
          if(!RV.empty())
            {
              rv_type = m_traci->vehicle.getTypeID (RV);
              double deceleration_supported = -m_traci->vehicletype.getDecel (rv_type);
              double d = 0;
              // The leader in this case is HV
              double leader_length = m_traci->vehicletype.getLength (my_type);
              // Starting with the least invasive deceleration for RV (= 0)
              // Do multiple trial until we find a possible safe deceleration to apply
              while (d >= deceleration_supported)
                {
                  std::vector<trajectoryPrediction::TrajectoryItem> trajectory = m_traj_predictor->predictConstantSpeed (x_RV, speed_RV, d, sign, trajectoryPrediction::ActorType::RV);
                  bool evaluation = trajectoryEvaluation (mp_HV, trajectory, leader_length, m_step_time, m_negotiation_time, m_time_to_lc);
                  if (evaluation)
                    {
                      feasible_for_RV = true;
                      deceleration_RV = d;
                      trajectory_RV = std::move(trajectory);
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
          if(!RVAhead.empty())
            {
              rvahead_type = m_traci->vehicle.getTypeID (RVAhead);
              double acceleration_supported = m_traci->vehicletype.getAccel (rvahead_type);
              double a = 0.0;
              // The leader in this case is RVAhead
              double leader_length = m_traci->vehicletype.getLength (rvahead_type);
              // Starting with the least invasive acceleration for RVAhead (= 0)
              // Do multiple trial until we find a possible safe acceleration to apply
              while (a <= acceleration_supported)
                {
                  std::vector<trajectoryPrediction::TrajectoryItem> trajectory = m_traj_predictor->predictConstantSpeed (x_RVAhead, speed_RVAhead, a, sign, trajectoryPrediction::ActorType::RVAhead);
                  bool evaluation = trajectoryEvaluation (mp_HV, trajectory, leader_length, m_step_time, m_negotiation_time, m_time_to_lc);
                  if (evaluation)
                    {
                      feasible_for_RVAhead = true;
                      acceleration_RVAhead = a;
                      trajectory_RVAhead = std::move(trajectory);
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

          // Prediction for HV Ahead, considering constant speed
          if(!HVAhead.empty())
            {
              double delta_v = std::abs(my_speed - speed_HVAhead);
              double current_ttc = min_dist_hv_ahead / delta_v;
              if (current_ttc >= MIN_TTC)
                {
                  feasible_for_HVAhead = true;
                }
              else
                {
                  feasible_for_HVAhead = false;
                }
            }
          else
            {
              feasible_for_HVAhead = true;
            }


          // If this condition is not verified,one or both the actors cannot perform the requested maneuver, it is not feasible for them
          // The maneuver must not be executed
          if (feasible_for_RV && feasible_for_RVAhead && feasible_for_HVAhead)
            {
              if (deceleration_RV != DEFAULT_ACC_VALUE || acceleration_RVAhead != DEFAULT_ACC_VALUE)
                {
                  // At least RV or RVAhead are present
                  // Insert in the data structure the new coordination event that is going to happen
                  (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
                  uint8_t maneuver_id = left_criterion ? 12 : 13;
                  Simulator::Schedule (MilliSeconds(0), &foresee::startCoordination, this, RV_id, RVAhead_id, HVAhead_id, left_criterion);
                }
              else
                {
                  // The maneuver is feasible but there is no RV neither RVAhead
                  // The coordination is not needed, manually change lane
                  target_lane = 3 - target_lane;
                  m_traci->vehicle.changeLane (m_vehicle_id, target_lane, m_time_to_lc);
                }
            }
        }
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

void
foresee::startCoordination (long RV_id, long RVAhead_id, long HVAhead_id, bool left_criterion)
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

      // Allocate and fill the submanoeuvre
      Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
      subm->submanoeuvreId = ManeuverID::Slowdown;
      subm->advisedTargetRoadResource = nullptr;
      subm->advisedTrajectory = nullptr;

      // Add it to adv.submaneuvres directly — no need for a separate Submanoeuvres allocation
      specification.add (asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);

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

      // Allocate and fill the submanoeuvre
      Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
      subm->submanoeuvreId = ManeuverID::Accelerate;
      subm->advisedTargetRoadResource = nullptr;
      subm->advisedTrajectory = nullptr;

      // Add it to adv.submaneuvres directly — no need for a separate Submanoeuvres allocation
      specification.add (asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);

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

      // Allocate and fill the submanoeuvre
      Submanoeuvre_t* subm = specification.create<Submanoeuvre_t>();
      subm->submanoeuvreId = ManeuverID::Undefined;
      subm->advisedTargetRoadResource = nullptr;
      subm->advisedTrajectory = nullptr;

      // Add it to adv.submaneuvres directly — no need for a separate Submanoeuvres allocation
      specification.add (asn_DEF_Submanoeuvre, &adv.submaneuvres, subm);

      specification.pushManeuverAdvice(adv);
    }

  specification.setMCMConcept (0); // MCM Goal will be set
  specification.setMCMCost (0); // Default, it is not used for this use case
  specification.setMCMGoal (ManoeuvreCooperationGoal_localTrafficManagement); // FORESEE manages local traffic interactions
  specification.setMCMType (McmType_request); // HV asks the others for a cooperation
  specification.setManeuverID (left_criterion ? ManeuverID::GoToLeftLane : ManeuverID::GoToRightLane);
  // FORESEE is designed for passenger cars and trucks
  specification.setVehicleType (m_station_type == StationType_passengerCar ? Iso3833VehicleType_passengerCar : Iso3833VehicleType_truckStationWagon);
  // The logic for the coordination process will be: Request -> ACK -> SYN ACK
  m_mcs_ptr->generateAndEncodeMCM (&specification);
  // Free the CurrentStateAdvisedChange we allocated above
  // Simulator::Schedule(MilliSeconds(m_negotiation_time), &foresee::startCoordination, this);
  m_termination_event = Simulator::Schedule(MilliSeconds(m_FORESEE_max_time), &foresee::terminateCoordination, this);
  m_maneuver_execution = true;
}

void
foresee::terminateCoordination ()
{
  // Clear the data structure after terminating the coordination
  (*m_lc_data_structure).erase(m_vehicle_id_int);
  // Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
}

bool
foresee::trajectoryEvaluation (std::vector<trajectoryPrediction::TrajectoryItem> trajectory_HV,
                               std::vector<trajectoryPrediction::TrajectoryItem> trajectory_other,
                               double leader_length,
                               double step_time,
                               double negotiation_time,
                               double lc_duration)
{
  size_t length = trajectory_HV.size();
  int i = 0;
  std::vector<std::tuple<double, bool>> ttc_over_time;
  while (i < length)
    {
      // Exclude the negotiation time from the evaluation
      double t = trajectory_HV[i].time;
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
  return possible;
}

void foresee::receiveMCM(asn1cpp::Seq<MCM> mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  std::cout<< "Received" << std::endl;
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

}

