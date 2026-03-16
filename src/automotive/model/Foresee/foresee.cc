//
// Created by diego on 01/12/25.
//

#include "foresee.h"

int counter1 = 0;
int counter2 = 0;

namespace ns3
{
void
foresee::setTrajectoryPredictor (double horizon_time, double step_time, double negotiation_time,
                                 double deceleration_time, PredictionType prediction_type)
{
  m_traj_predictor = new trajectoryPrediction(horizon_time, step_time, negotiation_time, deceleration_time);
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
          std::vector<trajectoryPrediction::TrajectoryItem> mp_RV;
          std::vector<trajectoryPrediction::TrajectoryItem> mp_RVAhead;
          std::vector<trajectoryPrediction::TrajectoryItem> mp_HVAhead;
          std::vector<trajectoryPrediction::TrajectoryItem> mp_HV;
          int8_t sign = my_heading == 270 ? -1 : 1;
          // Do prediction for each actor, if present
          if(!RV.empty()){
              rv_type = m_traci->vehicle.getTypeID (RV);
              switch(m_prediction_type)
                {
                case CONSTANT_SPEED:
                  mp_RV = m_traj_predictor->predictConstantSpeed (x_RV, speed_RV, -m_traci->vehicletype.getDecel (rv_type), sign, true);
                  break;
                case CONSTANT_ACCELERATION:
                  break;
                default:
                  NS_ABORT_MSG ("FORESEE: Prediction type not declared.");
                  break;
                }
            }
          if(!RVAhead.empty()){
              rvahead_type = m_traci->vehicle.getTypeID (RVAhead);
              switch(m_prediction_type)
                {
                case CONSTANT_SPEED:
                  mp_RVAhead = m_traj_predictor->predictConstantSpeed (x_RVAhead, speed_RVAhead, -m_traci->vehicletype.getDecel (rvahead_type), sign);
                  break;
                case CONSTANT_ACCELERATION:
                  break;
                default:
                  NS_ABORT_MSG ("FORESEE: Prediction type not declared.");
                  break;
                }
              }
          if(!HVAhead.empty()){
              hvahead_type = m_traci->vehicle.getTypeID (HVAhead);
              switch(m_prediction_type)
                {
                case CONSTANT_SPEED:
                  mp_HVAhead = m_traj_predictor->predictConstantSpeed (x_HVAhead, speed_HVAhead, -m_traci->vehicletype.getDecel (hvahead_type), sign);
                  break;
                case CONSTANT_ACCELERATION:
                  break;
                default:
                  NS_ABORT_MSG ("FORESEE: Prediction type not declared.");
                  break;
                }
            }
          switch(m_prediction_type)
            {
            case CONSTANT_SPEED:
              mp_HV = m_traj_predictor->predictConstantSpeed (my_x, my_speed, -m_traci->vehicletype.getDecel (my_type), sign);
              break;
            case CONSTANT_ACCELERATION:
              break;
            default:
              NS_ABORT_MSG ("FORESEE: Prediction type not declared.");
              break;
            }
          double estimated_time_hv_rv = 0, estimated_time_hv_rvahead = 0;
          std::vector<double> accelerations_hv_rv, accelerations_hv_rvahead;
          if(!mp_RV.empty())
            {
              auto tuple = m_traj_predictor->estimateTimeFromPredictionIDM (
                  mp_HV,
                  mp_RV,
                  m_traci->vehicletype.getAccel (rv_type),
                  m_desired_speed,
                  -m_traci->vehicletype.getDecel (rv_type),
                  m_traci->vehicletype.getMinGap (rv_type),
                  m_traci->vehicletype.getTau (rv_type)
              );
              estimated_time_hv_rv = std::get<0>(tuple);
              accelerations_hv_rv = std::get<1>(tuple);
            }
          if (!mp_RVAhead.empty())
            {
              auto tuple = m_traj_predictor->estimateTimeFromPredictionIDM (
                  mp_RVAhead,
                  mp_HV,
                  m_traci->vehicletype.getAccel (my_type),
                  m_desired_speed,
                  -m_traci->vehicletype.getDecel (my_type),
                  m_traci->vehicletype.getMinGap (my_type),
                  m_traci->vehicletype.getTau (my_type)
              );
              estimated_time_hv_rvahead = std::get<0> (tuple);
              accelerations_hv_rvahead = std::get<1> (tuple);
            }
          if (estimated_time_hv_rv == -1 || estimated_time_hv_rvahead == -1){
              // TODO start a maneuver coordination
              // Insert in the data structure the new coordination event that is going to happen
              // (*m_lc_data_structure)[m_vehicle_id_int] = std::make_tuple (my_heading, my_x, my_y);
              // Simulator::Schedule (MilliSeconds(0), &foresee::doCoordination, this);
            }
          else
            {
              // TODO automatically move vehicle since it can change the lane without any issues
            }
        }
    }
  Simulator::Schedule (MilliSeconds(m_FORESEE_check_ms), &foresee::FORESEEMobilityModel, this);
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

}

