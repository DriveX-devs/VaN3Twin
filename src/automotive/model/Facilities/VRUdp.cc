#include "VRUdp.h"
#include <math.h>

extern "C" {
  #include "ns3/VAM.h"
}

namespace ns3
{
  VRUdp::VRUdp(){
    m_traci_client = NULL;
    m_id = "null";

    m_prev_speed = 0;
    m_prev_gen_time = 0;
    m_first_gen_time = 0;

    m_first_transmission = true;
    m_init_gen_time = true;

    m_compute_acceleration = true;
  }

  VRUdp::VRUdp(Ptr<TraciClient> mobility_client, std::string node_id){
    m_traci_client = NULL;
    m_id = "null";

    m_prev_speed = 0;
    m_prev_gen_time = 0;
    m_first_gen_time = 0;

    m_first_transmission = true;
    m_init_gen_time = true;

    m_compute_acceleration = true;

    m_traci_client = mobility_client;
    m_id = node_id;
  }

  VAM_mandatory_data_t VRUdp::getVAMMandatoryData(){
    VAM_mandatory_data_t VAMdata;
    int64_t now = computeTimestampUInt64 ();

    // Timestamp of the first transmission
    if(m_init_gen_time){
        m_first_gen_time = now;
        m_init_gen_time = false;
      }

    // Computation of the time interval since the last VAM transmitted
    double gen_interval = ((double) (now - m_prev_gen_time))/(NANO_TO_CENTI*CENTI);

    /* Speed [0.01 m/s] */
    VAMdata.speed = VRUdpValueConfidence<>(m_traci_client->TraCIAPI::person.getSpeed (m_id)*CENTI,
                                       SpeedConfidence_unavailable);

    /* Longitudinal acceleration [0.1 m/s^2] */
    if(m_compute_acceleration){
        if(m_first_transmission){
          VAMdata.longAcceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);
          m_first_transmission = false;
        } else{
          VAMdata.longAcceleration = VRUdpValueConfidence<>((VAMdata.speed.getValue()-m_prev_speed)*DECI/(now-m_prev_gen_time),
                                                          AccelerationConfidence_unavailable);
          }

        m_prev_speed = VAMdata.speed.getValue ();
        m_prev_gen_time = now;
      } else
      VAMdata.longAcceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos=m_traci_client->TraCIAPI::person.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    VAMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    VAMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    VAMdata.altitude = VRUdpValueConfidence<>(pos.z*CENTI,
                                          AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Heading WGS84 north [0.1 degree] */
    VAMdata.heading = VRUdpValueConfidence<>(m_traci_client->TraCIAPI::person.getAngle (m_id) * DECI,
                                         HeadingConfidence_unavailable);

    return VAMdata;
  }

  VRUdp_position_latlon_t VRUdp::getPedPosition(){
    VRUdp_position_latlon_t vrudppos;

    libsumo::TraCIPosition pos=m_traci_client->TraCIAPI::person.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    vrudppos.lat=pos.y;
    vrudppos.lon=pos.x;
    vrudppos.alt=DBL_MAX;

    return vrudppos;
  }

  VRUdpValueConfidence<> VRUdp::getLongAcceleration(){
    VRUdpValueConfidence <> longAcceleration;

    int64_t now = computeTimestampUInt64 ();
    double speed = m_traci_client->TraCIAPI::person.getSpeed (m_id)*CENTI;

    m_compute_acceleration = false;

    if(m_first_transmission){
        longAcceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);
        m_first_transmission = false;
      } else{
        longAcceleration = VRUdpValueConfidence<>((speed-m_prev_speed)*DECI/(now-m_prev_gen_time),
                                                              AccelerationConfidence_unavailable);
      }

    m_prev_gen_time = now;
    m_prev_speed = speed;
  }

  // Ported from OScar (src/VRUBasicService.cpp, VRUBasicService::get_min_distance())
  std::vector<distance_t> VRUdp::get_min_distance(Ptr<LDM> LDM, uint64_t ego_station_id, double long_safe_d, double lat_safe_d, double vert_safe_d){
    std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false});
    min_distance[0].station_type = StationType_pedestrian;
    min_distance[1].station_type = StationType_passengerCar;

    // For each type of station: whether the station currently stored in min_distance is inside the safe distances, and its absolute distance
    bool min_inside[2] = {false,false};
    double min_abs_distance[2] = {DBL_MAX,DBL_MAX};

    m_safe_dist_inside.clear();

    libsumo::TraCIPosition pos_node;
    std::vector<LDM::returnedVehicleData_t> selectedStations;

    // Get position and heading of the current pedestrian
    libsumo::TraCIPosition pos_ped = m_traci_client->TraCIAPI::person.getPosition(m_id);
    double ped_heading = m_traci_client->TraCIAPI::person.getAngle(m_id);

    // Extract all stations from the LDM
    VRUdp_position_latlon_t ped_pos = getPedPosition ();
    LDM->rangeSelect (MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

    // Iterate over all stations present in the LDM
    for(std::vector<LDM::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
        // Skip the ego station
        if(it->vehData.stationID == ego_station_id) {
            continue;
        }

        distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false};
        curr_distance.ID = it->vehData.stationID;
        curr_distance.station_type = it->vehData.stationType;

        // If a node has no defined heading, consider the heading of the ego node (VRU) instead of the one of the remote node (VRU or vehicle),
        // as done in OScar (here the heading of the ego VRU, coming from SUMO, is always available)
        double node_heading = it->vehData.heading;
        if(node_heading >= (double)HeadingValue_unavailable/DECI) {
            node_heading = ped_heading;
        }

        pos_node.x = it->vehData.lon;
        pos_node.y = it->vehData.lat;
        pos_node = m_traci_client->TraCIAPI::simulation.convertLonLattoXY (pos_node.x,pos_node.y);
        // The LDM stores the altitude in meters (AltitudeValue/CENTI): when it is unavailable (as in the CAMs generated by VaN3Twin),
        // the remote node is considered to be at the same height as the ego VRU
        if(it->vehData.elevation >= (double)AltitudeValue_unavailable/CENTI) {
            pos_node.z = pos_ped.z;
        } else {
            pos_node.z = it->vehData.elevation;
        }

        // Computation of the distances
        // According to ETSI TS 103 300-2 V2.1.1 (2020-05):
        // Longitudinal Distance (LoD): estimated distance between the VRU and the vehicle along the direction of the vehicle heading
        // Lateral Distance (LaD): estimated distance between the VRU and the vehicle perpendicular to the direction of the vehicle heading
        // Vertical Distance (VD): estimated distance in vertical direction (height) between the VRU and the vehicle
        double node_distance = sqrt((pos_node.x-pos_ped.x)*(pos_node.x-pos_ped.x)+(pos_node.y-pos_ped.y)*(pos_node.y-pos_ped.y));
        if(pos_ped.y==pos_node.y && pos_ped.x==pos_node.x) {
            curr_distance.longitudinal=0;
            curr_distance.lateral=0;
        } else {
            double angle = (M_PI/2.0)-DEG_2_RAD(node_heading)-atan2(pos_ped.y-pos_node.y,pos_ped.x-pos_node.x);
            curr_distance.longitudinal = std::abs(node_distance*cos(angle));
            curr_distance.lateral = std::abs(node_distance*sin(angle));
        }
        curr_distance.vertical = std::abs(pos_node.z - pos_ped.z);

        // Select the station to be stored for each type (index 0: pedestrians, index 1: all the other stations)
        // A station inside the safe distances (lateral, longitudinal and vertical thresholds satisfied simultaneously) is always
        // preferred to a station outside them, so that the safe distance condition is detected if at least one station satisfies it;
        // among the stations which are all inside (or all outside) the safe distances, the nearest one is kept
        int idx = (curr_distance.station_type == StationType_pedestrian) ? 0 : 1;
        bool curr_inside = curr_distance.lateral < lat_safe_d && curr_distance.longitudinal < long_safe_d && curr_distance.vertical < vert_safe_d;
        double curr_abs_distance = sqrt(curr_distance.lateral*curr_distance.lateral+curr_distance.longitudinal*curr_distance.longitudinal+curr_distance.vertical*curr_distance.vertical);

        // ETSI TS 103 300-3 V2.3.1, clause 6.4.1, condition 7: only the "new" stations which satisfy the safe distances condition
        // "after the lastly transmitted VAM" shall trigger a VAM. A station which was already inside the safe distances when the
        // last VAM was transmitted is thus not considered, until it is found outside the safe distances at the time of a VAM transmission
        if(curr_inside) {
            m_safe_dist_inside.insert(curr_distance.ID);
            if(m_safe_dist_reported.count(curr_distance.ID) > 0) {
                continue;
            }
        }

        if((curr_inside && !min_inside[idx]) || (curr_inside == min_inside[idx] && curr_abs_distance < min_abs_distance[idx])) {
            min_distance[idx].lateral = curr_distance.lateral;
            min_distance[idx].longitudinal = curr_distance.longitudinal;
            min_distance[idx].vertical = curr_distance.vertical;

            min_distance[idx].ID = curr_distance.ID;

            min_inside[idx] = curr_inside;
            min_abs_distance[idx] = curr_abs_distance;
        }
      }

    return min_distance;
  }

  int64_t VRUdp::computeTimestampUInt64()
  {
    int64_t int_tstamp=0;

    int_tstamp=Simulator::Now ().GetNanoSeconds ();

    return int_tstamp;
  }

  VRUdp::VRUDP_position_cartesian_t
  VRUdp::getXY(double lon, double lat)
  {
    VRUDP_position_cartesian_t vdppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::simulation.convertLonLattoXY (lon, lat);

    vdppos.x=pos.x;
    vdppos.y=pos.y;
    vdppos.z=pos.z;

    return vdppos;
  }

}
