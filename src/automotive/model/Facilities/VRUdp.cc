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

  std::vector<distance_t> VRUdp::get_min_distance(Ptr<LDM> LDM){
    std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false});
    min_distance[0].station_type = StationType_pedestrian;
    min_distance[1].station_type = StationType_passengerCar;

    libsumo::TraCIPosition pos_node;
    std::vector<LDM::returnedVehicleData_t> selectedStations;

    // Get position and heading of the current pedestrian
    libsumo::TraCIPosition pos_ped = m_traci_client->TraCIAPI::person.getPosition(m_id);
    double ped_heading = m_traci_client->TraCIAPI::person.getAngle(m_id);
    ped_heading += (ped_heading>180.0) ? -360.0 : (ped_heading<-180.0) ? 360.0 : 0.0;

    // Extract all stations from the LDM
    VRUdp_position_latlon_t ped_pos = getPedPosition ();
    LDM->rangeSelect (MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

    // Iterate over all stations present in the LDM
    for(std::vector<LDM::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
        distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false};
        curr_distance.ID = it->vehData.stationID;
        curr_distance.station_type = it->vehData.stationType;

        pos_node.x = it->vehData.lon;
        pos_node.y = it->vehData.lat;
        pos_node = m_traci_client->TraCIAPI::simulation.convertLonLattoXY (pos_node.x,pos_node.y);
        pos_node.z = it->vehData.elevation;

        // Computation of the distances
        curr_distance.lateral = abs((pos_node.x - pos_ped.x)*cos(ped_heading));
        curr_distance.longitudinal = abs((pos_node.y - pos_ped.y)*cos(ped_heading));
        curr_distance.vertical = abs(pos_node.z - pos_ped.z);

        if(curr_distance.station_type == StationType_pedestrian){
            if(curr_distance.lateral<min_distance[1].lateral && curr_distance.longitudinal<min_distance[1].longitudinal && curr_distance.vertical < min_distance[1].vertical){
                min_distance[0].lateral = curr_distance.lateral;
                min_distance[0].longitudinal = curr_distance.longitudinal;
                min_distance[0].vertical = curr_distance.vertical;

                min_distance[0].ID = curr_distance.ID;
              }
          } else{
            if(curr_distance.lateral<min_distance[2].lateral && curr_distance.longitudinal<min_distance[2].longitudinal && curr_distance.vertical < min_distance[2].vertical){
                min_distance[1].lateral = curr_distance.lateral;
                min_distance[1].longitudinal = curr_distance.longitudinal;
                min_distance[1].vertical = curr_distance.vertical;

                min_distance[1].ID = curr_distance.ID;
              }
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
