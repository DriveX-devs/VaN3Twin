//
// Created by victorcarvalho on 09/12/24.
//
#include <math.h>
#include "vrudpOpenCDA.h"
#include "ns3/OpenCDAClient.h"

extern "C" {
#include "ns3/VAM.h"
}

namespace ns3
{
    VRUdpOpenCDA::VRUdpOpenCDA(){
    m_opencda_client = NULL;
    m_id = 0;

    m_prev_speed = 0;
    m_prev_gen_time = 0;
    m_first_gen_time = 0;

    m_first_transmission = true;
    m_init_gen_time = true;

    m_compute_acceleration = true;
}

    VRUdpOpenCDA::VRUdpOpenCDA(Ptr<OpenCDAClient> mobility_client, std::string node_id){
    m_opencda_client = NULL;
    m_string_id = node_id;
    m_id = std::stoi (node_id);

    m_prev_speed = 0;
    m_prev_gen_time = 0;
    m_first_gen_time = 0;

    m_first_transmission = true;
    m_init_gen_time = true;

    m_compute_acceleration = true;

    m_opencda_client = mobility_client;
    m_id = std::stoi (node_id);
}

VAM_mandatory_data_t VRUdpOpenCDA::getVAMMandatoryData(){
    VAM_mandatory_data_t VAMdata;
    int64_t now = computeTimestampUInt64 ();
    carla::Actor actor = m_opencda_client->GetActorById(m_id);

    // Timestamp of the first transmission
    if(m_init_gen_time){
        m_first_gen_time = now;
        m_init_gen_time = false;
    }

    // Computation of the time interval since the last VAM transmitted
    double gen_interval = ((double) (now - m_prev_gen_time))/(NANO_TO_CENTI*CENTI);

    /* Speed [0.01 m/s] */
//    VAMdata.speed = VRUdpValueConfidence<>(m_traci_client->TraCIAPI::person.getSpeed (m_id)*CENTI,
//                                           SpeedConfidence_unavailable);
    VAMdata.speed = VRUdpValueConfidence<>(sqrt(pow(actor.speed ().x (),2) + pow(actor.speed ().y (),2))*CENTI,SpeedConfidence_unavailable);

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
//    libsumo::TraCIPosition pos=m_traci_client->TraCIAPI::person.getPosition(m_id);
//    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);
    auto lat = actor.latitude();
    auto lon = actor.longitude();

    auto geo = m_opencda_client->getGeo(actor.location().x(),actor.location().y());
    //geo.x = latitude

    // longitude WGS84 [0,1 microdegree]
    VAMdata.longitude=(Longitude_t)(lon*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    VAMdata.latitude=(Latitude_t)(lat*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    VAMdata.altitude = VRUdpValueConfidence<>(actor.transform().location().z()*CENTI,
                                              AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Heading WGS84 north [0.1 degree] */
//    VAMdata.heading = VRUdpValueConfidence<>(m_traci_client->TraCIAPI::person.getAngle (m_id) * DECI,
//                                             HeadingConfidence_unavailable);
    VAMdata.heading = VRUdpValueConfidence<>(actor.heading() * DECI,
                                             HeadingConfidence_unavailable);

    return VAMdata;
}

VRUdp_position_latlon_t VRUdpOpenCDA::getPedPosition(){
    VRUdp_position_latlon_t vrudppos;
    carla::Actor actor = m_opencda_client->GetActorById(m_id);

//    libsumo::TraCIPosition pos=m_traci_client->TraCIAPI::person.getPosition(m_id);
//    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);
    auto geo = m_opencda_client->getGeo(actor.location().x(),actor.location().y());
    vrudppos.lat=geo.x();
    vrudppos.lon=geo.y();
    vrudppos.alt=DBL_MAX;

    return vrudppos;
}

VRUdpValueConfidence<> VRUdpOpenCDA::getLongAcceleration(){
    VRUdpValueConfidence <> longAcceleration;
    carla::Actor actor = m_opencda_client->GetActorById(m_id);

    int64_t now = computeTimestampUInt64 ();
    //double speed = m_traci_client->TraCIAPI::person.getSpeed (m_id)*CENTI;
    double speed = sqrt(pow(actor.speed().x(),2)+pow(actor.speed().y(),2));

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

std::vector<distance_t> VRUdpOpenCDA::get_min_distance(Ptr<LDM> LDM){
    std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false});
    min_distance[0].station_type = StationType_pedestrian;
    min_distance[1].station_type = StationType_passengerCar;
    carla::Actor actor = m_opencda_client->GetActorById(m_id);

//    VDP::VDP_position_cartesian_t pos_node;
//    libsumo::TraCIPosition pos_node;
    std::vector<LDM::returnedVehicleData_t> selectedStations;

    // Get position and heading of the current pedestrian
//    libsumo::TraCIPosition pos_ped = m_traci_client->TraCIAPI::person.getPosition(m_id);
//    double ped_heading = m_traci_client->TraCIAPI::person.getAngle(m_id);
    auto pos_ped = actor.transform().location();
    double ped_heading = actor.heading();

    ped_heading += (ped_heading>180.0) ? -360.0 : (ped_heading<-180.0) ? 360.0 : 0.0;
    // Extract all stations from the LDM
    VRUdp_position_latlon_t ped_pos = getPedPosition ();
    LDM->rangeSelect (MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

    // Iterate over all stations present in the LDM
    for(std::vector<LDM::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
        distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false};
        curr_distance.ID = it->vehData.stationID;
        curr_distance.station_type = it->vehData.stationType;

        carla::Actor actor = m_opencda_client->GetActorById(it->vehData.stationID);
        //todo: some objects on the LDM does not have position as x,y and z
        auto pos_node = actor.transform().location();

        //pos_node = m_traci_client->TraCIAPI::simulation.convertLonLattoXY (pos_node.x,pos_node.y);


        // Computation of the distances
        auto dx = pos_node.x() - pos_ped.x();
        auto dy = pos_node.y() - pos_ped.y();
        curr_distance.lateral = abs(dx*sin(ped_heading) - dy*cos(ped_heading));
        curr_distance.longitudinal = abs(dx*cos(ped_heading) + dy*sin(ped_heading));
        curr_distance.vertical = abs(pos_node.z() - pos_ped.z());
        if (curr_distance.vertical > 5){
            curr_distance.vertical = 0.5;
        }

        if(curr_distance.station_type == StationType_pedestrian){
            if(curr_distance.lateral<min_distance[0].lateral && curr_distance.longitudinal<min_distance[0].longitudinal && curr_distance.vertical < min_distance[0].vertical){
                min_distance[0].lateral = curr_distance.lateral;
                min_distance[0].longitudinal = curr_distance.longitudinal;
                min_distance[0].vertical = curr_distance.vertical;

                min_distance[0].ID = curr_distance.ID;
            }
        } else{
            if(curr_distance.lateral<min_distance[1].lateral && curr_distance.longitudinal<min_distance[1].longitudinal && curr_distance.vertical < min_distance[1].vertical){
                min_distance[1].lateral = curr_distance.lateral;
                min_distance[1].longitudinal = curr_distance.longitudinal;
                min_distance[1].vertical = curr_distance.vertical;

                min_distance[1].ID = curr_distance.ID;
            }
        }
    }

    return min_distance;
}

VDP::VDP_position_cartesian_t VRUdpOpenCDA::getPedPositionValue(){
    carla::Actor actor = m_opencda_client->GetActorById(m_id);
    VDP::VDP_position_cartesian_t pos;
    pos.x = actor.location ().x ();
    pos.y = actor.location ().y ();
    pos.z = actor.location ().z ();

    return pos;
    }

int64_t VRUdpOpenCDA::computeTimestampUInt64()
{
    int64_t int_tstamp=0;

    int_tstamp=Simulator::Now ().GetNanoSeconds ();

    return int_tstamp;
}

}
