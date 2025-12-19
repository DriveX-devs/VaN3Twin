//
// Created by carlosrisma on 09/12/24.
//

#ifndef VRUDPOPENCDA_H
#define VRUDPOPENCDA_H

#include "ns3/asn_utils.h"
#include <float.h>
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/OpenCDAClient.h"
#include "ns3/LDM.h"
#include "ns3/VRUdp.h"

extern "C" {
#include "ns3/VAM.h"
}

namespace ns3
{
//template <class V = int, class C = int>
//class VRUdpValueConfidence
//{
//private:
//    V m_value;
//    C m_confidence;
//
//public:
//    VRUdpValueConfidence() {}
//    VRUdpValueConfidence(V value,C confidence):
//            m_value(value), m_confidence(confidence) {}
//
//    V getValue() {return m_value;}
//    C getConfidence() {return m_confidence;}
//};
//
//typedef struct VRUdp_PosConfidenceEllipse {
//    long semiMajorConfidence;
//    long semiMinorConfidence;
//    long semiMajorOrientation;
//} VRUdp_PosConfidenceEllipse_t;
//
//typedef struct VAM_mandatory_data {
//    VRUdpValueConfidence<> speed;
//    long longitude;
//    long latitude;
//    VRUdpValueConfidence<> altitude;
//    VRUdp_PosConfidenceEllipse_t posConfidenceEllipse;
//    VRUdpValueConfidence<> longAcceleration;
//    VRUdpValueConfidence<> heading;
//} VAM_mandatory_data_t;
//
//typedef struct VRUdp_position_latlon {
//    double lat,lon,alt;
//} VRUdp_position_latlon_t;
//
//typedef struct distance {
//    double longitudinal,lateral,vertical;
//    StationID_t ID;
//    StationType_t station_type;
//    bool safe_dist;
//} distance_t;

class VRUdpOpenCDA: public Object
{
public:
    VRUdpOpenCDA();
    VRUdpOpenCDA(Ptr<OpenCDAClient> mobility_client, std::string node_id);

    //Ptr<TraciClient> getTraciClient() {return m_traci_client;}

    VAM_mandatory_data_t getVAMMandatoryData();

    VRUdp_position_latlon_t getPedPosition();
    double getPedSpeedValue() {return m_opencda_client->getSpeed (m_id);}
    double getPedHeadingValue() {return m_opencda_client->getHeading (m_id);}
    VDP::VDP_position_cartesian_t getPedPositionValue();

    VRUdpValueConfidence<> getLongAcceleration();

    std::vector<distance_t> get_min_distance(Ptr<LDM> LDM);

private:
    int64_t computeTimestampUInt64();

    std::string m_string_id;
    int m_id;
    Ptr<OpenCDAClient> m_opencda_client;

    int64_t m_first_gen_time;
    int64_t m_prev_gen_time;
    double m_prev_speed;

    bool m_first_transmission;
    bool m_init_gen_time;

    bool m_compute_acceleration;
};
}

#endif //CARLA_PB2_PY_VRUDPOPENCDA_H
