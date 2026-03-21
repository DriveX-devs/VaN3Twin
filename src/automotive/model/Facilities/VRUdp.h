#ifndef VRUdp_h
#define VRUdp_h

#include "ns3/LDM.h"
#include "vdp.h"
#include "ns3/asn_utils.h"
#include <float.h>
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"
#include "ns3/traci-client.h"

extern "C" {
  #include "ns3/VAM.h"
}

namespace ns3
{ 
  template <class V = int, class C = int>
  class VRUdpValueConfidence
  {
      private:
        V m_value;
        C m_confidence;

      public:
        VRUdpValueConfidence() {}
        VRUdpValueConfidence(V value,C confidence):
          m_value(value), m_confidence(confidence) {}

        V getValue() {return m_value;}
        C getConfidence() {return m_confidence;}
  };

  typedef struct VRUdp_PosConfidenceEllipse {
    long semiMajorConfidence;
    long semiMinorConfidence;
    long semiMajorOrientation;
  } VRUdp_PosConfidenceEllipse_t;

  typedef struct VAM_mandatory_data {
    VRUdpValueConfidence<> speed;
    long longitude;
    long latitude;
    VRUdpValueConfidence<> altitude;
    VRUdp_PosConfidenceEllipse_t posConfidenceEllipse;
    VRUdpValueConfidence<> longAcceleration;
    VRUdpValueConfidence<> heading;
  } VAM_mandatory_data_t;

  typedef struct VRUdp_position_latlon {
    double lat,lon,alt;
  } VRUdp_position_latlon_t;

  typedef struct distance {
    double longitudinal,lateral,vertical;
    StationID_t ID;
    StationType_t station_type;
    bool safe_dist;
  } distance_t;

  class VRUdp
  {
  public:
    typedef struct VRUDP_position_cartesian {
      double x,y,z;
    } VRUDP_position_cartesian_t;

    VRUdp();
    VRUdp(Ptr<TraciClient> mobility_client, std::string node_id);

    Ptr<TraciClient> getTraciClient() {return m_traci_client;}

    virtual VAM_mandatory_data_t getVAMMandatoryData();

    virtual VRUdp_position_latlon_t getPedPosition();
    virtual double getPedSpeedValue() {return m_traci_client->TraCIAPI::person.getSpeed (m_id);}
    virtual double getPedHeadingValue() {return m_traci_client->TraCIAPI::person.getAngle (m_id);}
    virtual libsumo::TraCIPosition getPedPositionValue() {return m_traci_client->TraCIAPI::person.getPosition (m_id);}

    VRUdpValueConfidence<> getLongAcceleration();

    virtual std::vector<distance_t> get_min_distance(Ptr<LDM> LDM);

    virtual /**
     * @brief This function converts the pedestrian's position in lat/lon coordinates to cartesian coordinates.
     * @param lon The VRU longitude.
     * @param lat The VRU latitude.
     * @return
     */
    VRUDP_position_cartesian_t getXY(double lon, double lat);

  private:
    int64_t computeTimestampUInt64();

    std::string m_id;
    Ptr<TraciClient> m_traci_client;

    int64_t m_first_gen_time;
    int64_t m_prev_gen_time;
    double m_prev_speed;

    bool m_first_transmission;
    bool m_init_gen_time;

    bool m_compute_acceleration;
  };
}

#endif /* VRUdp_h */
