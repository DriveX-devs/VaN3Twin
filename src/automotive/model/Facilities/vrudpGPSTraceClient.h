#ifndef VRUDPGPSTRACECLIENT_H
#define VRUDPGPSTRACECLIENT_H

#include "VRUdp.h"
#include "ns3/gps-tc.h"

namespace ns3 {
  class VRUDPGPSTraceClient : public VRUdp
  {
  public:
    VRUDPGPSTraceClient(Ptr<GPSTraceClient>,std::string);
    VRUDPGPSTraceClient();

    VAM_mandatory_data_t getVAMMandatoryData() override;

    libsumo::TraCIPosition getPedPositionValue() override {
      libsumo::TraCIPosition pos;
      pos.x = m_gps_trace_client->getX();
      pos.y = m_gps_trace_client->getY();
      pos.z = 0.0;
      return pos;
    }
    double getPedSpeedValue() override {return m_gps_trace_client->getSpeedms ();}
    double getPedHeadingValue() override {return m_gps_trace_client->getHeadingdeg ();}

    std::vector<distance_t> get_min_distance(Ptr<LDM> LDM, uint64_t ego_station_id, double long_safe_d, double lat_safe_d, double vert_safe_d) override {return std::vector<distance_t>();}

    // Added for GeoNet functionalities
    VRUdp_position_latlon_t getPedPosition() override;
    VRUdp::VRUDP_position_cartesian_t getXY(double lon, double lat) override;
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2){return 0.0;}

    private:
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VRUDPGPSTRACECLIENT_H
