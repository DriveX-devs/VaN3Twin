#ifndef VDPGPSTRACECLIENT_H
#define VDPGPSTRACECLIENT_H

#include "vdp.h"
#include "ns3/gps-tc.h"

namespace ns3 {
  class VDPGPSTraceClient : public VDP
  {
  public:
    VDPGPSTraceClient(Ptr<GPSTraceClient>,std::string);
    VDPGPSTraceClient();

    CAM_mandatory_data_t getCAMMandatoryData();
    CPM_mandatory_data_t getCPMMandatoryData();
    MCM_mandatory_data_t getMCMMandatoryData();

    double getSpeedValue() {return m_gps_trace_client->getSpeedms ();}
    double getAccelerationValue() {return m_gps_trace_client->getAccelmsq();}
    double getTravelledDistance() {return m_gps_trace_client->getTravelledDistance ();}
    double getHeadingValue() {return m_gps_trace_client->getHeadingdeg ();}
    double getVehicleLength() {return 0;}

    // Added for GeoNet functionalities
    VDP_position_latlon_t getPosition();
    VDP_position_cartesian_t getPositionXY();
    VDP_position_cartesian_t getXY(double lon, double lat);
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2){return 0.0;}

    VDPDataItem<uint8_t> getAccelerationControl() {return VDPDataItem<uint8_t>(false);}
    VDPDataItem<int> getLanePosition() {return VDPDataItem<int>(false);}
    VDPDataItem<VDPValueConfidence<int,int>> getSteeringWheelAngle() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
    VDPDataItem<VDPValueConfidence<int,int>> getLateralAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
    VDPDataItem<VDPValueConfidence<int,int>> getVerticalAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
    VDPDataItem<int> getPerformanceClass() {return VDPDataItem<int>(false);}
    VDPDataItem<VDP_CEN_DSRC_tolling_zone_t> getCenDsrcTollingZone() {return VDPDataItem<VDP_CEN_DSRC_tolling_zone_t>(false);}

    VDPDataItem<unsigned int> getVehicleRole() {return VDPDataItem<unsigned int>(false);}
    VDPDataItem<uint8_t> getExteriorLights() {return VDPDataItem<uint8_t>(false);}
    VDPDataItem<VDP_PublicTransportContainerData_t> getPublicTransportContainerData() {return VDPDataItem<VDP_PublicTransportContainerData_t>(false);}
    VDPDataItem<VDP_SpecialTransportContainerData_t> getSpecialTransportContainerData() {return VDPDataItem<VDP_SpecialTransportContainerData_t>(false);}
    VDPDataItem<int> getDangerousGoodsBasicType() {return VDPDataItem<int>(false);}
    VDPDataItem<VDP_RoadWorksContainerBasicData_t> getRoadWorksContainerBasicData_t() {return VDPDataItem<VDP_RoadWorksContainerBasicData_t>(false);}
    VDPDataItem<uint8_t> getRescueContainerLightBarSirenInUse() {return VDPDataItem<uint8_t>(false);}
    VDPDataItem<VDP_EmergencyContainerData_t> getEmergencyContainerData() {return VDPDataItem<VDP_EmergencyContainerData_t>(false);}
    VDPDataItem<VDP_SafetyCarContainerData_t> getSafetyCarContainerData() {return VDPDataItem<VDP_SafetyCarContainerData_t>(false);}

    private:
      std::string m_id;
      Ptr<GPSTraceClient> m_gps_trace_client;
  };
}

#endif // VDPGPSTRACECLIENT_H
