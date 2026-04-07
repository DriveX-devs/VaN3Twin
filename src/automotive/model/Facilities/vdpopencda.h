#ifndef VDPOPENCDA_H
#define VDPOPENCDA_H

#include "vdp.h"
#include "ns3/OpenCDAClient.h"



namespace ns3 {
  class VDPOpenCDA : public VDP
  {
  public:
    VDPOpenCDA(Ptr<OpenCDAClient>, std::string);
    VDPOpenCDA();
    CAM_mandatory_data_t getCAMMandatoryData();
    CPM_mandatory_data_t getCPMMandatoryData();
    MCM_mandatory_data_t getMCMMandatoryData();

    double getSpeedValue() {return m_opencda_client->getSpeed (m_id);}
    double getTravelledDistance() {return 0;}
    double getHeadingValue() {return m_opencda_client->getHeading (m_id);}
    double getVehicleLength() {return 0;}

    VDP_position_latlon_t getPosition();
    VDP_position_cartesian_t getPositionXY();
    VDP_position_cartesian_t getXY(double lon, double lat);
    double getCartesianDist (double lon1, double lat1, double lon2, double lat2);

    VDPDataItem<uint8_t> getAccelerationControl() {return VDPDataItem<uint8_t>(false);}
    VDPDataItem<int> getLanePosition();
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
      std::string m_string_id;
      int m_id;
      Ptr<OpenCDAClient> m_opencda_client;

      int m_length;
      int m_width;

  };

}

#endif // VDPOPENCDA_H
