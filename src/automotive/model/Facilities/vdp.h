#ifndef VDP_H
#define VDP_H

#include "asn_utils.h"
#include <float.h>
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"

extern "C" {
  #include "ns3/CAM.h"
}

namespace ns3
{
  template <class T>
  class VDPDataItem
  {
      private:
        bool m_available;
        T m_dataitem;

      public:
        VDPDataItem(T data): m_dataitem(data) {m_available=true;}
        VDPDataItem(bool availability) {m_available=availability;}
        VDPDataItem() {m_available=false;}
        T getData() {return m_dataitem;}
        bool isAvailable() {return m_available;}
        T setData(T data) {m_dataitem=data; m_available=true;}
  };

  template <class V = int, class C = int>
  class VDPValueConfidence
  {
      private:
        V m_value;
        C m_confidence;

      public:
        VDPValueConfidence() {}
        VDPValueConfidence(V value,C confidence):
          m_value(value), m_confidence(confidence) {}

        V getValue() {return m_value;}
        C getConfidence() {return m_confidence;}
        void setValue(V value) {m_value=value;}
        void setConfidence(C confidence) {m_confidence=confidence;}
  };

  /**
   * \ingroup automotive
   *
   * \brief This class provides the interface for the Vehicle Data Provider (VDP).
   *
   * This class encapsulates various data structures representing different aspects of vehicle data and its processing.
   */
  class VDP
  {
    public:
    /**
     * @struct VDP_PosConfidenceEllipse
     * @brief Represents the positional confidence ellipse for vehicle data processing.
     *
     * @var long VDP_PosConfidenceEllipse::semiMajorConfidence
     * Semi-major axis confidence.
     *
     * @var long VDP_PosConfidenceEllipse::semiMinorConfidence
     * Semi-minor axis confidence.
     *
     * @var long VDP_PosConfidenceEllipse::semiMajorOrientation
     * Orientation of the semi-major axis.
     */
      typedef struct VDP_PosConfidenceEllipse {
        long semiMajorConfidence;
        long semiMinorConfidence;
        long semiMajorOrientation;
      } VDP_PosConfidenceEllipse_t;

    /**
     * @struct CAM_mandatory_data
     * @brief Struct representing the mandatory data in a CAM message.
     *
     * This includes various parameters like speed, position, acceleration, heading, etc.
     */
      typedef struct CAM_mandatory_data {
        VDPValueConfidence<> speed;
        long longitude;
        long latitude;
        long lane;
        VDPValueConfidence<> altitude;
        VDP_PosConfidenceEllipse_t posConfidenceEllipse;
        VDPValueConfidence<> longAcceleration;
        VDPValueConfidence<> heading;
        int driveDirection; // enum
        VDPValueConfidence<> curvature;
        int curvature_calculation_mode; // enum
        VDPValueConfidence<long,long> VehicleLength;
        int VehicleWidth;
        VDPValueConfidence<> yawRate;
      } CAM_mandatory_data_t;

      /**
     * @struct MCM_mandatory_data
     * @brief Struct representing the mandatory data in a MCM message.
     *
     * This includes various parameters like speed, position, acceleration, heading, etc.
     */
      typedef struct MCM_mandatory_data {
        VDPValueConfidence<> speed;
        long longitude;
        long latitude;
        long lane;
        VDPValueConfidence<> altitude;
        VDP_PosConfidenceEllipse_t posConfidenceEllipse;
        VDPValueConfidence<> longAcceleration;
        VDPValueConfidence<> heading;
        int driveDirection; // enum
        VDPValueConfidence<> curvature;
        int curvature_calculation_mode; // enum
        VDPValueConfidence<long,long> VehicleLength;
        int VehicleWidth;
        VDPValueConfidence<> yawRate;
      } MCM_mandatory_data_t;

    /**
     * @struct CPM_mandatory_data
     * @brief Mandatory data for Collective Perception Messages.
     *
     * Similar to CAM_mandatory_data, but used for collective perception.
     */
      typedef struct CPM_mandatory_data {
        VDPValueConfidence<> speed;
        long longitude;
        long latitude;
        VDPValueConfidence<> altitude;
        VDP_PosConfidenceEllipse_t posConfidenceEllipse;
        VDPValueConfidence<> longAcceleration;
        VDPValueConfidence<> heading;
        int driveDirection; // enum
        VDPValueConfidence<> curvature;
        int curvature_calculation_mode; // enum
        VDPValueConfidence<long,long> VehicleLength;
        int VehicleWidth;
        VDPValueConfidence<> yawRate;
      } CPM_mandatory_data_t;

    /**
     * @struct VDP_position_cartesian
     * @brief Represents a position in Cartesian coordinates.
     *
     * Describes a vehicle's position in a 3D Cartesian coordinate system.
     */
      typedef struct VDP_position_cartesian {
        double x,y,z;
      } VDP_position_cartesian_t;

    /**
     * @struct VDP_position_latlon
     * @brief Represents a geographic position.
     *
     * Describes a vehicle's position in terms of latitude, longitude, and altitude.
     */
      typedef struct VDP_position_latlon {
        double lat,lon,alt;
      } VDP_position_latlon_t;

    /**
     * @struct VDP_CEN_DSRC_tolling_zone
     * @brief Represents a CEN DSRC tolling zone.
     *
     * Contains data about a tolling zone as defined in the CEN DSRC standards.
     */
      typedef struct VDP_CEN_DSRC_tolling_zone {
        long latitude;
        long longitude;
        unsigned long cenDsrcTollingZoneID;
      } VDP_CEN_DSRC_tolling_zone_t;

      typedef struct VDP_ProtectedCommunicationsZonesRSU{
        long protectedZoneType;
        VDPDataItem<int> expiryTime	/* OPTIONAL */;
        long protectedZoneLatitude;
        long protectedZoneLongitude;
        VDPDataItem<long> protectedZoneRadius	/* OPTIONAL */;
        VDPDataItem<long> protectedZoneID	/* OPTIONAL */;
      } VDP_ProtectedCommunicationsZonesRSU_t;

      typedef struct VDP_PublicTransportContainerData {
        int embarkationStatus;
        VDPDataItem<uint32_t> ptActivationData;
        VDPDataItem<long> ptActivationType;
      } VDP_PublicTransportContainerData_t;

      typedef struct VDP_SpecialTransportContainerData {
        uint8_t specialTransportType;
        uint8_t lightBarSirenInUse;
      } VDP_SpecialTransportContainerData_t;

      typedef struct VDP_RoadWorksContainerBasicData {
        uint8_t lightBarSirenInUse;
        VDPDataItem<long> roadWorksSubCauseCode;
        VDPDataItem<long> innerhardShoulderStatus;
        VDPDataItem<long> outerhardShoulderStatus;
        VDPDataItem<uint16_t> drivingLaneStatus; // Up to 14 lanes require at least 14 bits (as this value corresponds to an ASN.1 BIT FIELD)
      } VDP_RoadWorksContainerBasicData_t;


      typedef struct VDP_EmergencyContainerData {
        uint8_t lightBarSirenInUse;
        VDPDataItem<long> causeCode;
        VDPDataItem<long> subCauseCode;
        VDPDataItem<uint8_t> emergencyPriority; // Bit field
      } VDP_EmergencyContainerData_t;

      typedef struct VDP_SafetyCarContainerData {
        uint8_t lightBarSirenInUse;
        VDPDataItem<long> incidentIndicationCauseCode;
        VDPDataItem<long> incidentIndicationSubCauseCode;
        VDPDataItem<long> trafficRule; // optional ASN.1 enum
        VDPDataItem<long> speedLimit; // optional speed limit (in km/h - the ASN.1 unit is already km/h)
      } VDP_SafetyCarContainerData_t;

      virtual CAM_mandatory_data_t getCAMMandatoryData() = 0;
      virtual CPM_mandatory_data_t getCPMMandatoryData() = 0;
      virtual MCM_mandatory_data_t getMCMMandatoryData() = 0;

      // These methods are used by the CAM generation frequency management mechanism,
      // as mandated by ETSI, and they should return values which are not already
      // converted to the ETSI measurements units for CAM messages
      // For instance, the speed is encoded into a CAM in terms of [0.1 m/s], and it
      // is correct that getCAMMandatoryData() should return a speed already converted to
      // [0.1 m/s].
      // However, as these functions are used in the aforementioned mechanism inside the
      // CA Basic Service implementation, they need to return normal units, i.e.
      // [m/s] for speed, [m] for the travelled distance and [deg] for heading
      // That's why they return non-ASN1 types but normal 'double' values
      virtual double getSpeedValue() = 0;
      virtual double getAccelerationValue() = 0;
      virtual double getTravelledDistance() = 0;
      virtual double getHeadingValue() = 0;
      virtual double getVehicleLength() = 0;

      // These methods shall return a position either cartesian or geodetic (needed mainly by the GeoNetworking module)
      // getPositionLatLon() shall return the current position of the object as (Latitude,Longitude,Altitude)
      // getPositionXY() shall return the current position as cartesian projected coordinates (x,y,z)
      // If the Altitude (or z) is not available, it must be set to DBL_MAX
      // getXY() shall be a utility function converting from Lat,Lon to x,y on a projected coordinate system (e.g. using Transverse Mercator)
      virtual VDP_position_latlon_t getPosition() = 0;
      virtual VDP_position_cartesian_t getPositionXY() = 0;
      virtual VDP_position_cartesian_t getXY(double lon, double lat) = 0;
      virtual double getCartesianDist (double lon1, double lat1, double lon2, double lat2) = 0;


      // These methods refer to optional fields in mandatory containers
      // If the information is not provided, they should be implemented
      // such that the returned VDPDataItem has m_available = false

      VDPDataItem<uint8_t> getAccelerationControl() {return VDPDataItem<uint8_t>(false);}
      virtual VDPDataItem<int> getLanePosition() = 0;

//      virtual VDPDataItem<VDPValueConfidence<int,int>> getSteeringWheelAngle() = 0;
//      virtual VDPDataItem<VDPValueConfidence<int,int>> getLateralAcceleration() = 0;
//      virtual VDPDataItem<VDPValueConfidence<int,int>> getVerticalAcceleration() = 0;
//      virtual VDPDataItem<int> getPerformanceClass() = 0;
//      virtual VDPDataItem<VDP_CEN_DSRC_tolling_zone_t> getCenDsrcTollingZone() = 0;

      VDPDataItem<VDPValueConfidence<int,int>> getSteeringWheelAngle() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
      VDPDataItem<VDPValueConfidence<int,int>> getLateralAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
      VDPDataItem<VDPValueConfidence<int,int>> getVerticalAcceleration() {return VDPDataItem<VDPValueConfidence<int,int>>(false);}
      VDPDataItem<int> getPerformanceClass() {return VDPDataItem<int>(false);}
      VDPDataItem<VDP_CEN_DSRC_tolling_zone_t> getCenDsrcTollingZone() {return VDPDataItem<VDP_CEN_DSRC_tolling_zone_t>(false);}


      // Low frequency container data (PH should be computed by the CA Basic Service and not provided here)
      virtual VDPDataItem<unsigned int> getVehicleRole() {return m_vehicleRole;}
      virtual VDPDataItem<uint8_t> getExteriorLights() = 0;

      // Special vehicle container
      virtual VDPDataItem<VDP_PublicTransportContainerData_t> getPublicTransportContainerData() {return m_publicTransportContainerData;}
      virtual VDPDataItem<VDP_SpecialTransportContainerData_t> getSpecialTransportContainerData() {return m_specialTransportContainerData;}
      virtual VDPDataItem<int> getDangerousGoodsBasicType() {return m_dangerousGoodsBasicType;} // For the DangerousGoodsContainer
      virtual VDPDataItem<VDP_RoadWorksContainerBasicData_t> getRoadWorksContainerBasicData_t() {return m_roadWorksContainerBasicData;}
      virtual VDPDataItem<uint8_t> getRescueContainerLightBarSirenInUse() {return m_rescueContainerLightBarSirenInUse;}
      virtual VDPDataItem<VDP_EmergencyContainerData_t> getEmergencyContainerData() {return m_emergencyContainerData;}
      virtual VDPDataItem<VDP_SafetyCarContainerData_t> getSafetyCarContainerData() {return m_safetyCarContainerData;}


      virtual void setVehicleRole(unsigned int data){m_vehicleRole = VDPDataItem<unsigned int>(data);}

      // Special vehicle container setters
      virtual void setPublicTransportContainerData(VDP_PublicTransportContainerData_t data) {m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t>(data);}
      virtual void setSpecialTransportContainerData(VDP_SpecialTransportContainerData_t data) {m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t>(data);}
      virtual void setDangerousGoodsBasicType (int data) {m_dangerousGoodsBasicType = VDPDataItem<int>(data);} // For the DangerousGoodsContainer
      virtual void setRoadWorksContainerBasicData(VDP_RoadWorksContainerBasicData_t data) {m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t>(data);}
      virtual void setRescueContainerLightBarSirenInUse(uint8_t data) {m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t>(data);}
      virtual void setEmergencyContainerData(VDP_EmergencyContainerData_t data) {m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t>(data);}
      virtual void setSafetyCarContainerData(VDP_SafetyCarContainerData_t data) {m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t>(data);}


      void setFixedVehicleLength(VDPValueConfidence<long,long> vehicle_length)
      {
         m_vehicle_length=vehicle_length;
      }

      void setFixedVehicleWidth(long vehicle_width)
      {
         m_vehicle_width=vehicle_width;
      }

      protected:


        VDPValueConfidence<long,long> m_vehicle_length;
        long m_vehicle_width;

        VDPDataItem<unsigned int> m_vehicleRole;

        // Special vehicle container
        VDPDataItem<VDP_PublicTransportContainerData_t> m_publicTransportContainerData;
        VDPDataItem<VDP_SpecialTransportContainerData_t> m_specialTransportContainerData;
        VDPDataItem<int> m_dangerousGoodsBasicType; // For the DangerousGoodsContainer
        VDPDataItem<VDP_RoadWorksContainerBasicData_t> m_roadWorksContainerBasicData;
        VDPDataItem<uint8_t> m_rescueContainerLightBarSirenInUse;
        VDPDataItem<VDP_EmergencyContainerData_t> m_emergencyContainerData;
        VDPDataItem<VDP_SafetyCarContainerData_t> m_safetyCarContainerData;

  };
}
#endif // VDP_H
