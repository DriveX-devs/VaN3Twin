/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "vdpTraci.h"
#include "ns3/vdp.h"

extern "C" {
  #include "ns3/CAM.h"
}

namespace ns3
{
  VDPTraCI::VDPTraCI()
  {
    m_traci_client=NULL;
    m_id="(null)";
    m_isStatic = NULL;

    m_vehicleRole = VDPDataItem<unsigned int>(false);
    // Special vehicle container
    m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t>(false);
    m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t>(false);
    m_dangerousGoodsBasicType = VDPDataItem<int>(false); // For the DangerousGoodsContainer
    m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t>(false);
    m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t>(false);
    m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t>(false);
    m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t>(false);

  }

  VDPTraCI:: VDPTraCI(Ptr<TraciClient> traci_client, std::string node_id)
  {
    m_traci_client=traci_client;

    m_isStatic = false;

    m_id = node_id;

    if (!m_isStatic)
      {
        /* Length and width of car [0.1 m] */
        m_vehicle_length = VDPValueConfidence<long, long> (
            m_traci_client->TraCIAPI::vehicle.getLength (m_id) * DECI,
            VehicleLengthConfidenceIndication_unavailable);
        //    m_vehicle_length.vehicleLengthValue = m_traci_client->TraCIAPI::vehicle.getLength (m_id)*DECI;
        //    m_vehicle_length.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

        // ETSI TS 102 894-2 V1.2.1 - A.92 (Length greater than 102,2 m should be set to 102,2 m)
        if (m_vehicle_length.getValue () > 1022)
          {
            m_vehicle_length.setValue (1022);
          }

        m_vehicle_width = m_traci_client->TraCIAPI::vehicle.getWidth (m_id) * DECI;

        // ETSI TS 102 894-2 V1.2.1 - A.95 (Width greater than 6,1 m should be set to 6,1 m)
        if (m_vehicle_width > 61)
          {
            m_vehicle_width = 61;
          }
      }

    m_vehicleRole = VDPDataItem<unsigned int>(false);
    // Special vehicle container
    m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t>(false);
    m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t>(false);
    m_dangerousGoodsBasicType = VDPDataItem<int>(false); // For the DangerousGoodsContainer
    m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t>(false);
    m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t>(false);
    m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t>(false);
    m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t>(false);
  }

  VDPTraCI::VDPTraCI(Ptr<TraciClient> traci_client, std::string node_id, bool isStatic)
  {
    m_traci_client=traci_client;

    m_isStatic = isStatic;

    m_id = node_id;

    if(!m_isStatic)
      {
        /* Length and width of car [0.1 m] */
        m_vehicle_length = VDPValueConfidence<long, long> (
            m_traci_client->TraCIAPI::vehicle.getLength (m_id) * DECI,
            VehicleLengthConfidenceIndication_unavailable);
        //    m_vehicle_length.vehicleLengthValue = m_traci_client->TraCIAPI::vehicle.getLength (m_id)*DECI;
        //    m_vehicle_length.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

        // ETSI TS 102 894-2 V1.2.1 - A.92 (Length greater than 102,2 m should be set to 102,2 m)
        if (m_vehicle_length.getValue () > 1022)
          {
            m_vehicle_length.setValue (1022);
          }

        m_vehicle_width = m_traci_client->TraCIAPI::vehicle.getWidth (m_id) * DECI;

        // ETSI TS 102 894-2 V1.2.1 - A.95 (Width greater than 6,1 m should be set to 6,1 m)
        if (m_vehicle_width > 61)
          {
            m_vehicle_width = 61;
          }

        m_vehicleRole = VDPDataItem<unsigned int> (false);
        // Special vehicle container
        m_publicTransportContainerData = VDPDataItem<VDP_PublicTransportContainerData_t> (false);
        m_specialTransportContainerData = VDPDataItem<VDP_SpecialTransportContainerData_t> (false);
        m_dangerousGoodsBasicType = VDPDataItem<int> (false); // For the DangerousGoodsContainer
        m_roadWorksContainerBasicData = VDPDataItem<VDP_RoadWorksContainerBasicData_t> (false);
        m_rescueContainerLightBarSirenInUse = VDPDataItem<uint8_t> (false);
        m_emergencyContainerData = VDPDataItem<VDP_EmergencyContainerData_t> (false);
        m_safetyCarContainerData = VDPDataItem<VDP_SafetyCarContainerData_t> (false);
      }
  }

  VDP::VDP_position_latlon_t
  VDPTraCI::getPosition()
  {
    VDP_position_latlon_t vdppos;

    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);

    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    vdppos.lat=pos.y;
    vdppos.lon=pos.x;
    vdppos.alt=DBL_MAX;

    return vdppos;
  }

  VDP::VDP_position_cartesian_t
  VDPTraCI::getPositionXY()
  {
    VDP_position_cartesian_t vdppos;

    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos=m_traci_client->TraCIAPI::poi.getPosition(m_id);

    vdppos.x=pos.x;
    vdppos.y=pos.y;
    vdppos.z=pos.z;

    return vdppos;
  }

  VDP::VDP_position_cartesian_t
  VDPTraCI::getXY(double lon, double lat)
  {
    VDP_position_cartesian_t vdppos;

    libsumo::TraCIPosition pos;
    pos=m_traci_client->TraCIAPI::simulation.convertLonLattoXY (lon,lat);

    vdppos.x=pos.x;
    vdppos.y=pos.y;
    vdppos.z=pos.z;

    return vdppos;
  }

  double
  VDPTraCI::getCartesianDist (double lon1, double lat1, double lon2, double lat2)
  {
    libsumo::TraCIPosition pos1,pos2;
    pos1 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon1,lat1);
    pos2 = m_traci_client->TraCIAPI::simulation.convertLonLattoXY(lon2,lat2);
    return sqrt((pow((pos1.x-pos2.x),2)+pow((pos1.y-pos2.y),2)));
  }

  VDPTraCI::MCM_mandatory_data_t
  VDPTraCI::getMCMMandatoryData ()
  {
    MCM_mandatory_data_t MCMdata;

    /* Speed [0.01 m/s] */
    if (!m_isStatic)
      MCMdata.speed = VDPValueConfidence<> (m_traci_client->TraCIAPI::vehicle.getSpeed (m_id) * CENTI,
                                            SpeedConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    MCMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    MCMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    MCMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                             AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    MCMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    MCMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    MCMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Longitudinal acceleration [0.1 m/s^2] */
    if (!m_isStatic)
      MCMdata.longAcceleration = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAcceleration (m_id) * DECI,
                                                       AccelerationConfidence_unavailable);

    /* Heading WGS84 north [0.1 degree] */
    if (!m_isStatic)
      MCMdata.heading = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAngle (m_id) * DECI,
                                              HeadingConfidence_unavailable);

    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    MCMdata.driveDirection = DriveDirection_unavailable;

    /* Curvature and CurvatureCalculationMode */
    MCMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                              CurvatureConfidence_unavailable);
    MCMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

    /* Length and Width [0.1 m] */
    if (!m_isStatic) {
        MCMdata.VehicleLength = m_vehicle_length;
        MCMdata.VehicleWidth = m_vehicle_width;
      }

    /* Yaw Rate */
    MCMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,
                                            YawRateConfidence_unavailable);

    return MCMdata;
  }

  VDPTraCI::CAM_mandatory_data_t
  VDPTraCI::getCAMMandatoryData ()
  {
    CAM_mandatory_data_t CAMdata;

    /* Speed [0.01 m/s] */
    if (!m_isStatic)
      CAMdata.speed = VDPValueConfidence<> (m_traci_client->TraCIAPI::vehicle.getSpeed (m_id) * CENTI,
                                            SpeedConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos;
    if (!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    CAMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    CAMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                          AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    CAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    CAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    CAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Longitudinal acceleration [0.1 m/s^2] */
    if (!m_isStatic)
      CAMdata.longAcceleration = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAcceleration (m_id) * DECI,
                                                  AccelerationConfidence_unavailable);

    /* Heading WGS84 north [0.1 degree] */
    if (!m_isStatic)
      CAMdata.heading = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAngle (m_id) * DECI,
                                         HeadingConfidence_unavailable);

    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    CAMdata.driveDirection = DriveDirection_unavailable;

    /* Curvature and CurvatureCalculationMode */
    CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                             CurvatureConfidence_unavailable);
    CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

    /* Length and Width [0.1 m] */
    if (!m_isStatic) {
        CAMdata.VehicleLength = m_vehicle_length;
        CAMdata.VehicleWidth = m_vehicle_width;
      }

    /* Yaw Rate */
    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,
                                           YawRateConfidence_unavailable);

    int lanes = m_traci_client->TraCIAPI::edge.getLaneNumber (m_traci_client->TraCIAPI::vehicle.getRoadID (m_id));
    int current_lane = m_traci_client->TraCIAPI::vehicle.getLaneIndex (m_id);
    // ETSI enumeration policy is the opposite of SUMO's one
    // For SUMO: lanes counting starts from 0 from the right most lane
    // For ETSI: lanes counting starts from 1 from the left most lane
    current_lane = lanes - current_lane;
    CAMdata.lane = current_lane;

    return CAMdata;
  }

  VDPTraCI::CPM_mandatory_data_t
  VDPTraCI::getCPMMandatoryData ()
  {
    CPM_mandatory_data_t CPMdata;

    /* Speed [0.01 m/s] */
    if (!m_isStatic)
      CPMdata.speed = VDPValueConfidence<> (m_traci_client->TraCIAPI::vehicle.getSpeed (m_id) * CENTI,
                                            SpeedConfidence_unavailable);

    /* Position */
    libsumo::TraCIPosition pos;
    if(!m_isStatic)
      pos=m_traci_client->TraCIAPI::vehicle.getPosition(m_id);
    else
      pos = m_traci_client->TraCIAPI::poi.getPosition(m_id);
    pos=m_traci_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    // longitude WGS84 [0,1 microdegree]
    CPMdata.longitude=(Longitude_t)(pos.x*DOT_ONE_MICRO);
    // latitude WGS84 [0,1 microdegree]
    CPMdata.latitude=(Latitude_t)(pos.y*DOT_ONE_MICRO);

    /* Altitude [0,01 m] */
    CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                          AltitudeConfidence_unavailable);

    /* Position Confidence Ellipse */
    CPMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
    CPMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
    CPMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

    /* Longitudinal acceleration [0.1 m/s^2] */
    if(!m_isStatic)
      CPMdata.longAcceleration = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAcceleration (m_id) * DECI,
                                                  AccelerationConfidence_unavailable);

    /* Heading WGS84 north [0.1 degree] */
    if(!m_isStatic)
      CPMdata.heading = VDPValueConfidence<>(m_traci_client->TraCIAPI::vehicle.getAngle (m_id) * DECI,
                                         HeadingConfidence_unavailable);

    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    CPMdata.driveDirection = DriveDirection_unavailable;

    /* Curvature and CurvatureCalculationMode */
    CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                             CurvatureConfidence_unavailable);
    CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

    /* Length and Width [0.1 m] */
    if(!m_isStatic){
        CPMdata.VehicleLength = m_vehicle_length;
        CPMdata.VehicleWidth = m_vehicle_width;
      }

    /* Yaw Rate */
    CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,
                                           YawRateConfidence_unavailable);

    return CPMdata;
  }

  VDPDataItem<int>
  VDPTraCI::getLanePosition()
  {
    if (m_isStatic)
      return VDPDataItem<int>((int)NULL);
    int lanePosition;

    int lanes = m_traci_client->TraCIAPI::edge.getLaneNumber (m_traci_client->TraCIAPI::vehicle.getRoadID (m_id));
    lanePosition = lanes - m_traci_client->TraCIAPI::vehicle.getLaneIndex (m_id);

    if (lanePosition < 0 || lanePosition > 14)
      {
        lanePosition = LanePosition_offTheRoad;
      }

    return VDPDataItem<int>(lanePosition);
  }

  VDPDataItem<uint8_t>
  VDPTraCI::getExteriorLights ()
  {
    if(m_isStatic)
      return VDPDataItem<uint8_t>(false);
    int extLights = m_traci_client->TraCIAPI::vehicle.getSignals (m_id);
    uint8_t retval = 0;
    if(extLights & VEH_SIGNAL_BLINKER_RIGHT)
      retval |= 1<< ExteriorLights_rightTurnSignalOn;
    if(extLights & VEH_SIGNAL_BLINKER_LEFT)
      retval |= 1<<ExteriorLights_leftTurnSignalOn;
    if(extLights & VEH_SIGNAL_FRONTLIGHT)
      retval |= 1<<ExteriorLights_lowBeamHeadlightsOn;
    if(extLights & VEH_SIGNAL_FOGLIGHT)
      retval |= 1<<ExteriorLights_fogLightOn;
    if(extLights & VEH_SIGNAL_HIGHBEAM)
      retval |= 1<<ExteriorLights_highBeamHeadlightsOn;
    if(extLights & VEH_SIGNAL_BACKDRIVE)
      retval |= 1<<ExteriorLights_reverseLightOn;

    return VDPDataItem<uint8_t> (retval);

  }
}
