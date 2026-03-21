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
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/
#include "LDM.h"
#include <cmath>
#include <iostream>
// #include "ns3/geonet.h"

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

#define VEHICLE_AREA 9
#define LOG_FREQ 100

namespace ns3 {

  // Function to compute the distance between two objects, given their Lon/Lat
  double compute_dist(double lat_a, double lon_a, double lat_b, double lon_b)
  {
      // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
      return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  }
  const point_type frontLeftPoint(0.0, 0.5);
  const point_type frontRightPoint(0.0, -0.5);
  const point_type backRightPoint(-1.0, -0.5);
  const point_type backLeftPoint(-1.0, 0.5);
  const point_type boxCenterPoint(-0.5, 0.0);
  uint64_t get_timestamp_us(void)
  {
    time_t seconds;
    uint64_t microseconds;
    struct timespec now;

    if(clock_gettime(CLOCK_REALTIME, &now) == -1) {
            perror("Cannot get the current microseconds UTC timestamp");
            return -1;
    }

    seconds=now.tv_sec;
    microseconds=round(now.tv_nsec/1e3);

    // milliseconds, due to the rounding operation, shall not exceed 999999
    if(microseconds > 999999) {
            seconds++;
            microseconds=0;
    }

    return seconds*1000000+microseconds;
  }

  LDM::LDM()
  {
    m_card = 0;
    m_count = 0;
    m_stationID = 0;
    m_polygons=false;
    m_client=NULL;

    m_LDM = std::unordered_map<uint64_t,returnedVehicleData_t> ();

    m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS),&LDM::deleteOlderThan,this);
  }

  LDM::~LDM() {
      Simulator::Cancel(m_event_deleteOlderThan);
      Simulator::Cancel(m_event_writeContents);
      clear();
  }

  LDM::LDM_error_t
  LDM::insert(vehicleData_t newVehicleData)
  {
    LDM_error_t retval;

    if(m_card==UINT64_MAX) {
            return LDM_MAP_FULL;
    }

    auto it = m_LDM.find(newVehicleData.stationID);

    if (it == m_LDM.end()) {
        newVehicleData.age_us = Simulator::Now().GetMicroSeconds ();
        m_LDM[newVehicleData.stationID].vehData = newVehicleData;
        m_LDM[newVehicleData.stationID].phData = PHpoints();
        m_LDM[newVehicleData.stationID].phData.insert (newVehicleData,m_stationID);
        m_card++;
        retval = LDM_OK;
    } else {
        newVehicleData.age_us = it->second.vehData.age_us;
        it->second.vehData = newVehicleData;
        it->second.phData.insert (newVehicleData,m_stationID);
        retval = LDM_UPDATED;
    }
    return retval;
  }

  LDM::LDM_error_t
  LDM::remove(uint64_t stationID)
  {

    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        m_LDM.erase (it);
        m_card--;
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::lookup(uint64_t stationID,returnedVehicleData_t &retVehicleData)
  {

    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        retVehicleData = it->second;
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::updateCPMincluded(uint64_t stationID,uint64_t timestamp)
  {
    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        it->second.vehData.lastCPMincluded = timestamp;
        it->second.phData.setCPMincluded ();
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

        if(haversineDist(lat,lon,it->second.vehData.lat,it->second.vehData.lon)<=range_m) {
                selectedVehicles.push_back(it->second);
        }
    }

    return LDM_OK;
  }

  bool
  LDM::getAllPOs (std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

	if(it->second.vehData.detected) {
		selectedVehicles.push_back(it->second);
		retval = true;
	}
    }

    return retval;
  }

  bool
  LDM::getAllIDs (std::set<int> &IDs)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        IDs.insert (it->first);
        retval = true;
    }

    return retval;

  }

  bool
  LDM::getAllCVs(std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

	if(!it->second.vehData.detected) {
		selectedVehicles.push_back(it->second);
		retval = true;
	}
    }

    return retval;
  }

  LDM::LDM_error_t
  LDM::rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    returnedVehicleData_t retData;

    // Get the latitude and longitude of the speficied vehicle
    if(lookup(stationID,retData)!=LDM_OK) {
            return LDM_ITEM_NOT_FOUND;
    }

    // Perform a rangeSelect() centered on that latitude and longitude values
    return rangeSelect(range_m,retData.vehData.lat,retData.vehData.lon,selectedVehicles);
  }

  void
  LDM::deleteOlderThan()
  {
    uint64_t now = Simulator::Now ().GetMicroSeconds ();
    double curr_dwell = 0.0;

    for (auto it = m_LDM.cbegin(); it != m_LDM.cend();) {
        std::string id = std::to_string(it->second.vehData.stationID);
        if(m_polygons && m_client!=NULL)
          {
            std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
            if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
               !it->second.vehData.detected)
              {
                m_client->TraCIAPI::polygon.remove(id,5);
              }
          }
        if(((double)(now-it->second.vehData.timestamp_us))/1000.0 > DB_DELETE_OLDER_THAN_SECONDS*1000) {
            if(it->second.vehData.detected)
              {
                long age = it->second.vehData.age_us;
                curr_dwell = now - age; //Dwelling time on database
                m_dwell_count ++;
                m_avg_dwell += (curr_dwell-m_avg_dwell)/m_dwell_count;

                if(m_client!=NULL)
                  {
                    std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
                    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
                     m_polygons)
                      m_client->TraCIAPI::polygon.remove(id,5);
                  }
              }

              it = m_LDM.erase(it);
              m_card--;
            } else {
              ++it;
            }
    }
    m_count++;
    //writeAllContents();
    m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS),&LDM::deleteOlderThan,this);
  }


  void
  LDM::deleteOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args)
  {
    uint64_t now = get_timestamp_us();
    double curr_dwell = 0.0;

    for (auto it = m_LDM.cbegin(); it != m_LDM.cend();) {
        std::string id = std::to_string(it->second.vehData.stationID);
        if(m_polygons && m_client!=NULL)
          {
            std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
            if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
               !it->second.vehData.detected)
              {
                m_client->TraCIAPI::polygon.remove(id,5);
              }
          }
        if(((double)(now-it->second.vehData.timestamp_us))/1000.0 > DB_DELETE_OLDER_THAN_SECONDS*1000) {
            if(it->second.vehData.detected)
              {
                long age = it->second.vehData.age_us;
                curr_dwell = now - age; //Dwelling time on database
                m_dwell_count ++;
                m_avg_dwell += (curr_dwell-m_avg_dwell)/m_dwell_count;

                if(m_client!=NULL)
                  {
                    std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
                    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
                     m_polygons)
                      m_client->TraCIAPI::polygon.remove(id,5);
                  }
              }
              oper_fcn(it->second.vehData.stationID,additional_args);
              it = m_LDM.erase(it);
              m_card--;
            } else {
              ++it;
            }
    }
  }

  void
  LDM::clear() {

    m_LDM.clear();
    // Set the cardinality of the map to 0 again
    m_card = 0;
  }

  void
  LDM::writeAllContents()
  {
    if (m_client == NULL)
      return;
    if (m_station_type == StationType_pedestrian)
      return;

    libsumo::TraCIPosition egoPosXY=m_client->TraCIAPI::vehicle.getPosition(m_id);

    std::vector<uint64_t> POs,CVs;
    double conf = 0.0;
    double age = 0.0;
    double assoc = 0.0;
    double dist = 0.0;
    double maxDist = 0.0;
    returnedVehicleData_t vehdata = {0};

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

        if(it->second.vehData.detected)
          POs.push_back (it->second.vehData.stationID);
        else if(!it->second.vehData.detected)
          CVs.push_back (it->second.vehData.stationID);
    }

    for (auto it = POs.begin(); it != POs.end(); it++)
      {
        lookup(*it,vehdata);
        //OptionalDataItem<long> respPMID = vehdata.vehData.respPMID;
        std::vector<long> assocCVIDs = vehdata.vehData.associatedCVs.getData ();

        conf += vehdata.vehData.confidence.getData();
        age += (Simulator::Now ().GetMicroSeconds () - (double) vehdata.vehData.timestamp_us)/1000;


        std::string sID = "veh" + std::to_string(*it);
        libsumo::TraCIPosition PosXY=m_client->TraCIAPI::vehicle.getPosition(sID);
        double distance = sqrt(pow((egoPosXY.x-PosXY.x),2)+pow((egoPosXY.y-PosXY.y),2));
        dist += distance;
        if(distance > maxDist)
          maxDist = distance;

        if(!assocCVIDs.empty ())
          {
            std::vector<uint64_t> assocPMs;
            for(auto it = assocCVIDs.begin();it !=assocCVIDs.end ();it++)
              {
                if(std::find(assocPMs.begin (),assocPMs.end(),*it) == assocPMs.end())
                  assocPMs.push_back (*it);
              }
            assoc += assocPMs.size ();
          }
      }

    if(!POs.empty ())
      {
        conf = conf / POs.size();
        age = age / POs.size();
        assoc = assoc / POs.size();
        dist = dist / POs.size();
      }

    m_csv_file << Simulator::Now ().GetSeconds () << ","
               << m_card << ","
               << POs.size() << ","
               << conf << ","
               << age << ","
               << m_avg_dwell/1000 << ","
               << assoc << ","
               << CVs.size () << ","
               << dist << ","
               << maxDist << ","
               << std::endl;
    m_event_writeContents = Simulator::Schedule(MilliSeconds(LOG_FREQ),&LDM::writeAllContents,this);
  }

  void
  LDM::cleanup()
  {
    Simulator::Cancel(m_event_writeContents);
  }

  void
  LDM::executeOnAllContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args)
  {

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        oper_fcn(it->second.vehData,additional_args);
    }
  }

  void
  LDM::updatePolygons()
  {
    if(m_client == NULL)
      return;
    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        if (m_polygons)
        {
            std::string veh = std::to_string(it->second.vehData.stationID);
            //if(it->second.vehData.detected)
            if(it->second.vehData.stationID != m_stationID)
              drawPolygon(it->second.vehData);
        }
    }
    m_event_updatePolygons = Simulator::Schedule(MilliSeconds (50),&LDM::updatePolygons,this); // 20fps
  }

  void
  LDM::drawPolygon(vehicleData_t data)
  {
    if(m_client == NULL)
      return;
    using namespace boost::geometry::strategy::transform;
    libsumo::TraCIPosition SPos;
    double angle = 0.0;
    vehiclePoints_t Spoints;
    std::string id = std::to_string(data.stationID);

    //Get sensed points
    // Scale with vehicle size
    scale_transformer<double, 2, 2> scaleS((double) data.vehicleLength.getData ()/10,(double) data.vehicleWidth.getData()/10);
    boost::geometry::transform(boxCenterPoint, Spoints.center, scaleS);
    boost::geometry::transform(frontLeftPoint, Spoints.front_left,scaleS);
    boost::geometry::transform(frontRightPoint, Spoints.front_right,scaleS);
    boost::geometry::transform(backLeftPoint, Spoints.back_left, scaleS);
    boost::geometry::transform(backRightPoint, Spoints.back_right,scaleS);


    // Rotate
    angle = -1.0 * (90-data.heading);
    rotate_transformer<boost::geometry::degree, double, 2, 2> rotateS(angle);
    boost::geometry::transform(Spoints.center, Spoints.center, rotateS);
    boost::geometry::transform(Spoints.front_left, Spoints.front_left, rotateS);
    boost::geometry::transform(Spoints.front_right, Spoints.front_right, rotateS);
    boost::geometry::transform(Spoints.back_left, Spoints.back_left, rotateS);
    boost::geometry::transform(Spoints.back_right, Spoints.back_right, rotateS);

    //Translate to actual front bumper position
    SPos = m_client->TraCIAPI::simulation.convertLonLattoXY (data.lon,data.lat);
    translate_transformer<double, 2, 2> translateS(SPos.x,SPos.y);
    boost::geometry::transform(Spoints.center, Spoints.center, translateS);
    boost::geometry::transform(Spoints.front_left, Spoints.front_left, translateS);
    boost::geometry::transform(Spoints.front_right, Spoints.front_right, translateS);
    boost::geometry::transform(Spoints.back_left, Spoints.back_left, translateS);
    boost::geometry::transform(Spoints.back_right, Spoints.back_right, translateS);

    libsumo::TraCIPositionVector SUMOPolygon;
    libsumo::TraCIColor magenta;
    magenta.r=255;magenta.g=0;magenta.b=255;magenta.a=255;
    libsumo::TraCIColor magenta_cpm;
    magenta_cpm.r=0;magenta_cpm.g=214;magenta_cpm.b=0;magenta_cpm.a=225;
    libsumo::TraCIColor cian_connected;
    cian_connected.r=189;cian_connected.g=238;cian_connected.b=245;cian_connected.a=255;
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_left));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.back_left));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.back_right));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_right));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_left));


    std::vector<std::string> polygonList;
    polygonList = m_client->TraCIAPI::polygon.getIDList ();
    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end ())
      {
        m_client->TraCIAPI::polygon.setShape (id,SUMOPolygon);
        if (data.detected)
          {
            long who = data.perceivedBy.getData();
            if (who == m_stationID)
              m_client->TraCIAPI::polygon.setColor (id,magenta);
            else
              m_client->TraCIAPI::polygon.setColor (id,magenta_cpm);
          }
      }
    else
      {
        if(data.detected)
          {
            long who = data.perceivedBy.getData();
            if (who == m_stationID)
              m_client->TraCIAPI::polygon.add(id,SUMOPolygon,magenta,1,"building.yes",5);
            else
              m_client->TraCIAPI::polygon.add(id,SUMOPolygon,magenta_cpm,1,"building.yes",5);
          }
      }
  }

  libsumo::TraCIPosition
  LDM::boost2TraciPos(point_type point_type)
  {
    libsumo::TraCIPosition retPos;
    retPos.x = boost::geometry::get<0>(point_type);
    retPos.y = boost::geometry::get<1>(point_type);
    retPos.z = 1.0;
    return retPos;
  }
}
