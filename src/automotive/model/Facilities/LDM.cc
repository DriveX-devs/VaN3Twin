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


#define DEG_2_RAD(val) ((val)*M_PI/180.0)

#define VEHICLE_AREA 9
#define LOG_FREQ 100

namespace ns3 {

    // Function to compute the distance between two objects, given their Lon/Lat
    double compute_dist(double lat_a, double lon_a, double lat_b, double lon_b) {
        // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
        return 12742000.0 * asin(sqrt(sin(DEG_2_RAD(lat_b - lat_a) / 2) * sin(DEG_2_RAD(lat_b - lat_a) / 2) +
                                      cos(DEG_2_RAD(lat_a)) * cos(DEG_2_RAD(lat_b)) *
                                      sin(DEG_2_RAD(lon_b - lon_a) / 2) * sin(DEG_2_RAD(lon_b - lon_a) / 2)));
    }

    const point_type frontLeftPoint(0.0, 0.5);
    const point_type frontRightPoint(0.0, -0.5);
    const point_type backRightPoint(-1.0, -0.5);
    const point_type backLeftPoint(-1.0, 0.5);
    const point_type boxCenterPoint(-0.5, 0.0);

    //cluster matching parameters
    const double MIN_ID_OVERLAP_RATIO = 0.0; //any intersection (between id sets) is valid
    const double MAX_DISTANCE_THRESHOLD = 5; //within 5 meters
    const double MAX_HEADING_DIFF = 60; //within 60 degrees
    const double MIN_IoU = 0.1;

    uint64_t get_timestamp_us(void) {
        time_t seconds;
        uint64_t microseconds;
        struct timespec now;

        if (clock_gettime(CLOCK_REALTIME, &now) == -1) {
            perror("Cannot get the current microseconds UTC timestamp");
            return -1;
        }

        seconds = now.tv_sec;
        microseconds = round(now.tv_nsec / 1e3);

        // milliseconds, due to the rounding operation, shall not exceed 999999
        if (microseconds > 999999) {
            seconds++;
            microseconds = 0;
        }

        return seconds * 1000000 + microseconds;
    }

    LDM::LDM() {
        m_card = 0;
        m_count = 0;
        m_stationID = 0;
        m_polygons = false;
        m_client = NULL;

        m_LDM = std::unordered_map<uint64_t, returnedVehicleData_t>();

        m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS), &LDM::deleteOlderThan,
                                                      this);

        std::srand(Simulator::Now().GetNanoSeconds());
        double desync = ((double) std::rand() / RAND_MAX);
        m_event_writeContents = Simulator::Schedule(MilliSeconds(LOG_FREQ + (desync * 100)), &LDM::writeAllContents,
                                                    this);

    }

    LDM::~LDM() {
        Simulator::Cancel(m_event_deleteOlderThan);
        Simulator::Cancel(m_event_writeContents);
        clear();
    }

    LDM::LDM_error_t
    LDM::insert(vehicleData_t newVehicleData) {
        LDM_error_t retval;

        if (m_card == UINT64_MAX) {
            return LDM_MAP_FULL;
        }

        auto it = m_LDM.find(newVehicleData.stationID);

        if (it == m_LDM.end()) {
            newVehicleData.age_us = Simulator::Now().GetMicroSeconds();
            m_LDM[newVehicleData.stationID].vehData = newVehicleData;
            m_LDM[newVehicleData.stationID].phData = PHpoints();
            m_LDM[newVehicleData.stationID].phData.insert(newVehicleData, m_stationID);
            m_card++;
            retval = LDM_OK;
        } else {
            newVehicleData.age_us = it->second.vehData.age_us;
            it->second.vehData = newVehicleData;
            it->second.phData.insert(newVehicleData, m_stationID);
            retval = LDM_UPDATED;
        }
        return retval;
    }

    LDM::LDM_error_t
    LDM::remove(uint64_t stationID) {

        auto it = m_LDM.find(stationID);

        if (it == m_LDM.end()) {
            return LDM_ITEM_NOT_FOUND;
        } else {
            m_LDM.erase(it);
            m_card--;
        }
        return LDM_OK;
    }

    LDM::LDM_error_t
    LDM::lookup(uint64_t stationID, returnedVehicleData_t &retVehicleData) {

        auto it = m_LDM.find(stationID);

        if (it == m_LDM.end()) {
            return LDM_ITEM_NOT_FOUND;
        } else {
            retVehicleData = it->second;
        }
        return LDM_OK;
    }

    LDM::LDM_error_t
    LDM::updateCPMincluded(uint64_t stationID, uint64_t timestamp) {
        auto it = m_LDM.find(stationID);

        if (it == m_LDM.end()) {
//            auto jt = m_cluster_map.find(stationID);
//            if (jt != m_cluster_map.end()) {
//                jt->second.lastCPMincluded = timestamp;
//                std::cout << "Cluster " << stationID << " sent at " << timestamp << std::endl;
//            } else {
                return LDM_ITEM_NOT_FOUND;
//            }
        } else {
            it->second.vehData.lastCPMincluded = timestamp;
            it->second.phData.setCPMincluded();
        }
        return LDM_OK;
    }
    LDM::LDM_error_t
    LDM::updateClusterCPMincluded(uint64_t stationID, uint64_t timestamp) {
        auto it = m_cluster_map.find(stationID);
            if (it != m_cluster_map.end()) {
                it->second.lastCPMincluded = timestamp;
                std::cout << "Cluster " << stationID << " sent at " << timestamp << std::endl;
            } else {
                return LDM_ITEM_NOT_FOUND;
            }
            return LDM_OK;
    }

    LDM::LDM_error_t
    LDM::rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles) {
        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (haversineDist(lat, lon, it->second.vehData.lat, it->second.vehData.lon) <= range_m) {
                selectedVehicles.push_back(it->second);
            }
        }

        return LDM_OK;
    }

    bool
    LDM::getAllPOs(std::vector<returnedVehicleData_t> &selectedVehicles) {
        bool retval = false;
        selectedVehicles.clear();

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (it->second.vehData.detected) {
                selectedVehicles.push_back(it->second);
                retval = true;
            }
        }

        return retval;
    }

    bool
    LDM::getAllPOs_TypeA(std::vector<returnedVehicleData_t> &selectedVehicles) {
        bool retval = false;
        selectedVehicles.clear();

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (it->second.vehData.detected) {
                if (it->second.vehData.itsType == itsType_pedestrian || it->second.vehData.itsType == itsType_bicycle)
                    selectedVehicles.push_back(it->second);
                retval = true;
            }
        }

        return retval;
    }

    bool
    LDM::getAllIDs(std::set<int> &IDs) {
        bool retval = false;

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
            IDs.insert(it->first);
            retval = true;
        }

        return retval;

    }

    bool
    LDM::getAllCVs(std::vector<returnedVehicleData_t> &selectedVehicles) {
        bool retval = false;

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (!it->second.vehData.detected &&
                !(it->second.vehData.itsType == itsType_pedestrian || it->second.vehData.itsType == itsType_bicycle)) {
                selectedVehicles.push_back(it->second);
                retval = true;
            }
        }

        return retval;
    }

    bool
    LDM::getAllCVRUs(std::vector<returnedVehicleData_t> &selectedVehicles) {
        bool retval = false;

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (it->second.vehData.VAM &&
                (it->second.vehData.itsType == itsType_pedestrian || it->second.vehData.itsType == itsType_bicycle)) {
                selectedVehicles.push_back(it->second);
                retval = true;
            }
        }

        return retval;
    }

    bool
    LDM::updateClusterMap(std::map<size_t, ClusterInfo> &cluster_map,
                          bool fromPerception) {   //call this from -perception- or -cpm-
        const double HEADING_DEVIATION_THRESH = 30;
        const double MIN_PED_per_CLUSTER = 3;
        LDM_error_t retval;
        std::set<int> ClusterIDs;
        // if we are not detecting any cluster we delete the ones we perceived
        //debug
        std::cout << "---STORED CLUSTER MAP---" << std::endl;
        if (!m_cluster_map.empty()) {
            for (auto &cluster: m_cluster_map) {
                std::cout << " Cluster ID: " << cluster.first << std::endl;
                for (auto id: cluster.second.IDs) {
                    std::cout << " Object ID: " << id << std::endl;
                }
                if (!cluster.second.IDs.empty()){
                    cluster.second.cardinality = cluster.second.IDs.size();
                }
                std::cout << " Size: " << cluster.second.cardinality << std::endl;
                std::cout << " Perceived by: " << cluster.second.perceivedBy << std::endl;
                std::cout << " (x = " << cluster.second.center.x << ", y = " << cluster.second.center.y << ")"
                          << std::endl;
                std::cout << " Mean heading: " << cluster.second.heading << std::endl;
                std::cout << " Speed: " << cluster.second.speed_ms << " m/s" << std::endl;

            }
        }
        //---update existing clusters---
        for (auto it = m_cluster_map.begin(); it != m_cluster_map.end(); ++it) {
            //update cluster dynamics given timestamp, velocity and heading
            auto time_since_last_prediction = Simulator::Now().GetMicroSeconds() - it->second.last_predicted_timestamp;
            auto time_since_last_detection = Simulator::Now().GetMicroSeconds() - it->second.timestamp_us;
            double delta_t = static_cast<double> (time_since_last_prediction) / 1000000.0; //convert to seconds

            it->second.center.x += it->second.speed_ms * std::cos(it->second.heading*M_PI/180) * delta_t;
            it->second.center.y += it->second.speed_ms * std::sin(it->second.heading*M_PI/180) * delta_t;

            it->second.last_predicted_timestamp = Simulator::Now().GetMicroSeconds();
//            std::cout << "Updating position of existing clusters: " << std::endl;
//            std::cout << "  Time since last predicted: " << delta_t << " seconds" << std::endl;
//            std::cout << "  Time since last detected: " << time_since_last_detection/1000000.0 << " seconds" << std::endl;
//            std::cout << "  New center - (x = " << it->second.center.x << ", y = " << it->second.center.y << ")"
//                      << std::endl;
        }

        std::set<size_t> matched_new_clusters; //new cluster that found a match
        std::set<size_t> matched_stored_clusters; //stored cluster to be updated


        findMatchCluster(cluster_map,matched_new_clusters);
        // Handle unmatched clusters
        for (auto& new_candidate_pair : cluster_map){
            if(matched_new_clusters.find(new_candidate_pair.first) == matched_new_clusters.end()){
                //assign a new id if necessary
                auto newID = 1;
                while (m_cluster_map.find(newID) != m_cluster_map.end()){
                    newID++;
                }
                std::cout << "Initiating new cluster: " << newID << std::endl;
                ClusterInfo& new_cluster = new_candidate_pair.second;

                m_cluster_map[newID] = new_cluster;
                m_cluster_map[newID].last_predicted_timestamp = Simulator::Now().GetMicroSeconds();
                m_cluster_map[newID].timestamp_us = Simulator::Now().GetMicroSeconds();
//                m_cluster_map[new_candidate_pair.first].perceivedBy = std::stoi(m_id);
            }
        }
        //remove possible duplicate cluster
        std::set<size_t> matched_clusters;
        findMatchCluster(m_cluster_map,matched_clusters,true); //set true to find duplicates
        for (auto it = m_cluster_map.begin(); it != m_cluster_map.end();){
            if(matched_clusters.find(it->first) == matched_clusters.end()){
                ++it;
            } else {
                std::cout << "---CLUSTER DELETED FROM MAP--- ID: "<< it->first << std::endl;
                it = m_cluster_map.erase(it);
            }
        }

        //check pedestrians heading within cluster
        for (auto it = m_cluster_map.begin(); it != m_cluster_map.end();){
            if (it->second.IDs.size()>0) {
                std::vector<double> member_headings;
                auto &[key, cluster] = *it;
                std::cout << "Checking stored cluster for pedestrian heading: "<< std::endl;
                std::cout << "Cluster heading: "<< cluster.heading <<std::endl;

                std::vector<unsigned long> &point_ids = cluster.IDs;
                bool removed_ped = true;
                while(removed_ped) {
                    removed_ped = false;
                    int most_outlier_id = -1;
                    double highest_diff = 0;
                    for (auto id: point_ids) {
                        returnedVehicleData_t retVRUdata;
                        LDM_error_t retval;
                        retval = lookup(id, retVRUdata);
                        if (retval == LDM_ITEM_NOT_FOUND){
                            std::cout << "Pedestrian already removed from LDM"<< std::endl;
                            continue;
                        }
                        std::cout << " PED ID " << id << " - PED heading: "<< retVRUdata.vehData.heading <<std::endl;
                        double diff = std::fmod(it->second.heading - retVRUdata.vehData.heading, 360.0);
                        if (diff > 180.0) {
                            diff -= 360.0;
                        } else if (diff < -180.0) {
                            diff += 360.0;
                        }
                        diff = std::abs(diff);
                        if (diff > highest_diff) {
                            highest_diff = diff;
                            most_outlier_id = id;
                        }
                    }
                    if (highest_diff > HEADING_DEVIATION_THRESH) {
                        std::cout << "after matching - Outlier ID: " << most_outlier_id << std::endl;
                        //remove outlier from point_ids
                        auto new_end = std::remove(point_ids.begin(), point_ids.end(), most_outlier_id);
                        point_ids.erase(new_end, point_ids.end());
                        //clear member_headings
                        member_headings.clear();
                        for (auto id: point_ids) {
                            LDM::returnedVehicleData_t retVRUdata;
                            lookup(id, retVRUdata);
                            member_headings.push_back(retVRUdata.vehData.heading);
//                                std::cout << "ID: " << id << " Heading: " << retVRUdata.vehData.heading << std::endl;
                        }
                        it->second.heading = get_circular_mean_heading(member_headings);
                        std::cout << "New Cluster mean heading: " << it->second.heading << " deg" << std::endl;

                        removed_ped = true;
                        it->second.cardinality = it->second.IDs.size();
                    }
                }
                if (point_ids.size() < MIN_PED_per_CLUSTER) {
                        std::cout << "Removing cluster " << key << " due to low cardinality: " << point_ids.size()
                                  << std::endl;
                    it = m_cluster_map.erase(it);
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }



        std::cout << "---NEW STORED CLUSTER MAP---" << std::endl;
        if (!m_cluster_map.empty()) {
            for (const auto &cluster: m_cluster_map) {
                std::cout << " Cluster ID: " << cluster.first << std::endl;
                for (auto id: cluster.second.IDs) {
                    std::cout << " Object ID: " << id << std::endl;
                }
                std::cout << " Size: " << cluster.second.cardinality << std::endl;
                std::cout << " Perceived by: " << cluster.second.perceivedBy << std::endl;
                std::cout << " (x = " << cluster.second.center.x << ", y = " << cluster.second.center.y << ")"
                          << std::endl;
                std::cout << " Mean heading: " << cluster.second.heading << std::endl;
                std::cout << " Speed: " << cluster.second.speed_ms << " m/s" << std::endl;

            }
        }
        retval = LDM_UPDATED;
        return retval;

    }

    bool
    LDM::getClusterMap(std::map<size_t, ClusterInfo> &cluster_map) {
        bool retval = false;
        if (!m_cluster_map.empty()) {
            cluster_map = m_cluster_map;
            retval = true;
        }
        return retval;
    }

    LDM::LDM_error_t
    LDM::rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles) {
        returnedVehicleData_t retData;

        // Get the latitude and longitude of the speficied vehicle
        if (lookup(stationID, retData) != LDM_OK) {
            return LDM_ITEM_NOT_FOUND;
        }

        // Perform a rangeSelect() centered on that latitude and longitude values
        return rangeSelect(range_m, retData.vehData.lat, retData.vehData.lon, selectedVehicles);
    }

    void
    LDM::deleteOlderThan() {
        uint64_t now = Simulator::Now().GetMicroSeconds();
        double curr_dwell = 0.0;

        for (auto it = m_LDM.begin(); it != m_LDM.end();) {
            std::string id = std::to_string(it->second.vehData.stationID);
            if (m_polygons && m_client != NULL) {
                std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList();
                if (std::find(polygonList.begin(), polygonList.end(), id) != polygonList.end() &&
                    !it->second.vehData.detected) {
                    m_client->TraCIAPI::polygon.remove(id, 5);
                }
            }
            if (m_last_stored.find(it->second.vehData.stationID) != m_last_stored.end()){
                m_last_stored.erase(it->second.vehData.stationID);
            }
            if (((double) (now - it->second.vehData.timestamp_us)) / 1000.0 > DB_DELETE_OLDER_THAN_SECONDS * 1000) {
                if (it->second.vehData.detected) {
                    long age = it->second.vehData.age_us;
                    curr_dwell = now - age; //Dwelling time on database
                    m_dwell_count++;
                    m_avg_dwell += (curr_dwell - m_avg_dwell) / m_dwell_count;

                    if (m_client != NULL) {
                        std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList();
                        if (std::find(polygonList.begin(), polygonList.end(), id) != polygonList.end() &&
                            m_polygons)
                            m_client->TraCIAPI::polygon.remove(id, 5);
                    }
                }
                //store excluded peds for tracking to manage clusters
                if (it->second.vehData.itsType == itsType_pedestrian){
                    auto excluded = m_last_stored.find(it->second.vehData.stationID);
                    if (excluded == m_last_stored.end()) {
                        it->second.vehData.timestamp_us = Simulator::Now().GetMicroSeconds();
                        m_last_stored[it->second.vehData.stationID] = it->second.vehData;
                    } else {
                        excluded->second.timestamp_us = Simulator::Now().GetMicroSeconds() ;
                    }
                }
                std::cout << "Pedestrian deleted from LDM -  " << it->first << std::endl;
                it = m_LDM.erase(it);
                m_card--;
            }
            else {
                ++it;
            }
        }
        for (auto jt = m_cluster_map.begin(); jt != m_cluster_map.end(); ) {
            uint64_t now_ms = Simulator::Now().GetMicroSeconds() / 1000;
            bool cluster_removed = false;

            // Check cluster timeout
            if (now_ms - (jt->second.timestamp_us / 1000) > DB_DELETE_OLDER_THAN_SECONDS_CLUSTER * 1000) {
                std::cout << "Vehicle " << m_id << std::endl;
                std::cout << "Cluster deleted - ID: " << jt->first << std::endl;
                std::cout << "Now: " << now_ms << "ms - last detected: "
                          << jt->second.timestamp_us / 1000 << "ms" << std::endl;

                jt = m_cluster_map.erase(jt);
                cluster_removed = true;
                continue;
            }
            //check singular pedestrian only for perceiving CAV
            if (jt->second.perceivedBy != m_stationID){
                ++jt;
                continue;
            }

            // Rebuild cluster membership
            std::vector<int> survivors;
            std::vector<cv::Point2f> cluster_points;

            for (int pedId : jt->second.IDs) {
                LDM::returnedVehicleData_t retVRUdata;
                LDM_error_t retval = lookup(pedId, retVRUdata);

                if (retval != LDM_ITEM_NOT_FOUND) {
                    // Pedestrian is in LDM → keep, use live position
                    survivors.push_back(pedId);
                    cluster_points.push_back(cv::Point2f(retVRUdata.vehData.x, retVRUdata.vehData.y));

                    // Clear any grace state if present
                    m_grace_time_ids.erase(pedId);
                    m_last_stored.erase(pedId);

                } else {
                    // Ped not in LDM → check grace time
                    auto grace = m_grace_time_ids.find(pedId);
                    if (grace == m_grace_time_ids.end()) {
                        // Start grace timer
                        m_grace_time_ids[pedId] = now_ms;
                        auto excluded = m_last_stored.find(pedId);
                        if (excluded != m_last_stored.end()) {
                            cluster_points.push_back(cv::Point2f(excluded->second.x, excluded->second.y));
                        }
                        survivors.push_back(pedId);
                    } else {
                        // Grace already running → check expiry
                        if (now_ms - grace->second > DB_DELETE_OLDER_THAN_SECONDS_CLUSTER * 1000) {
                            std::cout << "Ped " << pedId << " removed from cluster " << jt->first << std::endl;
                            m_grace_time_ids.erase(pedId);
                            m_last_stored.erase(pedId);
                        } else {
                            auto excluded = m_last_stored.find(pedId);
                            if (excluded != m_last_stored.end()) {
                                cluster_points.push_back(cv::Point2f(excluded->second.x, excluded->second.y));
                            }
                            survivors.push_back(pedId);
                        }
                    }
                }
            }

            // Replace cluster membership with survivors
            jt->second.IDs.clear();
            jt->second.IDs.insert(jt->second.IDs.begin(), survivors.begin(), survivors.end());

            // Remove cluster if too small, otherwise update geometry
            if (jt->second.IDs.size() < 3) {
                std::cout << "Cluster removed due to low cardinality" << std::endl;
                jt = m_cluster_map.erase(jt);
                cluster_removed = true;
            } else {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(cluster_points, center, radius);
                jt->second.center = center;
                jt->second.radius = radius;
                ++jt;
            }
        }


//        for (auto jt = m_cluster_map.begin(); jt != m_cluster_map.end();) {
//            bool  cluster_removed = false;
//            bool pedRemoved = false;
//            std::vector<cv::Point2f> cluster_points{};
//            if (((double) (now - jt->second.timestamp_us)) / 1000.0 > DB_DELETE_OLDER_THAN_SECONDS_CLUSTER * 1000) {
//                std::cout << "Vehicle " << m_id << std::endl;
//                std::cout << "Cluster deleted - ID: " << jt->first << std::endl;
//                std::cout << "Now: " << now/1000 << "ms - last detected: " << jt->second.timestamp_us/1000 << "ms" << std::endl;
//                jt = m_cluster_map.erase(jt);
//                cluster_removed = true;
//            } else {
//                // remove old pedestrians if cluster is not old enough
//                // pedestrians that are not in the LDM anymore + 2second
//                int count_connected_peds = 0;
//                for (auto pedId_it = jt->second.IDs.begin(); pedId_it != jt->second.IDs.end(); ){
//                    LDM::returnedVehicleData_t retVRUdata;
//                    LDM_error_t retval = lookup(*pedId_it, retVRUdata);
//                    if (retVRUdata.vehData.VAM){++count_connected_peds;}
//                    if (retval == LDM_ITEM_NOT_FOUND){
//                        auto excluded_ped = m_last_stored.find(*pedId_it);
//                        auto graceMember = m_grace_time_ids.find(*pedId_it);
//                        if (graceMember == m_grace_time_ids.end()) {
//                            m_grace_time_ids.insert({*pedId_it, Simulator::Now().GetMicroSeconds() / 1000});
//                            if (excluded_ped != m_last_stored.end()) {
//                                cluster_points.push_back(cv::Point2f(excluded_ped->second.x, excluded_ped->second.y));
//                            }
//                            ++pedId_it;
//                        } else {
//                            if ((Simulator::Now().GetMicroSeconds() / 1000) - graceMember->second > DB_DELETE_OLDER_THAN_SECONDS_CLUSTER * 1000){
//                                std::cout << "Ped " << *pedId_it << " removed from cluster " << jt->first << std::endl;
//                                int removed_ped_id = *pedId_it;
//                                pedId_it = jt->second.IDs.erase(pedId_it);
//                                m_grace_time_ids.erase(removed_ped_id);
//                                pedRemoved = true;
//                                m_last_stored.erase(removed_ped_id);
//                            } else {
//                                if (excluded_ped != m_last_stored.end()) { // Ensure it was actually stored
//                                    cluster_points.push_back(cv::Point2f(excluded_ped->second.x, excluded_ped->second.y));
//                                }
//                                ++pedId_it;
//                            }
//                        }
//                    } else {
//                        cluster_points.push_back(cv::Point2f(retVRUdata.vehData.x, retVRUdata.vehData.y));
//                        //pedestrian IS in the LDM - check if it is in grace time --remove it--
//                        auto graceMember = m_grace_time_ids.find(*pedId_it);
//                        if (graceMember != m_grace_time_ids.end()){
//                            m_grace_time_ids.erase(graceMember);
//                        }
//                        auto excluded_ped = m_last_stored.find(*pedId_it); // Also remove from m_last_stored if it was there
//                        if (excluded_ped != m_last_stored.end()){
//                            m_last_stored.erase(excluded_ped);
//                        }
//                        ++pedId_it;
//                    }
//                }
//                if (pedRemoved && cluster_points.size()>=3){
//                    cv::Point2f center;
//                    float radius;
//                    cv::minEnclosingCircle(cluster_points, center, radius);
//                    jt->second.center = center;
//                    jt->second.radius = radius;
//                    std::cout << "Ped removed --- new center: x = " << center.x << ", y = " << center.y << std::endl;
//                }
//                //after removing old pedestrians, check for cluster size
//                if (pedRemoved && cluster_points.size()<3){
//                    std::cout << "Cluster removed due to low cardinality" <<std::endl;
//                    jt = m_cluster_map.erase(jt);
//                    cluster_removed = true;
//                }
//            }
//            if (!cluster_removed){
//                ++jt;
//            }
//        }
        m_count++;
        //writeAllContents();
        m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS), &LDM::deleteOlderThan,
                                                      this);
    }


    void
    LDM::deleteOlderThanAndExecute(double time_milliseconds, void (*oper_fcn)(uint64_t, void *),
                                   void *additional_args) {
        uint64_t now = get_timestamp_us();
        double curr_dwell = 0.0;

        for (auto it = m_LDM.cbegin(); it != m_LDM.cend();) {
            std::string id = std::to_string(it->second.vehData.stationID);
            if (m_polygons && m_client != NULL) {
                std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList();
                if (std::find(polygonList.begin(), polygonList.end(), id) != polygonList.end() &&
                    !it->second.vehData.detected) {
                    m_client->TraCIAPI::polygon.remove(id, 5);
                }
            }
            if (((double) (now - it->second.vehData.timestamp_us)) / 1000.0 > DB_DELETE_OLDER_THAN_SECONDS * 1000) {
                if (it->second.vehData.detected) {
                    long age = it->second.vehData.age_us;
                    curr_dwell = now - age; //Dwelling time on database
                    m_dwell_count++;
                    m_avg_dwell += (curr_dwell - m_avg_dwell) / m_dwell_count;

                    if (m_client != NULL) {
                        std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList();
                        if (std::find(polygonList.begin(), polygonList.end(), id) != polygonList.end() &&
                            m_polygons)
                            m_client->TraCIAPI::polygon.remove(id, 5);
                    }
                }
                oper_fcn(it->second.vehData.stationID, additional_args);
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
    LDM::writeAllContents() {
        if (m_client == NULL)
            return;
        if (m_station_type == StationType_pedestrian)
            return;

        libsumo::TraCIPosition egoPosXY = m_client->TraCIAPI::vehicle.getPosition(m_id);

        std::vector<uint64_t> POs, CVs;
        double conf = 0.0;
        double age = 0.0;
        double assoc = 0.0;
        double dist = 0.0;
        double maxDist = 0.0;
        returnedVehicleData_t vehdata = {0};

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

            if (it->second.vehData.detected)
                POs.push_back(it->second.vehData.stationID);
            else if (!it->second.vehData.detected)
                CVs.push_back(it->second.vehData.stationID);
        }

        for (auto it = POs.begin(); it != POs.end(); it++) {
            lookup(*it, vehdata);
            //OptionalDataItem<long> respPMID = vehdata.vehData.respPMID;
            std::vector<long> assocCVIDs = vehdata.vehData.associatedCVs.getData();

            conf += vehdata.vehData.confidence.getData();
            age += (Simulator::Now().GetMicroSeconds() - (double) vehdata.vehData.timestamp_us) / 1000;


            std::string sID = "veh" + std::to_string(*it);
            libsumo::TraCIPosition PosXY = m_client->TraCIAPI::vehicle.getPosition(sID);
            double distance = sqrt(pow((egoPosXY.x - PosXY.x), 2) + pow((egoPosXY.y - PosXY.y), 2));
            dist += distance;
            if (distance > maxDist)
                maxDist = distance;

            if (!assocCVIDs.empty()) {
                std::vector<uint64_t> assocPMs;
                for (auto it = assocCVIDs.begin(); it != assocCVIDs.end(); it++) {
                    if (std::find(assocPMs.begin(), assocPMs.end(), *it) == assocPMs.end())
                        assocPMs.push_back(*it);
                }
                assoc += assocPMs.size();
            }
        }

        if (!POs.empty()) {
            conf = conf / POs.size();
            age = age / POs.size();
            assoc = assoc / POs.size();
            dist = dist / POs.size();
        }

        m_csv_file << Simulator::Now().GetSeconds() << ","
                   << m_card << ","
                   << POs.size() << ","
                   << conf << ","
                   << age << ","
                   << m_avg_dwell / 1000 << ","
                   << assoc << ","
                   << CVs.size() << ","
                   << dist << ","
                   << maxDist << ","
                   << std::endl;
        m_event_writeContents = Simulator::Schedule(MilliSeconds(LOG_FREQ), &LDM::writeAllContents, this);
    }

    void
    LDM::cleanup() {
        Simulator::Cancel(m_event_writeContents);
    }

    void
    LDM::executeOnAllContents(void (*oper_fcn)(vehicleData_t, void *), void *additional_args) {

        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
            oper_fcn(it->second.vehData, additional_args);
        }
    }

    void
    LDM::updatePolygons() {
        if (m_client == NULL)
            return;
        for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
            if (m_polygons) {
                std::string veh = std::to_string(it->second.vehData.stationID);
                //if(it->second.vehData.detected)
                if (it->second.vehData.stationID != m_stationID)
                    drawPolygon(it->second.vehData);
            }
        }
        m_event_updatePolygons = Simulator::Schedule(MilliSeconds(50), &LDM::updatePolygons, this); // 20fps
    }

    void
    LDM::drawPolygon(vehicleData_t data) {
        if (m_client == NULL)
            return;
        using namespace boost::geometry::strategy::transform;
        libsumo::TraCIPosition SPos;
        double angle = 0.0;
        vehiclePoints_t Spoints;
        std::string id = std::to_string(data.stationID);

        //Get sensed points
        // Scale with vehicle size
        scale_transformer<double, 2, 2> scaleS((double) data.vehicleLength.getData() / 10,
                                               (double) data.vehicleWidth.getData() / 10);
        boost::geometry::transform(boxCenterPoint, Spoints.center, scaleS);
        boost::geometry::transform(frontLeftPoint, Spoints.front_left, scaleS);
        boost::geometry::transform(frontRightPoint, Spoints.front_right, scaleS);
        boost::geometry::transform(backLeftPoint, Spoints.back_left, scaleS);
        boost::geometry::transform(backRightPoint, Spoints.back_right, scaleS);


        // Rotate
        angle = -1.0 * (90 - data.heading);
        rotate_transformer<boost::geometry::degree, double, 2, 2> rotateS(angle);
        boost::geometry::transform(Spoints.center, Spoints.center, rotateS);
        boost::geometry::transform(Spoints.front_left, Spoints.front_left, rotateS);
        boost::geometry::transform(Spoints.front_right, Spoints.front_right, rotateS);
        boost::geometry::transform(Spoints.back_left, Spoints.back_left, rotateS);
        boost::geometry::transform(Spoints.back_right, Spoints.back_right, rotateS);

        //Translate to actual front bumper position
        SPos = m_client->TraCIAPI::simulation.convertLonLattoXY(data.lon, data.lat);
        translate_transformer<double, 2, 2> translateS(SPos.x, SPos.y);
        boost::geometry::transform(Spoints.center, Spoints.center, translateS);
        boost::geometry::transform(Spoints.front_left, Spoints.front_left, translateS);
        boost::geometry::transform(Spoints.front_right, Spoints.front_right, translateS);
        boost::geometry::transform(Spoints.back_left, Spoints.back_left, translateS);
        boost::geometry::transform(Spoints.back_right, Spoints.back_right, translateS);

        libsumo::TraCIPositionVector SUMOPolygon;
        libsumo::TraCIColor magenta;
        magenta.r = 255;
        magenta.g = 0;
        magenta.b = 255;
        magenta.a = 255;
        libsumo::TraCIColor magenta_cpm;
        magenta_cpm.r = 0;
        magenta_cpm.g = 214;
        magenta_cpm.b = 0;
        magenta_cpm.a = 225;
        libsumo::TraCIColor cian_connected;
        cian_connected.r = 189;
        cian_connected.g = 238;
        cian_connected.b = 245;
        cian_connected.a = 255;
        SUMOPolygon.push_back(boost2TraciPos(Spoints.front_left));
        SUMOPolygon.push_back(boost2TraciPos(Spoints.back_left));
        SUMOPolygon.push_back(boost2TraciPos(Spoints.back_right));
        SUMOPolygon.push_back(boost2TraciPos(Spoints.front_right));
        SUMOPolygon.push_back(boost2TraciPos(Spoints.front_left));


        std::vector<std::string> polygonList;
        polygonList = m_client->TraCIAPI::polygon.getIDList();
        if (std::find(polygonList.begin(), polygonList.end(), id) != polygonList.end()) {
            m_client->TraCIAPI::polygon.setShape(id, SUMOPolygon);
            if (data.detected) {
                long who = data.perceivedBy.getData();
                if (who == m_stationID)
                    m_client->TraCIAPI::polygon.setColor(id, magenta);
                else
                    m_client->TraCIAPI::polygon.setColor(id, magenta_cpm);
            }
        } else {
            if (data.detected) {
                long who = data.perceivedBy.getData();
                if (who == m_stationID)
                    m_client->TraCIAPI::polygon.add(id, SUMOPolygon, magenta, 1, "building.yes", 5);
                else
                    m_client->TraCIAPI::polygon.add(id, SUMOPolygon, magenta_cpm, 1, "building.yes", 5);
            }
        }
    }

    libsumo::TraCIPosition
    LDM::boost2TraciPos(point_type point_type) {
        libsumo::TraCIPosition retPos;
        retPos.x = boost::geometry::get<0>(point_type);
        retPos.y = boost::geometry::get<1>(point_type);
        retPos.z = 1.0;
        return retPos;
    }

    void
    LDM::findMatchCluster(std::map<size_t, ClusterInfo> &cluster_map, std::set<size_t> &matched_new_clusters,bool findDuplicates){
        for (auto &new_candidate_pair: cluster_map) {
            size_t new_candidate_id = new_candidate_pair.first;
            ClusterInfo &new_candidate_cluster = new_candidate_pair.second;

            size_t best_matched_stored_id = 0;
            bool found_match = false;
            for (auto &stored_cluster_pair: m_cluster_map) {
                size_t stored_cluster_id = stored_cluster_pair.first;
                ClusterInfo &stored_cluster = stored_cluster_pair.second;
                //avoid comparing one cluster with itself when checking for duplicates and double check (1vs2 and 2vs1)
                if (findDuplicates && new_candidate_id >= stored_cluster_id){
                    std::cout << "id1 >= id2" << std::endl;
                    continue;
                }
                //same cluster stored twice
                if (findDuplicates && new_candidate_cluster.center == stored_cluster.center){
                    matched_new_clusters.insert(stored_cluster_id);
                    std::cout << "duplicate found - delete one cluster" << std::endl;
                    continue;
                }
                //---matching---
                //1. ID overlap (if both clusters are from perception)
                std::set<int> new_ids_set(new_candidate_cluster.IDs.begin(), new_candidate_cluster.IDs.end());
                std::set<int> stored_ids_set(stored_cluster.IDs.begin(), stored_cluster.IDs.end());

                std::vector<int> intersection_ids;
                std::set_intersection(new_ids_set.begin(), new_ids_set.end(), stored_ids_set.begin(),
                                      stored_ids_set.end(), std::back_inserter(intersection_ids));
                std::vector<int> union_ids;
                std::set_union(new_ids_set.begin(), new_ids_set.end(), stored_ids_set.begin(), stored_ids_set.end(),
                               std::back_inserter(union_ids));
                double id_overlap_ratio = static_cast<double>(intersection_ids.size()) / union_ids.size();

                //2. Proximity
                //Euclidean distance
                double distance = std::hypot(new_candidate_cluster.center.x - stored_cluster.center.x,
                                             new_candidate_cluster.center.y - stored_cluster.center.y);

                //3. Circular Mean Heading similarity
                double heading_diff = std::abs(new_candidate_cluster.heading - stored_cluster.heading);
                if (heading_diff > 180) {
                    heading_diff = std::abs(heading_diff - 360);
                }

                //4. IoU
                double clusterIoU = calculateCircleIoU(new_candidate_cluster, stored_cluster);
                std::cout << "StoredID: " << stored_cluster_id << " - Overlap: " << id_overlap_ratio
                          << " --- Distance: "
                          << distance << " m ---- Heading diff: " << heading_diff << "degrees ---- IoU: " << clusterIoU
                          << std::endl;

                if (id_overlap_ratio > MIN_ID_OVERLAP_RATIO && heading_diff <= MAX_HEADING_DIFF) {
                    std::cout << "Matched by overlap" << std::endl;
                    found_match = true;
                    best_matched_stored_id = stored_cluster_id;
                    break;
                }
                if (clusterIoU >= MIN_IoU && heading_diff <= MAX_HEADING_DIFF) {
                    std::cout << "Matched by IoU" << std::endl;
                    found_match = true;
                    best_matched_stored_id = stored_cluster_id;
                    break;
                }
                //todo check if distance is not too restrictive to match
                if (distance <= MAX_DISTANCE_THRESHOLD &&
                    heading_diff <= MAX_HEADING_DIFF) {
                    found_match = true;
                    best_matched_stored_id = stored_cluster_id;
                    break;
                }
                //delete duplicate
                if (findDuplicates && id_overlap_ratio == 1.00){
                    matched_new_clusters.insert(stored_cluster_id);
                    std::cout << "duplicate found - delete one cluster" << std::endl;
                    continue;
                }
            }
            if (found_match) {
                //fuse both clusters
                std::cout << "Matched new cluster " << new_candidate_id << " to stored cluster "
                          << best_matched_stored_id << std::endl;
                ClusterInfo &stored_cluster = m_cluster_map[best_matched_stored_id];

                // 1. update cluster state - center position and radius - and heading
                cv::Point2f combined_center;
                double combined_radius;
                findMECForClusters(stored_cluster, new_candidate_cluster, combined_center, combined_radius);

                stored_cluster.center = combined_center;
                stored_cluster.radius = combined_radius;

                std::vector<double> member_headings;
                //we give more weight to new heading
                member_headings.push_back(new_candidate_cluster.heading); //1x
                member_headings.push_back(new_candidate_cluster.heading); //2x new
                member_headings.push_back(stored_cluster.heading); //1x old
                auto mean_heading = get_circular_mean_heading(member_headings);
                stored_cluster.heading = mean_heading;

                uint64_t estimate_cardinality = 0;
                // 2. Merge Member IDs (Union)
                std::set<int> new_members_set(new_candidate_cluster.IDs.begin(), new_candidate_cluster.IDs.end());
                std::set<int> current_members_set(stored_cluster.IDs.begin(), stored_cluster.IDs.end());

                auto old_cardinality = stored_cluster.cardinality;
                //if id sets are not empty -> from perception - merge ids
                //perception + perception -> union ids -cardinality from list
                //cpm + perception ->union ids -cardinality max
                //cpm + cpm -> cardinality from newest cpm if same perceivedby.
                if (!new_members_set.empty() && !current_members_set.empty()) {
                    std::vector<int> merged_ids_vec;
                    std::set_union(current_members_set.begin(), current_members_set.end(),
                                   new_members_set.begin(), new_members_set.end(),
                                   std::back_inserter(merged_ids_vec));

                    stored_cluster.IDs.assign(merged_ids_vec.begin(),
                                              merged_ids_vec.end()); // Update the stored cluster's IDs
                    estimate_cardinality = stored_cluster.IDs.size(); // Update cardinality
                }

                if ((stored_cluster.perceivedBy == new_candidate_cluster.perceivedBy) &&
                        new_members_set.empty() && current_members_set.empty()){
                    stored_cluster.cardinality = new_candidate_cluster.cardinality;
//
                } else {
                    stored_cluster.cardinality = std::max(new_candidate_cluster.cardinality, estimate_cardinality);
                }
                stored_cluster.timestamp_us = Simulator::Now().GetMicroSeconds();
                stored_cluster.last_predicted_timestamp = Simulator::Now().GetMicroSeconds();

                if (old_cardinality != stored_cluster.cardinality){
                    stored_cluster.lastCPMincluded = 0;
                    std::cout << "Cluster changed size -> set lastCPMincluded = 0" << std::endl;
                }

                //store ids of matched clusters
                //when finding duplicates we don't care about new clusters
                if (!findDuplicates){
                    matched_new_clusters.insert(new_candidate_id);
                }
//                matched_stored_clusters.insert(best_matched_stored_id);
            }else{
                std::cout << "NO MATCH FOUND" << std::endl;
            }

        }
    }

    void
    LDM::findMECForClusters(const ClusterInfo &cluster1, const ClusterInfo &cluster2,
                            cv::Point2f &new_center, double &new_radius) {

        cv::Point2f p1 = cluster1.center;
        double r1 = cluster1.radius;
        cv::Point2f p2 = cluster2.center;
        double r2 = cluster2.radius;

        double D = std::hypot(p1.x - p2.x, p1.y - p2.y);

        // Epsilon to account for floating point inaccuracies
        const double EPSILON = 1e-9;

        // Step 1: Check for containment
        if (D + r2 <= r1 + EPSILON) { // C1 encloses C2
            new_center = p1;
            new_radius = r1;
            std::cout << "Cluster 1 encloses Cluster 2. MEC is Cluster 1.\n";
            return;
        }
        if (D + r1 <= r2 + EPSILON) { // C2 encloses C1
            new_center = p2;
            new_radius = r2;
            std::cout << "Cluster 2 encloses Cluster 1. MEC is Cluster 2.\n";
            return;
        }

        // Step 2: No containment - calculate new MEC
        std::cout << "No containment. Calculating combined MEC.\n";
        new_radius = (D + r1 + r2) / 2.0;

        // Calculate new center
        if (D < EPSILON) { // Centers are practically the same
            new_center = p1; // Or p2, doesn't matter
        } else {
            // Unit vector from P1 to P2
            double ux = (p2.x - p1.x) / D;
            double uy = (p2.y - p1.y) / D;

            // Offset distance from P1 to the new center
            double offset_distance = new_radius - r1;

            new_center.x = p1.x + ux * offset_distance;
            new_center.y = p1.y + uy * offset_distance;
        }
    }

    double
    LDM::calculateCircleIoU(const ClusterInfo& new_cluster, const ClusterInfo& stored_cluster) {
        cv::Point2f center1 = new_cluster.center;
        double r1 = new_cluster.radius;
        cv::Point2f center2 = stored_cluster.center;
        double r2 = stored_cluster.radius;

        // 1. Calculate the distance between the centers
        double d = std::hypot(center2.x - center1.x, center2.y - center1.y);

        // 2. Calculate areas of individual circles
        double area1 = M_PI * r1 * r1;
        double area2 = M_PI * r2 * r2;

        double intersection_area = 0.0;

        // 3. Handle different overlap cases:

        // Case A: Circles do not overlap (distance >= sum of radii)
        if (d >= r1 + r2) {
            intersection_area = 0.0;
        }
            // Case B: One circle is completely inside the other (distance <= absolute difference of radii)
        else if (d <= std::abs(r1 - r2)) {
            // Intersection is the area of the smaller circle
            intersection_area = M_PI * std::min(r1, r2) * std::min(r1, r2);
        }
            // Case C: Circles partially overlap
        else {
            // This is the most complex case, using the formula for the intersection of two circle segments.
            // It involves the law of cosines and areas of circular segments.

            // Clamp arguments to acos to prevent NaN due to floating point inaccuracies
            // Values should theoretically be between -1 and 1 due to the 'else if' conditions
            double arg1 = (d * d + r1 * r1 - r2 * r2) / (2 * d * r1);
            double arg2 = (d * d + r2 * r2 - r1 * r1) / (2 * d * r2);

            // Clamp to [-1, 1] to handle potential floating-point precision issues
            arg1 = std::max(-1.0, std::min(1.0, arg1));
            arg2 = std::max(-1.0, std::min(1.0, arg2));

            double alpha = 2 * std::acos(arg1); // Angle for circle 1
            double beta = 2 * std::acos(arg2);  // Angle for circle 2

            // Area of circular segment = 0.5 * r^2 * (angle_in_radians - sin(angle_in_radians))
            double segment_area1 = 0.5 * r1 * r1 * (alpha - std::sin(alpha));
            double segment_area2 = 0.5 * r2 * r2 * (beta - std::sin(beta));

            intersection_area = segment_area1 + segment_area2;
        }

        // 4. Calculate Union Area
        // Union = Area1 + Area2 - IntersectionArea
        double union_area = area1 + area2 - intersection_area;

        // 5. Calculate IoU
        if (union_area <= std::numeric_limits<double>::epsilon()) { // Check for near-zero union area
            return 0.0; // Or 1.0 if both circles are effectively zero-area and identical
        }

        return intersection_area / union_area;
    }

    double
    LDM::get_circular_mean_heading(const std::vector<double> &headings_deg) {
        if (headings_deg.empty()) return 0.0;

        double sum_cos = 0.0;
        double sum_sin = 0.0;

        for (double heading_deg: headings_deg) {
            double heading_rad = heading_deg * M_PI / 180.0;
            sum_cos += std::cos(heading_rad);
            sum_sin += std::sin(heading_rad);
        }

        // Handle sum near zero vector
        if (std::abs(sum_cos) < 1e-9 && std::abs(sum_sin) < 1e-9) {
            // If all headings were identical, return that heading
            bool all_identical = true;
            if (headings_deg.size() > 1) {
                for (size_t i = 1; i < headings_deg.size(); ++i) {
                    double diff = std::fmod(headings_deg[i] - headings_deg[0], 360.0);
                    if (diff > 180.0) {
                        diff -= 360.0;
                    } else if (diff < -180.0) {
                        diff += 360.0;
                    }
                    if (std::abs(diff) > 1e-9) {
                        all_identical = false;
                        break;
                    }
                }
            }
            if (all_identical && !headings_deg.empty()) return headings_deg[0];

            return 0.0; // Fallback for sum near zero vector
        }


        double mean_heading_rad = std::atan2(sum_sin, sum_cos);

        double mean_heading_deg = mean_heading_rad * 180.0 / M_PI;
        if (mean_heading_deg < 0) {
            mean_heading_deg += 360.0;
        }

        return mean_heading_deg;
    }
}

