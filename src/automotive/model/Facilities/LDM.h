#ifndef LDM_H
#define LDM_H

#include "ns3/ldm-utils.h"
#include "ns3/phPoints.h"
#include "ns3/core-module.h"
#include "ns3/traci-client.h"
#include "ns3/vdpTraci.h"
#include <unordered_map>
#include <vector>
#include <random>
#include <shared_mutex>
#include <boost/geometry.hpp>
//#include <boost/optional/optional.hpp>


#define DB_CLEANER_INTERVAL_SECONDS 0.5
#define DB_DELETE_OLDER_THAN_SECONDS 1
#define DB_DELETE_OLDER_THAN_SECONDS_CLUSTER 1
namespace ns3 {


  typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;

  using polygon_type = boost::geometry::model::polygon<point_type>;
  using linestring_type = boost::geometry::model::linestring<point_type>;

class LDM : public Object
{
public:
  typedef enum {
          LDM_OK,
          LDM_UPDATED,
          LDM_ITEM_NOT_FOUND,
          LDM_MAP_FULL,
          LDM_UNKNOWN_ERROR
  } LDM_error_t;

  typedef struct {
          vehicleData_t vehData;
          PHpoints phData;
  } returnedVehicleData_t;

    LDM();
    ~LDM();

    /**
     * @brief This function clears the whole database (to be used only when the dabatase and its content is not going to be accessed again)
     *
     * */
    void clear();
    /**
     * @brief  This function inserts or updates a vehicle in the database
     *
     * This function inserts or updates a vehicle in the database
     * The vehicle data should be passed inside a vehicleData_t structure and vehicles are univocally identified by their stationID
     * This function returns LDMMAP_OK if a new vehicle has been inserted, LDMMAP_UPDATED is an existing vehicle entry has been updated,
     * LDMMAP_MAP_FULL if the database if full and the insert operation failed (this should never happen, in any case)
     *
     * @param newVehicleData the vehicle data to be inserted or updated
     * @return LDM_error_t the result of the operation
     * */
    LDM_error_t insert(vehicleData_t newVehicleData);

    /**
     * @brief This function removes from the database the vehicle entry with station ID == stationID
     *
     *
     * This function removes from the database the vehicle entry with station ID == stationID
     * It returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal
     * It returns LDMMAP_OK if the vehicle entry was succesfully removed
     *
     * @param stationID the station ID of the vehicle to be removed
     * @return LDM_error_t the result of the operation
     * */
    LDM_error_t remove(uint64_t stationID);

    /**
     * @brief This function returns the vehicle entry with station ID == stationID - the entry data is returned in retVehicleData
     *
     * This function returns the vehicle entry with station ID == stationID - the entry data is returned in retVehicleData
     * This function returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal, while
     * it returns LDMMAP_OK if the retVehicleData structure was properly filled with the requested vehicle data
     *
     * @param stationID the station ID of the vehicle to be looked up
     * @param retVehicleData the structure in which the vehicle data will be returned
     * @return LDM_error_t the result of the operation
     * */
    LDM_error_t lookup(uint64_t stationID, returnedVehicleData_t &retVehicleData);

    /**
     * @brief This function returns a vector of vehicles, including their Path History points, located within a certain radius
     *
     * This function returns a vector of vehicles, including their Path History points, located within a certain radius
     * centered on a given latitude and longitude
     * For the time being, this function should always return LDMMAP_OK (i.e. to understand if no vehicles are returned,
     * you should check the size of the selectedVehicles vector)
     *
     * @param range_m the radius in meters around the vehicle
     * @param lat the latitude of the center of the search area
     * @param lon the longitude of the center of the search area
     * @param selectedVehicles the vector in which the selected vehicles will be returned
     * @return vector of vehicles, including their Path History points, located within the radius
     * */
    LDM_error_t rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles);

    /**
     * @brief This function returns a vector of vehicles, including their Path History points, located within a certain radius
     *
     * This function is the same as the other method with the same name, but it will return all the vehicles around another
     * vehicle (which is also included in the returned vector), given it stationID
     * This function may return LDMMAP_ITEM_NOT_FOUND if the specified stationID is not stored inside the database
     *
     * @param range_m the radius in meters around the vehicle
     * @param stationID the station ID of the vehicle around which the search is performed
     * @param selectedVehicles the vector in which the selected vehicles will be returned
     * @return vector of vehicles, including their Path History points, located within the radius
     * */
    LDM_error_t rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles);

    /**
     * @brief This function updates the timestamp indicating the last time the given object has been included in a CPM
     *
     * This function updates the timestamp indicating the last time the given object has been included in a CPM
     * The timestamp is updated to the current simulation time
     * This function returns LDMMAP_ITEM_NOT_FOUND if the specified stationID is not stored inside the database
     *
     * @param stationID the station ID of the vehicle for which the timestamp should be updated
     * @param timestamp the timestamp to be set
     * */
    LDM_error_t updateCPMincluded(uint64_t stationID,uint64_t timestamp);

    /**
     * @brief This function returns all Perceived Objects (POs) that are currently in the LDM, false if there are not POs in LDM
     *
     * @param selectedVehicles
     * @return
     */
    bool getAllPOs(std::vector<returnedVehicleData_t> &selectedVehicles);
    /**
     * @brief This function returns all Connected Vehicles (CVs) that are currently in the LDM, false if there are not CVs in LDM
     * @param selectedVehicles
     * @return
     */
    bool getAllCVs(std::vector<returnedVehicleData_t> &selectedVehicles);

    /**
     * @brief This function returns all the IDs currently stored in the LDM
     *
     * This function returns all the IDs currently stored in the LDM
     *
     * @param IDs the set in which the IDs will be returned
     * @return
     */
    bool getAllIDs(std::set<int> &IDs);

    /**
     * @brief This function deletes from the database all the entries older than time_milliseconds ms
     *
     * The entries are deleted if their age is > time_milliseconds ms if greater_equal == false,
     * or >= time_milliseconds ms if greater_equal == true
     * This function performs a full database read operation
     */
    void deleteOlderThan();

    /**
     * @brief This function is a combination of deleteOlderThan() and executeOnAllContents(), calling the open_fcn() callback for every deleted entry
     *
     * @param time_milliseconds the time in milliseconds, older than which the entries will be deleted
     * @param oper_fcn the callback function to be called for each deleted entry
     * @param additional_args the additional arguments to be passed to the callback
     */
    void deleteOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args);

    /**
     * @brief This function can be used to write all the content of the database in a log file
     *
     */
    void writeAllContents();

    void cleanup();

    /**
     * @brief This function reads the whole database, and, for each entry, it executes the "oper_fcn" callback
     *
     * This callback should return void (i.e. nothing) and have two arguments:
     * - a vehicleData_t structure, in which the data stored in each entry will be made available to the callback
     * - a void * pointer, by means of which possible additional arguments can be passed to the callback
     * The additional arguments to be passed to the callback, each time it is called, can be specified by setting
     * void *additional_args to a value different than "nullptr"
     * If additional_args == nullptr, also the second argument of each callback call will be nullptr
     *
     *
     * @param oper_fcn the callback function to be called for each entry
     * @param additional_args the additional arguments to be passed to the callback
     */
    void executeOnAllContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args);

    int getCardinality() {return m_card;};

    void setStationID(std::string id){m_id=id;m_stationID=std::stol(id.substr (3));}

    void setStationID(int id){m_stationID=id; m_id = std::to_string(id);}

    void enableOutputFile(std::string id){m_csv_file.open(id+"-LDM.csv",std::ofstream::trunc);
                                          m_csv_file << "Time,Size,POs,AvgConf,AvgAcc,AvgAge,AvgDwell,AvgAssoc,CVs,AvgDist,MaxDist,AvgT2D,UnderPPrange,AvgPPDwell"<<std::endl;}

    void setTraCIclient(Ptr<TraciClient> client){m_client=client;}
    void setVDP(VDP* vdp) {m_vdp=vdp;}
    void setStationType(StationType_t station_type){m_station_type=station_type;}

    /**
     * @brief This function updates all the Perceived Object polygon's showing the current perception of them
     *
     * The function is called every 100 ms
     */
    void updatePolygons();
    void drawPolygon(vehicleData_t data);
    void enablePolygons(){m_polygons=true;m_event_updatePolygons = Simulator::Schedule(MilliSeconds (100),&LDM::updatePolygons,this);}

    libsumo::TraCIPosition boost2TraciPos(point_type point_type);

private:

	// Main database structure
	std::unordered_map<uint64_t,returnedVehicleData_t> m_LDM;
	// Database cardinality (number of entries stored in the database)
	uint64_t m_card;
	long m_count;
	//TraCI client pointer
	Ptr<TraciClient> m_client; //!< TraCI client

        uint64_t m_stationID;
        std::string m_id;
        std::ofstream m_logfile_file;
        std::ofstream m_csv_file;
        VDP* m_vdp;

        bool m_polygons;

        EventId m_event_deleteOlderThan;
        EventId m_event_writeContents;
        EventId m_event_updatePolygons;

	double m_avg_dwell = 0.0;
	int m_dwell_count = 0;
        StationType_t m_station_type;
    std::map<size_t, ClusterInfo> m_cluster_map {};

    std::unordered_map<int, uint64_t> m_grace_time_ids {};
    bool getAllPOs_TypeA(std::vector<returnedVehicleData_t> &selectedVehicles);
    bool getAllCVRUs(std::vector<returnedVehicleData_t> &selectedVehicles);

    bool updateClusterMap(std::map<size_t, ClusterInfo>& cluster_map, bool fromPerception);
    bool getClusterMap(std::map<size_t, ClusterInfo> &cluster_map);
    void findMECForClusters(const ClusterInfo& cluster1, const ClusterInfo& cluster2,
                                 cv::Point2f& new_center, double& new_radius);
    double calculateCircleIoU(const ClusterInfo& new_cluster, const ClusterInfo& stored_cluster);
    void findMatchCluster(std::map<size_t, ClusterInfo> &cluster_map, std::set<size_t> &matched_new_clusters, bool findDuplicates=false);
    double get_circular_mean_heading(const std::vector<double> &headings_deg);
    LDM_error_t updateClusterCPMincluded(uint64_t stationID, uint64_t timestamp);

    std::unordered_map<uint64_t, vehicleData_t> m_last_stored;


};
}
#endif // LDM_H
