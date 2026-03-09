#ifndef BSCONTAINER_H
#define BSCONTAINER_H

#include "ns3/caBasicService.h"
#include "ns3/caBasicService_v1.h"
#include "ns3/denBasicService.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/VRUBasicService.h"
#include "ns3/cpBasicService.h"
#include "ns3/cpBasicService_v1.h"
#include "ns3/mcBasicService.h"
#include "ns3/sumo-sensor.h"

namespace ns3
{
  class BSContainer: public Object
  {
  public:
    BSContainer();
    ~BSContainer();
    BSContainer(unsigned long fixed_stationid,long fixed_stationtype,Ptr<TraciClient> mobility_client,bool real_time,Ptr<Socket> socket_rxtx);

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setSocket(Ptr<Socket> socket_rxtx);
    void setMobilityClient(Ptr<TraciClient> mobility_client) {m_mobility_client=mobility_client;}

    void linkMetricSupervisor(Ptr<MetricSupervisor> metSup) {m_metric_sup_ptr=metSup;}
    void enablePRRSupervisorForGNBeacons() {m_prrsup_beacons=true;}
    void disablePRRSupervisorForGNBeacons() {m_prrsup_beacons=false;}

    void addCAMRxCallback(std::function<void(asn1cpp::Seq<CAM>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback) {m_CAReceiveCallbackExtended=rx_callback;}
    void addDENMRxCallback(std::function<void(denData,Address,unsigned long,long,SignalInfo)> rx_callback) {m_DENReceiveCallbackExtended=rx_callback;}
    void addVAMRxCallback(std::function<void(asn1cpp::Seq<VAM>, Address, StationID_t, StationType_t)> rx_callback) {m_VAMReceiveCallbackExtended=rx_callback;}
    void addCPMRxCallback(std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback) {m_CPMReceiveCallbackExtended=rx_callback;}
    void addMCMRxCallback(std::function<void(asn1cpp::Seq<MCM>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback) {m_MCMReceiveCallbackExtended=rx_callback;}

    void setRealTime(bool real_time){m_real_time=real_time;}

    // ETSI Basic Services
    // Additional Basic Service getters should be added here
    Ptr<CABasicService> getCABasicService() {return &m_cabs;}
    Ptr<DENBasicService> getDENBasicService() {return &m_denbs;}
    Ptr<VRUBasicService> getVRUBasicService() {return &m_vrubs;}
    Ptr<CPBasicService> getCPBasicService() {return &m_cpbs;}
    Ptr<MCBasicService> getMCBasicService() {return &m_mcbs;}

    // Function to easily retrieve a pointer to the Mobility Client leveraged by this BSContainer
    // Currently, only SUMO/TraCI clients are supported
    Ptr<TraciClient> getTraCIclient() {return m_mobility_client;}

    StationID_t getStationID() {return m_station_id;}

    // This function can be used to change the prefix this module supposes for all vehicle IDs in SUMO
    // If a new Basic Service container is created for station ID = 7, for example, this module
    // assumes that it will be related to "veh7" in the XML file, if the prefix is "veh"
    // By default, the prefix is "veh"
    // If you want to change it to something else, you can leverage this function
    // For example, if vehicles in the XML file have id "carX", you should call "changeSUMO_ID_prefix("car")"
    void changeSUMO_ID_prefix(std::string new_prefix) {m_sumo_vehid_prefix=new_prefix;}

    void setupContainer(bool CABasicService_enabled,bool DENBasicService_enabled,bool VRUBasicService_enabled, bool CPBasicService_enabled, bool MCBasicService_enabled = false, bool security_enabled = false);

    // Function to setup a circular GeoArea for DENMs - it must be called at least once when sending/receiving DENMs
    // Then, it may be called as many times as desired to change the DENMs GeoArea
    // It should always be called after setupContainer(), otherwise it will generate an error
    void setDENMCircularGeoArea(double lat, double lon, int radius_meters);

    // Function to pass to the VBS the name of the file where the metrics related to VAMs have to be stored
    void setVAMmetricsfile(std::string file_name, bool collect_metrics);

    void receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from);
    vehicleData_t translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                   asn1cpp::Seq<PerceivedObject> object, int objectIndex, int newID);

    // Function to be called when this BSContainer is no longer being used (e.g., when a vehicle exits from the SUMO scenario)
    // This function should be called to automatically perform all the necessary cleanup operations (like stopping the CAM
    // dissemation, if enabled, and the GeoNetworking egoPV updates)
    void cleanup();

    Ptr<GeoNet> getGeoNet() {return m_gn;};

    Ptr<LDM> getLDM() {return m_LDM;};

    VDP* getVDP() {return m_vdp_ptr;};

  private:
    // Message reception callbacks
    std::function<void(asn1cpp::Seq<CAM>, Address, StationID_t, StationType_t, SignalInfo)> m_CAReceiveCallbackExtended;
    std::function<void(denData,Address,unsigned long,long,SignalInfo)> m_DENReceiveCallbackExtended;
    std::function<void(asn1cpp::Seq<VAM>, Address, StationID_t, StationType_t)> m_VAMReceiveCallbackExtended;
    std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> m_CPMReceiveCallbackExtended;
    std::function<void(asn1cpp::Seq<MCM>, Address, StationID_t, StationType_t, SignalInfo)> m_MCMReceiveCallbackExtended;

    // ETSI Transport and Networking layer pointers
    Ptr<btp> m_btp;
    Ptr<GeoNet> m_gn;

    // ETSI Basic Services
    // Additional Basic Services should be added here
    CABasicService m_cabs;
    DENBasicService m_denbs;
    VRUBasicService m_vrubs;
    CPBasicService m_cpbs;
    MCBasicService m_mcbs;

    bool m_real_time;
    Ptr<TraciClient> m_mobility_client;
    Ptr<MetricSupervisor> m_metric_sup_ptr;
    bool m_prrsup_beacons;

    Ptr<Socket> m_socket; // Socket for reception and transmission of messages

    // Local dynamic map
    Ptr<LDM> m_LDM;

    // Vehicle properties
    StationId_t m_station_id;
    StationType_t m_stationtype;

    // Prefix for the full SUMO vehicle ID (default: "veh")
    std::string m_sumo_vehid_prefix = "veh";
    std::string m_sumo_pedid_prefix = "ped";

    // File containing all the VAM metrics of interest
    std::string m_csv_file_name;
    bool m_VAM_metrics;

    VDP *m_vdp_ptr;
    VRUdp *m_vrudp_ptr;

    bool m_is_configured = false;
    bool m_DENMs_enabled = false;
    bool m_CAMs_enabled = false;
    bool m_VAMs_enabled = false;
    bool m_CPMs_enabled = false;
    bool m_MCMs_enabled = false;

    Ptr<SUMOSensor> m_sumo_sensor;

    std::map<int, std::map<int,int>> m_recvCPMmap;  //! Structure mapping, for each CV that we have received a CPM from, the CPM's PO ids with the ego LDM's PO ids
  };
}

#endif // BSCONTAINER_H
