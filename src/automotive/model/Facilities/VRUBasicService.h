//
//  VRUBasicService.h
//

#ifndef VRUBasicService_h
#define VRUBasicService_h

#include "ns3/btpdatarequest.h"
#include "ns3/vdp.h"
#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/LDM.h"

extern "C" {
  #include "ns3/VAM.h"
  #include "ns3/CAM.h"
}

#define MAX_TIP_MAP_TIME 10000 // Maximum time in milliseconds to keep TIP values in the map (10 seconds)
#define DELTA_TIP 0.1

namespace ns3
{

typedef enum{
    VAM_NO_ERROR = 0,
    VAM_WRONG_INTERVAL = 1,
    VAM_ALLOC_ERROR = 2,
    VAM_ASN1_UPER_ENC_ERROR = 3,
    VAM_CANNOT_SEND = 4,
} VRUBasicService_error_t;

  typedef enum{
    VRU_ROLE_OFF = 0,
    VRU_ROLE_ON = 1,
  } VRURole_t;

  typedef enum{
    VRU_IDLE = 0,
    VRU_ACTIVE_STANDALONE = 1,
    VRU_ACTIVE_CLUSTER_LEADER = 2,
    VRU_PASSIVE = 3,
  } VRUClusteringstate_t;

  typedef enum{
    NOT_VALID = -1,
    DISSEMINATION_START = 0,
    MAX_TIME_ELAPSED = 1,
    HEADING_CHANGE = 2,
    POSITION_CHANGE = 3,
    SPEED_CHANGE = 4,
    SAFE_DISTANCES = 5,
    TIP_TRIGGER = 6,
  } triggcond_t;

class VRUBasicService: public Object
{
public:
    VRUBasicService();
    VRUBasicService(unsigned long fixed_stationid,long fixed_stationtype,VRUdp* VRUdp,bool real_time);
    ~VRUBasicService();

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
    void setSocketRx(Ptr<Socket> socket_rx);
    void setLDM(Ptr<LDM> LDM){m_LDM = LDM;}
    void setBTP(Ptr<btp> btp){m_btp = btp;}
    void setVRUdp(VRUdp* VRUdp) {m_VRUdp=VRUdp;}
    void setT_GenVam(long T_GenVam) {m_T_GenVam_ms = T_GenVam;}
    void setSafeLateralDistance(double safe_lat_d) {m_lat_safe_d = safe_lat_d;}
    void setSafeVerticalDistance(double safe_vert_d) {m_vert_safe_d = safe_vert_d;}

    void setTTCMax(double ttc_max) {m_TTC_max = ttc_max;}
    void setTTCMin(double ttc_min) {m_TTC_min = ttc_min;}
    void setSTCMin(double stc_min) {m_STC_min = stc_min;}
    void setTIPThreshold(double tip_th) {m_TIP_th = tip_th;}
    void setModality(std::string modality) {m_tip_modality = modality;}
    void setK(double k) {m_TTC_k = k;}
    void setSigma(double sigma) {m_TTC_sigma = sigma;}

    void setVAMmetricsfile(std::string file_name, bool collect_metrics);

    void receiveVam(BTPDataIndication_t dataIndication, Address from);
    void addVAMRxCallback(std::function<void(asn1cpp::Seq<VAM>, Address)> rx_callback) {m_VAMReceiveCallback=rx_callback;}
    void addVAMRxCallbackExtended(std::function<void(asn1cpp::Seq<VAM>, Address, StationID_t, StationType_t)> rx_callback) {m_VAMReceiveCallbackExtended=rx_callback;}

    void startVamDissemination();
    void startVamDissemination(double desync_s);

    void startAccelerationComputation(long computationT_acc);

    uint64_t terminateDissemination();

    double getTTCMax() {return m_TTC_max;}
    double getTTCMin() {return m_TTC_min;}
    double getSTCMin() {return m_STC_min;}
    std::string getTIPModality() {return m_tip_modality;}
    double getTIPThreshold() {return m_TIP_th;}
    double getK() {return m_TTC_k;}
    double getSigma() {return m_TTC_sigma;}

    void addNewTIPToMap(uint64_t id, double time, double tip);
    void updatePreviousSentTIPOnMap(uint64_t id, double previous_tip);
    void cleanTIPMap(double time);
    void searchFirstEightTIPs();
    
    const long T_GenVamMin_ms = 100;
    const long T_GenVamMax_ms = 5000;
    const long T_GenVamLFMin_ms = 2000;

    void SetLogTriggering(bool log, std::string log_filename) {m_log_triggering = log; m_log_filename = log_filename;};

    void write_log_triggering(bool condition_verified, bool vamredmit_verified, float head_diff, float pos_diff, float speed_diff, long time_difference, std::string data_head, std::string data_pos, std::string data_speed, std::string data_safed, std::string data_tip, std::string data_time, std::string data_vamredmit, std::string data_dcc);

    std::string printMinDist(double minDist) {
      return ((minDist>-DBL_MAX && minDist<MAXFLOAT) ? std::to_string(minDist) : "unavailable");
    }
    
    void VRUreceivedNewMessageForTIP(BTPDataIndication_t dataIndication, bool veh);
    std::tuple<double, double> compute_ttc_stc(vehicleData_t vehdata);
    void setUseTIP() {m_use_tip = true;};


private:
    void initDissemination();
    void checkVamConditions();
    bool checkVamRedundancyMitigation();
    VRUBasicService_error_t generateAndEncodeVam();
    void computeLongAcceleration();
    int64_t computeTimestampUInt64();
    void vLDM_handler(asn1cpp::Seq<VAM> decodedVAM);

    std::function<void(asn1cpp::Seq<VAM>, Address)> m_VAMReceiveCallback;
    std::function<void(asn1cpp::Seq<VAM>, Address, StationID_t, StationType_t)> m_VAMReceiveCallbackExtended;

    Ptr<btp> m_btp;

    Ptr<Socket> m_socket_tx; // Socket TX

    Ptr<LDM> m_LDM;
    
    long m_T_GenVam_ms;
    long m_T_CheckVamGen_ms;
    int64_t lastVamGen;

    int16_t m_N_GenVam_red;
    int16_t m_N_GenVam_max_red;
    
    StationID_t m_station_id;
    StationType_t m_stationtype;

    bool m_real_time;

    VRUdp* m_VRUdp;

    // Previous VAM relevant values
    double m_prev_heading;
    libsumo::TraCIPosition m_prev_position;
    double m_prev_lat;
    double m_prev_lon;
    double m_prev_speed;

    // Safe distances
    double m_long_safe_d;
    double m_lat_safe_d;
    double m_vert_safe_d;

    // Trajectory Interception Probability parameters and thresholds
    std::string m_tip_modality;
    double m_TTC_k;
    double m_TTC_max;
    double m_TTC_sigma;
    double m_TTC_min;
    double m_STC_min;
    double m_TIP_th;

    // Longitudinal acceleration
    VRUdpValueConfidence<> m_long_acceleration;

    // Statistic: number of VAMs successfully sent since the VRU Basic Service has been started
    // The VRU Basic Service can count up to 18446744073709551615 (UINT64_MAX) VAMs
    uint64_t m_vam_sent;

    // Time interval for the computation of the longitudinal acceleration
    long m_computationT_acc_ms;

    // File containing all the VAM metrics of interest
    std::string m_csv_file_name;
    bool m_VAM_metrics;

    // Triggering condition
    triggcond_t m_trigg_cond;

    // Variable containing the distance of the nearest vehicle and pedestrian from the current pedestrian
    std::vector<distance_t> m_min_dist;

    // ns-3 event IDs used to properly stop the simulation with terminateDissemination()
    EventId m_event_vamDisseminationStart;
    EventId m_event_vamCheckConditions;
    EventId m_event_startLongAccelerationComputation;
    EventId m_event_computeLongAcceleration;

    // VRU state variables
    int m_VRU_role;
    int m_VRU_clust_state;

    // Boolean/Enum variables to enable/disable the presence of certain optional containers in the VAM messages
    bool m_lowFreqContainerEnabled;

    double m_last_transmission = 0;
    double m_last_vam_gen_LF = -1;
    int64_t m_T_GenVamLFMin_ms;

    bool m_log_triggering = false;
    std::string m_log_filename;

    // Statistics: number of VAMs sent per triggering conditions
    uint64_t m_pos_sent = 0;
    uint64_t m_speed_sent = 0;
    uint64_t m_head_sent = 0;
    uint64_t m_safedist_sent = 0;
    uint64_t m_time_sent = 0;
    uint64_t m_tip_sent;

    long m_T_next_dcc = -1;

    std::unordered_map<uint64_t, std::tuple<double, double, double>> m_tip_map;
    uint8_t m_tip_map_size = 0;
    std::vector<std::tuple<uint64_t, double>> m_tip_array;
    bool m_use_tip = false;
};

}

#endif /* VRUBasicService_h */

