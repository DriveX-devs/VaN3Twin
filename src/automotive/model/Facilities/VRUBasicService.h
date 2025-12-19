//
//  VRUBasicService.h
//

#ifndef VRUBasicService_h
#define VRUBasicService_h

#include "ns3/vdp.h"
#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/LDM.h"
#include "ns3/vrudpOpenCDA.h"

extern "C" {
  #include "ns3/VAM.h"
}

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
  } triggcond_t;

    const char* triggerCondToString(triggcond_t triggerCond);

class VRUBasicService: public Object
{
public:
    VRUBasicService();
    VRUBasicService(unsigned long fixed_stationid,long fixed_stationtype,VRUdpOpenCDA* VRUdp,bool real_time);
    ~VRUBasicService();

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
    void setStationID(unsigned long fixed_stationid);
    void setStationType(long fixed_stationtype);
    void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
    void setSocketRx(Ptr<Socket> socket_rx);
    void setLDM(Ptr<LDM> LDM){m_LDM = LDM;}
    void setBTP(Ptr<btp> btp){m_btp = btp;}
    void setVRUdp(VRUdp* VRUdp) {m_VRUdp=VRUdp;}
    void setVRUdp(VRUdpOpenCDA* VRUdp) {m_VRUdpOCDA=VRUdp;}
    void setT_GenVam(long T_GenVam) {m_T_GenVam_ms = T_GenVam;}
    void setSafeLateralDistance(double safe_lat_d) {m_lat_safe_d = safe_lat_d;}
    void setSafeVerticalDistance(double safe_vert_d) {m_vert_safe_d = safe_vert_d;}
    void setRoleAndClustState(VRURole_t VRU_role, VRUClusteringstate_t ClustState);
    void setRoleAndClustState();

    void setVAMmetricsfile(std::string file_name, bool collect_metrics);

    void receiveVam(BTPDataIndication_t dataIndication, Address from);
    void addVAMRxCallback(std::function<void(asn1cpp::Seq<VAM>, Address)> rx_callback) {m_VAMReceiveCallback=rx_callback;}

    void startVamDissemination();
    void startVamDissemination(double desync_s);

    void startAccelerationComputation(long computationT_acc);

    uint64_t terminateDissemination();
    
    const long T_GenVamMin_ms = 100;
    const long T_GenVamMax_ms = 5000;

    /**
     * @brief Set the next time to check VAM condition
     * @param nextVAM The next time to check VAM condition
     */
    void setCheckVamGenMs(long nextVAM) {m_T_CheckVamGen_ms = nextVAM;};

    /**
     * @brief Used for DCC Adaptive approach to set the future time to check VAM condition after an update of delta value
     * @param delta new delta value calculated through DCC adaptive approach
     */
    void toffUpdateAfterDeltaUpdate(double delta);

    /**
     * @brief Used for DCC Adaptive approach to set the future time to check VAM condition after a transmission
     * @param delta new delta value calculated through DCC adaptive approach
     */
    void toffUpdateAfterTransmission();
    
private:
    void initDissemination();
    void checkVamConditions();
    bool checkVamRedundancyMitigation();
    VRUBasicService_error_t generateAndEncodeVam();
    void computeLongAcceleration();
    int64_t computeTimestampUInt64();
    void vLDM_handler(asn1cpp::Seq<VAM> decodedVAM);

    std::function<void(asn1cpp::Seq<VAM>, Address)> m_VAMReceiveCallback; //! Callback function for received VAMs

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
    VRUdpOpenCDA* m_VRUdpOCDA;

    // Previous VAM relevant values
    double m_prev_heading;
    //libsumo::TraCIPosition m_prev_position;
    VDP::VDP_position_cartesian_t m_prev_position;
    double m_prev_speed;

    // Safe distances
    double m_long_safe_d;
    double m_lat_safe_d;
    double m_vert_safe_d;

    // Longitudinal acceleration
    VRUdpValueConfidence<> m_long_acceleration;

    // Statistic: number of VAMs successfully sent since the VRU Basic Service has been started
    // The VRU Basic Service can count up to 18446744073709551615 (UINT64_MAX) VAMs
    uint64_t m_vam_sent;

    // Time interval for the computation of the longitudinal acceleration
    long m_computationT_acc_ms;

    // File containing all the VAM metrics of interest
    std::string m_csv_file_name;
    std::ofstream csv_VAM_ofstream;
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
    VRURole_t m_VRU_role;
    VRUClusteringstate_t m_VRU_clust_state;

    // Boolean/Enum variables to enable/disable the presence of certain optional containers in the VAM messages
    bool m_lowFreqContainerEnabled;

    double m_last_transmission = 0;
    double m_Ton_pp = 0;
    double m_last_delta = 0;

    int test;
    bool testbool;
};

}

#endif /* VRUBasicService_h */

