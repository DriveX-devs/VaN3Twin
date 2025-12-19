#ifndef CPBASICSERVICE_H
#define CPBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/vdp.h"
#include "ns3/vdpTraci.h"
#include "ns3/LDM.h"
#include "ns3/ldm-utils.h"
#include "signalInfoUtils.h"


extern "C" {
  #include "ns3/CollectivePerceptionMessage.h"
}

namespace ns3
{
/**
 * \ingroup automotive
 *
 * \brief This class implements the basic service for the Collective Perception Service.
 *
 * This class implements the basic functionalities of the Collective Perception Service defined in ETSI TS 103 324 V2.1.1 (2023-06).
 * Collective Perception Messages (CPMs) are transmitted by ITS-Ss in order to share information about perceived objects (non-connected vehicles in our framework).
 * This enhances the environmental perception of CPS-enabled ITS-Ss by providing information about non-V2X-equipped road users
 * and also increases the number of information sources for V2X-equipped road users.
 *
 */
class CPBasicService: public Object, public SignalInfoUtils
{ 
public:
  /**
   * \brief Constructor
   *
   * This constructor initializes the CPBasicService object.
   */
  CPBasicService();
  /**
   * @brief Set the station ID of the ITS-S.
   * @param fixed_stationid  The station ID to be set.
   */
  void setStationID(unsigned long fixed_stationid);
  /**
   * @brief Set the station type of the ITS-S.
   * @param fixed_stationtype   The station type to be set.
   */
  void setStationType(long fixed_stationtype);
  /**
   * @brief Set the station properties of the ITS-S.
   * @param fixed_stationid
   * @param fixed_stationtype
   */
  void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
  /**
   * @brief Set the socket to be used by BTP/GeoNet for transmitting CPMs.
   * @param socket_tx The socket to be set.
   */
  void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
  /**
   * @brief Set the socket to be used for receiving CPMs.
   * @param socket_rx The socket to be set.
   */
  void setSocketRx(Ptr<Socket> socket_rx);
  /**
   * @brief Set the LDM object to be used by the CPBasicService.
   * @param LDM The LDM object to be set.
   */
  void setLDM(Ptr<LDM> LDM){m_LDM = LDM;}
  /**
   * @brief Set the VDP object to be used by the CPBasicService.
   * @param vdp The VDP object to be set.
   */
  void setVDP(VDP* vdp) {m_vdp=vdp;}
  void setTraCIclient(Ptr<TraciClient> client){m_client=client;}
  /**
   * @brief Set the BTP object to be used by the CPBasicService.
   * @param btp The BTP object to be set.
   */
  void setBTP(Ptr<btp> btp){m_btp = btp;}

  /**
   * @brief Callback function used for the processing of received CPMs.
   * @param dataIndication The received data indication from BTP/GeoNet.
   * @param from The address of the sender of the CPM.
   */
  void receiveCpm(BTPDataIndication_t dataIndication, Address from);
  void changeNGenCpmMax(int16_t N_GenCpmMax) {m_N_GenCpmMax=N_GenCpmMax;}
  /**
   * @brief Set the callback function to be used for the processing of received CPMs.
   * @param rx_callback  The callback function to be set.
   */
  void addCPRxCallback(std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address)> rx_callback) {m_CPReceiveCallback=rx_callback;}
  void addCPRxCallbackExtended(std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback) {m_CPReceiveCallbackExtended=rx_callback;}
  /**
   * @brief Specify if using the real time or simulation time for the CPM timestamps.
   * @param real_time
   */
  void setRealTime(bool real_time){m_real_time=real_time;}
  /**
   * @brief Start the dissemination of CPMs.
   */
  void startCpmDissemination();
  /**
   * @brief Stop the dissemination of CPMs.
   */
  uint64_t terminateDissemination();
  /**
   * @brief Specify if using redundancy mitigation defined for the CPM generation.
   *
   * This method is used to specify the value of the ObjectInclusionConfig value defined in ETSI TS 103 324 V2.1.1 (2023-06) Section 6.1.2.3
   * True (default) --> The inclusion rules defined in ETSI TS 103 324 V2.1.1 (2023-06) are applied
   * False --> The inclusion rules defined in ETSI TS 103 324 V2.1.1 (2023-06) are not applied
   * @param choice
   */
  void setRedundancyMitigation(bool choice){m_redundancy_mitigation = choice;}
  void disableRedundancyMitigation(){m_redundancy_mitigation = false;}

  void setCheckCpmGenMs(long nextCPM) {m_N_GenCpm=nextCPM;};

  const long T_GenCpmMin_ms = 100;
  const long T_GenCpm_ms = 100;
  const long T_GenCpmMax_ms = 1000;
  const long m_T_AddSensorInformation = 1000;

  /**
     * @brief Used for DCC Adaptive approach to set the future time to check CPM condition after an update of delta value
     * @param delta new delta value calculated through DCC adaptive approach
     */
  void toffUpdateAfterDeltaUpdate(double delta);

  /**
     * @brief Used for DCC Adaptive approach to set the future time to check CPM condition after a transmission
     * @param delta new delta value calculated through DCC adaptive approach
     */
  void toffUpdateAfterTransmission();
    std::pair <double, double>  calculateHeadingAndVelocity(double lat1, double lon1, double timestamp1,
                                                            double lat2, double lon2, double timestamp2);


private:

  void initDissemination();
  void RSUDissemination();
  void checkCpmConditions();
  /**
   * @brief Generate and encode a CPM ASN.1 message.
   */
  void generateAndEncodeCPM();
  int64_t computeTimestampUInt64();
  /**
   * @brief Evaluate the conditions for the generation of a CPM as defined in ETSI TS 103 324 V2.1.1 (2023-06) Section 6.1.2.3
   * @param it  The iterator of the vehicle data to be checked.
   * @return
   */
  bool checkCPMconditionsTypeB(std::vector<LDM::returnedVehicleData_t>::iterator it);
  bool checkCPMconditionsTypeA(std::vector<LDM::returnedVehicleData_t>::iterator it);
  double cartesian_dist(double lon1, double lat1, double lon2, double lat2);

  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address)> m_CPReceiveCallback;  //! Callback function for received CPMs
  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, Ptr<Packet>)> m_CPReceiveCallbackPkt;
  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> m_CPReceiveCallbackExtended;

  Ptr<btp> m_btp; //! BTP object

  long m_T_CheckCpmGen_ms; //! Time interval for checking the conditions for CPM generation
  long m_T_LastSensorInfoContainer; //! Time interval for adding sensor information to the CPM

  long m_T_GenCpm_ms;
  int16_t m_N_GenCpm;
  int16_t m_N_GenCpmMax;

  int64_t lastCpmGen; //! Timestamp of the last CPM generation
  int64_t lastCpmGenLowFrequency;
  int64_t lastCpmGenSpecialVehicle;

  bool m_real_time; //! Flag to specify if using real time or simulation time for the CPM timestamps
  bool m_vehicle;
  bool m_redundancy_mitigation; //! Flag to specify if using redundancy mitigation defined for the CPM generation
  VDP* m_vdp; //! VDP object
  Ptr<TraciClient> m_client;

  bool m_clustering;

  Ptr<Socket> m_socket_tx; // Socket TX

  Ptr<LDM> m_LDM; //! LDM object

  StationID_t m_station_id; //! Station ID of the ITS-S
  StationType_t m_stationtype; //! Station type of the ITS-S

  // Previous Cpm relevant values
  double m_prev_heading;
  double m_prev_distance;
  double m_prev_speed;
  std::vector<long> m_lastCPM_POs; // Last Perceived Objects included in the previous CPM


  // The CP Basic Service can count up to 18446744073709551615 (UINT64_MAX) Cpms
  uint64_t m_cpm_sent; //! Statistic: number of CPMs successfully sent since the CP Basic Service has been started

  // ns-3 event IDs used to properly stop the simulation with terminateDissemination()
  EventId m_event_cpmDisseminationStart;
  EventId m_event_cpmSend;

  std::string m_csv_name; //!< CSV log file name
  std::ofstream m_csv_ofstream_cpm; //!< CSV log stream (CPM), created using m_csv_name

  bool m_send_all_typeA;

  double m_last_transmission = 0;
  double m_Ton_pp = 0;
  double m_last_delta = 0;

    int test;
    bool testbool;
};
}
#endif // CPBASICSERVICE_H
