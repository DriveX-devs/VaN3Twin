/** @file */
#ifndef MCBASICSERVICE_H
#define MCBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/LDM.h"
#include "signalInfoUtils.h"
#include "ns3/trajectoryPrediction.h"

extern "C" {
  #include "ns3/MCM.h"
}

//#define CURRENT_VDP_TYPE VDPTraCI

namespace ns3
{

enum ManeuverID
{
  // --- Traffic ---
  Undefined                         = 0x00,
  TransitToHumanDrivenMode          = 0x01,
  TransitToAutomatedDrivingMode     = 0x02,
  DriveStraight                     = 0x03,
  TurnLeft                          = 0x04,
  TurnRight                         = 0x05,
  UTurn                             = 0x06,
  MoveBackward                      = 0x07,
  Overtake                          = 0x08,
  Accelerate                        = 0x09,
  Slowdown                          = 0x0A,
  Stop                              = 0x0B,
  GoToLeftLane                      = 0x0C,
  GoToRightLane                     = 0x0D,
  GetOnHighway                      = 0x0E,
  ExitHighway                       = 0x0F,
  TakeTollingLane                   = 0x10,
  StopAndWait                       = 0x11,

  // --- Safety ---
  EmergencyBrakeAndStop             = 0x40,
  ResetStopAndRestartMoving         = 0x41,
  StayInLane                        = 0x42,
  ResetStayInLane                   = 0x43,
  StayAway                          = 0x44,
  ResetStayAway                     = 0x45,

  // --- Group Manoeuvre ---
  FollowMe                          = 0x80,
  ExistingGroup                     = 0x81,
  TemporarilyDisbandGroup           = 0x82,
  ConstituteTemporaryGroup          = 0x83,
  DisbandTemporaryGroup             = 0x84
};

  typedef enum {
    MCM_NO_ERROR=0,
    MCM_WRONG_INTERVAL=1,
    MCM_ALLOC_ERROR=2,
    MCM_NO_RSU_CONTAINER=3,
    MCM_ASN1_UPER_ENC_ERROR=4,
    MCM_CANNOT_SEND=5
  } MCBasicService_error_t;

  class MCSpecification: public Object
  {
  public:
    MCSpecification()
        : m_mcm_type(0), m_mcm_its_role(0), m_mcm_status(0),
          m_mcm_concept(0), m_mcm_goal(0), m_maneuver_id(Undefined), m_mcm_cost(0),
          m_vehicle_maneuver_container(false),
          m_vehicle_advise_container(false),
          m_vehicle_acknowledgement_container(false),
          m_vehicle_response_container(false),
          m_vehicle_terminator_container(false),
          m_vehicle_type{}, m_mcm_response(0) {}

    ~MCSpecification ();
    template <typename T>
    T* create()
    {
      T* new_item = (T *)CALLOC(1, sizeof(T));
      return new_item;
    }

    template <typename T, typename Container, typename Item>
    int add(T type, Container* container, Item* item)
    {
      if (ASN_SEQUENCE_ADD(container, item) != 0)
        {
          ASN_STRUCT_FREE(type, item);
          return 0;
        }
      else return 1;
    }

    bool checkContainers();

    void setAdviseContainer() {m_vehicle_advise_container = true;};
    bool getAdviseContainer() {return m_vehicle_advise_container;};
    void setManeuverContainer() {m_vehicle_maneuver_container = true;};
    bool getManeuverContainer() {return m_vehicle_maneuver_container;};
    void setAcknowledgmentContainer() {m_vehicle_acknowledgement_container = true;};
    bool getAcknowledgmentContainer() {return m_vehicle_acknowledgement_container;};
    void setResponseContainer() { m_vehicle_response_container = true; };
    bool getResponseContainer() { return m_vehicle_response_container; };
    void setTerminatorContainer() {m_vehicle_terminator_container = true;};
    bool getTerminatorContainer() {return m_vehicle_terminator_container;};
    void setMCMType(long type) {m_mcm_type = type;};
    long getMCMType() {return m_mcm_type;};
    void setMCMItsRole(long role) { m_mcm_its_role = role; };
    long getMCMItsRole() { return m_mcm_its_role; };
    void setMCMStatus(long status) { m_mcm_status = status; };
    long getMCMStatus() { return m_mcm_status; };
    void setMCMConcept(long concept) { m_mcm_concept = concept; };
    long getMCMConcept() { return m_mcm_concept; };
    void setMCMGoal(long goal) { m_mcm_goal = goal; };
    long getMCMGoal() { return m_mcm_goal; };
    void setMCMCost(long cost) { m_mcm_cost = cost; };
    long getMCMCost() { return m_mcm_cost; };
    void setMCMResponse(long response) { m_mcm_response = response; };
    long getMCMResponse() { return m_mcm_response; };
    void setManeuverID(ManeuverID id) { m_maneuver_id = id; };
    ManeuverID getManeuverID() { return m_maneuver_id; };
    void setVehicleType(Iso3833VehicleType type) { m_vehicle_type = type; };
    Iso3833VehicleType getVehicleType() { return m_vehicle_type; };
    void pushSubmaneuverDescription(SubmanoeuvreDescription* item)
    {
      m_submaneuver_description.push_back(item);
    };
    std::vector<SubmanoeuvreDescription*>& getSubmaneuverDescription()
    {
      return m_submaneuver_description;
    };
    void pushManeuverAdvice(ManoeuvreAdvice* item)
    {
      m_maneuver_advice.push_back(item);
    };
    std::vector<ManoeuvreAdvice*>& getManeuverAdvice()
    {
      return m_maneuver_advice;
    };

  private:
    long m_mcm_type;
    ManeuverID m_maneuver_id;
    long m_mcm_its_role;
    long m_mcm_status;
    long m_mcm_concept;
    long m_mcm_goal;
    long m_mcm_cost;
    bool m_vehicle_maneuver_container;
    bool m_vehicle_advise_container;
    bool m_vehicle_acknowledgement_container;
    bool m_vehicle_response_container;
    bool m_vehicle_terminator_container;
    std::vector<SubmanoeuvreDescription*> m_submaneuver_description; // For Vehicle Maneuver Container
    std::vector<ManoeuvreAdvice*> m_maneuver_advice; // For Vehicle Maneuver Container and Vehicle Advice Container

    Iso3833VehicleType m_vehicle_type;
    long m_mcm_response;
  };


  /**
   * \ingroup automotive
   * \brief This class implements the Maneuver Coordination Basic Service
   *
   * This class implements the Maneuver Coordination Basic Service (MC Basic Service) as defined in ETSI TS 103 561 V0.0.10 (2025-01).
   * The MC Basic Service is a service that allows vehicles and RSUs to exchange information about their current maneuver intentions.
   *
   */
  class MCBasicService: public Object, public SignalInfoUtils
  {
  public:
    /**
     * \brief Default constructor
     *
     * This constructor creates a MC Basic Service object with default values.
     */
    MCBasicService();
    ~MCBasicService();
    /**
     * @brief Constructor
     *
     * This constructor creates a MC Basic Service object with the given station ID and station type.
     *
     * @param fixed_stationid The station ID of the vehicle or RSU
     * @param fixed_stationtype The station type of the vehicle or RSU
     * @param vdp  The VDP object to be used by the MC Basic Service
     * @param real_time  If true, the MC Basic Service will generate MCM messages using real time timestamps
     * @param is_vehicle  If true, the MC Basic Service will generate MCM messages as a vehicle, otherwise it will generate MCM messages as an RSUs
     */
    MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp,bool real_time,bool is_vehicle);

    /**
     * @brief Constructor
     *
     * This constructor creates a MC Basic Service object with the given station ID, station type and socket.
     *
     * @param fixed_stationid The station ID of the vehicle or RSU
     * @param fixed_stationtype The station type of the vehicle or RSU
     * @param vdp  The VDP object to be used by the MC Basic Service
     * @param real_time  If true, the MC Basic Service will generate MCM messages using real time timestamps
     * @param is_vehicle  If true, the MC Basic Service will generate MCM messages as a vehicle, otherwise it will generate MCM messages as an RSUs
     * @param socket_tx   The socket used to send MCM messages
     */
    MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP* vdp,bool real_time,bool is_vehicle,Ptr<Socket> socket_tx);

    /**
     * @brief Set the station properties
     *
     * This function sets the station ID and station type of the vehicle or RSU.
     *
     * @param fixed_stationid   The station ID of the vehicle or RSU
     * @param fixed_stationtype   The station type of the vehicle or RSU
     */
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);

    /**
     * @brief  Set the fixed position of the RSU
     *
     * This function sets the fixed position of the RSU.
     *
     * @param latitude_deg
     * @param longitude_deg
     */
    void setFixedPositionRSU(double latitude_deg, double longitude_deg);

    /**
     * @brief Set the vehicle station ID
     * @param fixed_stationid
     */
    void setStationID(unsigned long fixed_stationid);
    /**
     * @brief Set the station type
     * @param fixed_stationtype
     */
    void setStationType(long fixed_stationtype);
    /**
     * @brief Set the socket used to send MCM messages
     *
     * This function passes the socket used to send MCM messages to the underlying BTP object.
     *
     * @param socket_tx   The socket used to send MCM messages
     */
    void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
    /**
     * @brief Set the socket used to receive MCM messages
     *
     * This function sets the socket used to receive MCM messages.
     *
     * @param socket_rx   The socket used to receive MCM messages
     */
    void setSocketRx(Ptr<Socket> socket_rx);
    void setRSU() {m_vehicle=false;}
    /**
     * @brief Set the VDP object
     *
     * This function sets the VDP object to be used by the MC Basic Service.
     *
     * @param vdp   The VDP object to be used by the MC Basic Service
     */
    void setVDP(VDP* vdp) {m_vdp=vdp;}
    /**
     * @brief Set the LDM object
     *
     * This function sets the LDM object to be used by the MC Basic Service.
     *
     * @param LDM   The LDM object to be used by the MC Basic Service
     */
    void setLDM(Ptr<LDM> LDM){m_LDM = LDM;}
    /**
     * @brief Set the BTP object
     *
     * This function sets the BTP object to be used by the MC Basic Service.
     *
     * @param btp   The BTP object to be used by the MC Basic Service
     */
    void setBTP(Ptr<btp> btp){m_btp = btp;}

    /**
     * @brief Callback function for processing received MCM messages
     *
     * This function is called by the BTP object when a MCM message is received.
     * It decodes the received MCM message and calls the callback application function set by the user.
     *
     * @param dataIndication   The received MCM message
     * @param from  The address of the sender
     */
    void receiveMCM(BTPDataIndication_t dataIndication, Address from);
    void changeRSUGenInterval(long RSU_GenMCM_ms) {m_RSU_GenMCM_ms=RSU_GenMCM_ms;}
    // Warning: if both the standard and extended callbacks are set, only the standard callback will be called
    /**
     * @brief Set the callback function for processing received MCM messages
     *
     * This function sets the callback function to be called when a MCM message is received.
     *
     * @param rx_callback   The callback function to be called when a MCM message is received
     */
    void addMCRxCallback(std::function<void(asn1cpp::Seq<MCM>, Address)> rx_callback) {m_MCReceiveCallback=rx_callback;}
    void addMCRxCallbackExtended(std::function<void(asn1cpp::Seq<MCM>, Address, StationId_t, StationType_t, SignalInfo)> rx_callback) {m_MCReceiveCallbackExtended=rx_callback;}
    void setRealTime(bool real_time){m_real_time=real_time;}

    /**
     * @brief Start the MCM dissemination
     *
     * This function starts the MCM dissemination process.
     */
    void startMCMDissemination();
    /**
     * @brief Start the MCM dissemination with a desynchronization interval
     *
     * This function starts the MCM dissemination process with a desynchronization interval to avoid MCM synchronization.
     *
     * @param desync_s   The desynchronization interval in seconds
     */
    void startMCMDissemination(double desync_s);

    /**
     * @brief Stop the MCM dissemination
     *
     * This function stops the MCM dissemination process.
     */
    uint64_t terminateDissemination();

    long T_GenMCMMin_ms = 100;
    long T_GenMCMMax_ms = 10000;

    void SetLogTriggering(bool log, std::string log_filename) {m_log_triggering = log; m_log_filename = log_filename;};

    // void write_log_triggering(bool condition_verified, float head_diff, float pos_diff, float speed_diff, long time_difference, std::string data_head, std::string data_pos, std::string data_speed, std::string data_time, std::string data_dcc);

    /**
     * @brief Generate and encode a MCM message
     *
     * This function generates and encodes a MCM message.
     *
     * @return MCBasicService_error_t   The error code
     */
    MCBasicService_error_t generateAndEncodeMCM(MCSpecification *specification);


  private:
    const size_t m_MaxPHLength = 23;

    /**
     * @brief Check the conditions to generate a MCM message
     *
     * This is function periodically checks the conditions to generate a MCM message according to the ETSI TS 103 561 V0.0.10 (2025-01) standard.
     */
    void checkMCMConditions();
    int64_t computeTimestampUInt64();
    /**
     * @brief Update the LDM with the received MCM message information
     * @param decodedMCM
     */
    void vLDM_handler(asn1cpp::Seq<MCM> decodedMCM);

    // std::function<void(MCM_t *, Address)> m_CAReceiveCallback;
    std::function<void(asn1cpp::Seq<MCM>, Address)> m_MCReceiveCallback;
    std::function<void(asn1cpp::Seq<MCM>, Address, Ptr<Packet>)> m_MCReceiveCallbackPkt;
    std::function<void(asn1cpp::Seq<MCM>, Address, StationId_t, StationType_t, SignalInfo)> m_MCReceiveCallbackExtended;

    Ptr<btp> m_btp; //! BTP object

    double m_T_CheckMCMGen_ms; //! MCM generation check interval

    long m_T_GenMCM_ms; //! MCM generation interval

    long m_RSU_GenMCM_ms; //! MCM generation interval for RSU ITS-Ss

    int64_t lastMCMGen; //! Last MCM generation timestamp


    bool m_real_time; //! If true, the MC Basic Service will generate MCM messages using real time timestamps

    bool m_vehicle; //! If true, the MC Basic Service will generate MCM messages as a vehicle, otherwise it will generate MCM messages as an RSU

    VDP* m_vdp; //! VDP object


    Ptr<Socket> m_socket_tx; //! Socket used to send MCM messages

    Ptr<LDM> m_LDM; //!< LDM object

    StationId_t m_station_id; //! Station ID

    StationType_t m_stationtype; //! Station type

    VDP::VDP_position_latlon_t m_prev_position;

    uint64_t m_MCM_sent;//! Number of MCMs successfully sent since the MC Basic Service has been started. The MC Basic Service can count up to 18446744073709551615 (UINT64_MAX) MCMs

    // ns-3 event IDs used to properly stop the simulation with terminateDissemination()

    EventId m_event_MCMDisseminationStart; //! Event ID for the start of the MCM dissemination

    EventId m_event_MCMCheckConditions; //! Event ID for the check of the MCM generation conditions

    EventId m_event_MCMRsuDissemination; //! Event ID for the RSU MCM dissemination


    std::vector<std::pair<ReferencePositionWithConfidence_t,PathHistoryDeltas_t>> m_refPositions; //! Reference positions for the PathHistory container

    //High frequency RSU container
    asn1cpp::Seq<RSUContainerHighFrequency> m_protectedCommunicationsZonesRSU;
    double m_RSUlon;
    double m_RSUlat;

    double m_last_transmission = 0;

    bool m_log_triggering = false;
    std::string m_log_filename;

    // Statistics: number of MCMs sent per triggering conditions
    uint64_t m_pos_sent = 0;
    uint64_t m_speed_sent = 0;
    uint64_t m_head_sent = 0;
    uint64_t m_time_sent = 0;

    long m_T_next_dcc = -1;

  };
}

#endif // MCBASICSERVICE_H
