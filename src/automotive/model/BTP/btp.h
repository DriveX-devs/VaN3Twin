#ifndef BTP_H
#define BTP_H
#include "ns3/geonet.h"
#include "ns3/socket.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/vdpTraci.h"
#include "ns3/asn_utils.h"
#include "ns3/btpHeader.h"
#include "ns3/tsb-header.h"
#include "ns3/gn-address.h"
#include "ns3/btpdatarequest.h"

#define CA_PORT 2001
#define DEN_PORT 2002
#define MC_PORT 2020
#define IVIM_PORT 2006
#define CP_PORT 2009
#define VA_PORT 2018

namespace ns3
{
/**
 * \ingroup automotive
 *
 * \brief This class implements the Basic Transport Protocol (BTP) as defined in ETSI EN 302 636-5-1
 *  The btp class provides an interface for transmitting and receiving BTP messages,
 *  integrating with GeoNetworking protocols for vehicular communication.
 */
  class btp : public Object
  {

  public:
    static TypeId GetTypeId();
    /**
     * @brief Construct a new btp object.
     *
     * Default constructor for the btp class.
     */
    btp();
    virtual ~btp();
    /**
     * @brief Sets the associated GeoNet pointer.
     *
     * @param geoNet Pointer to GeoNet instance.
     */
    void setGeoNet(Ptr<GeoNet> geoNet){m_geonet = geoNet;}

    /**
     * @brief Set the station properties.
     *
     * @param fixed_stationid   The station ID of the ITS-S.
     * @param fixed_stationtype  The station type of the ITS-S.
     */
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype){m_geonet->setStationProperties(fixed_stationid,fixed_stationtype);}
    /**
     * @brief Set the fixed position of the RSU for GeoNet object.
     *
     * @param latitude_deg  The latitude of the RSU.
     * @param longitude_deg The longitude of the RSU.
     */
    void setFixedPositionRSU(double latitude_deg, double longitude_deg) {m_geonet->setFixedPositionRSU(latitude_deg,longitude_deg);}
    /**
     * @brief Set the station ID of the ITS-S for GeoNet object.
     *
     * @param fixed_stationid The station ID of the ITS-S.
     */
    void setStationID(unsigned long fixed_stationid) {m_geonet->setStationID(fixed_stationid);}
    /**
     * @brief Set the station type of the ITS-S for GeoNet object.
     *
     * @param fixed_stationtype The station type of the ITS-S.
     */
    void setStationType(long fixed_stationtype) {m_station_type = fixed_stationtype; m_geonet->setStationType(fixed_stationtype);}
    /**
     * @brief Set the VDP object for GeoNet object.
     *
     * @param vdp Pointer to VDP object.
     */
    void setVDP(VDP* vdp){m_geonet->setVDP(vdp);}
    void setVRUdp(VRUdp* vrudp){m_geonet->setVRUdp(vrudp);}
    void setSocketTx(Ptr<Socket> socket_tx) {m_geonet->setSocketTx(socket_tx);}
    void setSocketRx(Ptr<Socket> socket_rx);
    /**
     * @brief Add a callback for CAM message reception.
     * @param rx_callback
     */
    void addCAMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_cam_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for DENM message reception.
     * @param rx_callback
     */
    void addDENMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_denm_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for IVIM message reception.
     * @param rx_callback
     */
    void addIVIMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_ivim_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for CPM message reception.
     * @param rx_callback
     */
    void addCPMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_cpm_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for VAM message reception.
     * @param rx_callback
     */
    void addVAMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_vam_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for MCM message reception.
     * @param rx_callback
     */
    void addMCMRxCallback(std::function<void(BTPDataIndication_t,Address)> rx_callback) {m_mcm_ReceiveCallback=rx_callback;}
    /**
     * @brief Add a callback for VRU message reception for TIP calculation.
     * @param rx_callback
     */
    void addVRURxCallback(std::function<void(BTPDataIndication_t,bool)> rx_callback) {m_vru_ReceiveCallback_tip = rx_callback;}
    /**
     * @brief Add BTP headers and pass a data request to the GeoNet object.
     *
     * @param dataRequest The BTPDataRequest_t structure containing the BTP message to be sent.
     */
    std::tuple<GNDataConfirm_t, MessageId_t> sendBTP(BTPDataRequest_t dataRequest, int priority, MessageId_t message_id);
    /**
     * @brief Receive a BTP message from GeoNet and pass it to the appropriate callback.
     *
     * @param dataIndication The GNDataIndication_t structure containing the BTP message.
     * @param address The address of the sender of the BTP message.
     */
    void receiveBTP(GNDataIndication_t, Address address);
    void cleanup();

  private:

    Ptr<GeoNet> m_geonet; //! Pointer to the GeoNet object.

    long m_station_type;

    std::function<void(BTPDataIndication_t,Address)> m_cam_ReceiveCallback; //! Callback for CAM message reception.
    std::function<void(BTPDataIndication_t,Address)> m_denm_ReceiveCallback; //! Callback for DENM message reception.
    std::function<void(BTPDataIndication_t,Address)> m_ivim_ReceiveCallback; //! Callback for IVIM message reception.
    std::function<void(BTPDataIndication_t,Address)> m_cpm_ReceiveCallback;   //! Callback for CPM message reception.
    std::function<void(BTPDataIndication_t,Address)> m_vam_ReceiveCallback;  //! Callback for VAM message reception.
    std::function<void(BTPDataIndication_t,Address)> m_mcm_ReceiveCallback;  //! Callback for MCM message reception.

    std::function<void(BTPDataIndication_t, bool)> m_vru_ReceiveCallback_tip;

  };
}

#endif // BTP_H
