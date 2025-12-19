#ifndef GEONET_H
#define GEONET_H

#include "ns3/MetricSupervisor.h"
#include <stdint.h>
#include <string>
#include <map>
#include <set>
#include <mutex>
#include "ns3/vdpTraci.h"
#include "ns3/asn_utils.h"
#include "ns3/address.h"
#include "ns3/packet.h"
#include "ns3/socket.h"
#include "ns3/object.h"
#include "ns3/mac48-address.h"
#include "ns3/basic-header.h"
#include "ns3/common-header.h"
#include "ns3/tsb-header.h"
#include "ns3/shb-header.h"
#include "ns3/gbc-header.h"
#include "ns3/beacon-header.h"
#include "ns3/gn-address.h"
#include "ns3/longpositionvector.h"
#include "ns3/btpdatarequest.h"
#include "ns3/VRUdp.h"
#include "ns3/vrudpOpenCDA.h"

extern "C" {
  #include "ns3/CAM.h"
}


#define TS_MAX 4294967295 //(2^32)-1
#define TS_MAX1 4294967296 //(2^32)

namespace ns3
{
/**
 * \ingroup automotive
 * \brief This class implements the GeoNetworking protocol as defined in ETSI EN 302 636-4-1
 *
 */
  class GeoNet : public Object
  {
    public:

      typedef struct _LocTableEntry {
        /**
        *   ETSI EN 302 636-4-1 [8.1.2]
        */
        GNAddress GN_ADDR;
        Mac48Address LL_ADDR;
        uint8_t type;
        uint8_t version;
        GNlpv_t lpv; //! long position vector
        bool LS_PENDING;
        bool IS_NEIGHBOUR;
        std::set<uint16_t> DPL; //! Duplicate packet list
        long timestamp;
        uint32_t PDR;
      } GNLocTE;

      typedef struct _egoPositionVector {
        /**
        *   ETSI EN 302 636-4-1 [8.2.2]
        */
        VDP::VDP_position_latlon_t POS_EPV;
        double S_EPV;
        double H_EPV;
        long TST_EPV;
        uint32_t PAI_EPV;
      }GNegoPV;


      static TypeId GetTypeId ();
      /**
       * @brief Construct a new GeoNet object.
       *
       * Default constructor for the GeoNet class.
       */
      GeoNet();
      virtual ~GeoNet();
      /**
       * @brief Set the Station properties.
       * @param fixed_stationid
       * @param fixed_stationtype
       */
      void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
      /**
       * @brief Set the fixed position of the RSU.
       * @param latitude_deg
       * @param longitude_deg
       */
      void setFixedPositionRSU(double latitude_deg, double longitude_deg);
      /**
       * @brief Set the station ID of the ITS-S.
       * @param fixed_stationid
       */
      void setStationID(unsigned long fixed_stationid);
      /**
       * @brief Set the station type of the ITS-S.
       * @param fixed_stationtype
       */
      void setStationType(long fixed_stationtype);
      /**
       * @brief Set the VDP object.
       * @param vdp
       */
      void setVDP(VDP* vdp);
      void setVRUdp(VRUdp* vrudp);
      void setVRUdpOCDA(VRUdpOpenCDA* vrudpOpenCDA);
      /**
       * @brief Set the socket to transmit packets.
       * @param socket_tx
       */
      void setSocketTx(Ptr<Socket> socket_tx);
      /**
       * @brief Set the callback function to receive packets.
       * @param rx_callback
       */
      void addRxCallback(std::function<void(GNDataIndication_t,Address)> rx_callback) {m_ReceiveCallback=rx_callback;}
      /**
       * @brief Create GeoNet PDU with the correct headers (GBC or TSB) and send it.
       * @param dataRequest
       * @return
       */
      GNDataConfirm_t sendGN(GNDataRequest_t dataRequest);
      /**
       * @brief Receive a GeoNet PDU and send a data indication to BTP layer.
       * @param socket
       */
      void receiveGN(Ptr<Socket> socket);
      /**
       * @brief Set the Metric supervisor object.
       * @param MetricSupervisor_ptr
       */
      void setMetricSupervisor(Ptr<MetricSupervisor> metric_supervisor_ptr) {m_metric_supervisor_ptr=metric_supervisor_ptr;}
      void cleanup();
      /**
       * @brief Disable the PRR supervisor for beacons.
       *
       * This method disables the PRR supervisor for beacons, ignoring them for PRR computations.
       */
      void disablePRRsupervisorForBeacons() {m_PRRsupervisor_beacons=false;}
      void enablePRRsupervisorForBeacons() {m_PRRsupervisor_beacons=true;}

      // This static method creates a new GeoNetworking socket, starting from the ns-3 PacketSocket and properly binding/connecting it
      // It requires as input a pointer to the node to which the socket should be bound
      static Ptr<Socket> createGNPacketSocket(Ptr<Node> node_ptr);

  private:
      void LocTE_timeout(GNAddress entry_address);
      void newLocTE(GNlpv_t longPositionVector);
      void LocTUpdate(GNlpv_t lpv,std::map<GNAddress,GNLocTE>::iterator locte_it);
      void processSHB(GNDataIndication_t dataIndication,Address address);
      void processGBC(GNDataIndication_t dataIndication,Address address,uint8_t shape);
      uint8_t encodeLT(double seconds);
      double decodeLT(uint8_t lifeTime);
      bool hasNeighbour();
      GNDataConfirm_t sendSHB(GNDataRequest_t dataRequest,GNCommonHeader commonHeader,GNBasicHeader basicHeader,GNlpv_t longPV);
      GNDataConfirm_t sendGBC(GNDataRequest_t dataRequest,GNCommonHeader commonHeader,GNBasicHeader basicHeader,GNlpv_t longPV);
      GNDataConfirm_t sendBeacon(GNDataRequest_t dataRequest,GNCommonHeader commonHeader,GNBasicHeader basicHeader,GNlpv_t longPV);
      bool isInsideGeoArea(GeoArea_t geoArea);
      bool DPD(uint16_t seqNumber,GNAddress address);
      bool DAD(GNAddress address);
      void setBeacon();
      void saveRepPacket(GNDataRequest_t dataRequest);
      void maxRepIntTimeout(GNDataRequest_t dataRequest);
      int get_messageID_from_BTP_port(int16_t port);


      std::map<GNAddress,GNLocTE> m_GNLocT;///! ETSI EN 302 636-4-1 [8.1]
      std::map<GNAddress,Timer> m_GNLocTTimer; ///! Timer for every new entry in th Location Table
      template<typename MEM_PTR> void setTLocT(Timer &timer,Time delay,MEM_PTR callback_fcn,GNAddress address);

      std::map<GNDataRequest_t,std::pair<Timer,Timer>> m_Repetition_packets;///! Timers for packets with repetition interval enabled
      template<typename MEM_PTR> void setRepInt(Timer &timer,Time delay,MEM_PTR callback_fcn,GNDataRequest_t dataRequest);

      std::mutex m_LocT_Mutex;

      GNegoPV m_egoPV{}; //! Ego Position Vector ETSI EN 302 636-4-1 [8.2]
      void EPVupdate();

      uint16_t m_seqNumber; //! Sequence Number ETSI EN 302 636-4-1 [8.3]

      GNAddress m_GNAddress; //! GeoNetworking Address ETSI EN 302 636-4-1

      Ptr<Socket> m_socket_tx;

      std::function<void(GNDataIndication_t,Address)> m_ReceiveCallback;


      VDP* m_vdp{}; //! Pointer to the VDP object
      VRUdp* m_vrudp{}; //! Pointer to the VRUdp object
      VRUdpOpenCDA* m_vrudpOCDA{}; //! Pointer to the VRUdp object from OpenCDA

      StationID_t m_station_id; //! Station ID
      StationType_t m_stationtype; //! Station Type

      EventId m_event_EPVupdate;
      EventId m_event_Beacon;

      //ETSI 302 636-4-1 ANNEX H: GeoNetworking protocol constans
      Mac48Address m_GnLocalGnAddr;
      uint8_t m_GnLocalAddrCongMethod = 1; ///! MANAGED
      uint8_t m_GnPtotocolVersion = 1;
      bool m_GnIsMobile=true; ///!To set wether if Mobile(1) or Stationary(0)
      uint8_t m_GnIfType = 1;
      double m_GnMinUpdateFrequencyEPV = 1000;
      uint32_t m_GnPaiInterval = 80;
      uint32_t m_GnMaxSduSize = 1398;
      uint8_t m_GnMaxGeoNetworkingHeaderSize = 88;
      uint8_t m_GnLifeTimeLocTE = 20; ///! seconds
      uint8_t m_GnSecurity = 0; ///!Disabled
      uint8_t m_GnSnDecapResultHandling = 0; //!STRICT
      uint8_t m_GnLocationServiceMaxRetrans = 10;
      uint16_t m_GnLocationServiceRetransmitTimer = 1000;
      uint16_t m_GnLocationServicePacketBufferSize = 1024;
      uint16_t m_GnBeaconServiceRetransmitTimer = 3000;
      uint16_t m_GnBeaconServiceMaxJItter = m_GnBeaconServiceRetransmitTimer/4;
      uint8_t m_GnDefaultHopLimit = 10;
      uint8_t m_GnDPLLength = 8;
      uint16_t m_GNMaxPacketLifetime = 600;
      uint8_t m_GnDefaultPacketLifetime = 60 ; ///! seconds (0xf2)
      uint16_t m_GNMaxPacketDataRate = 100;
      uint16_t m_GNMaxPacketDataRateEmaBeta = 90;
      uint16_t m_GNMaxGeoAreaSize = 10;
      uint16_t m_GNMinPacketRepetitionInterval = 100;
      uint16_t m_GNNonAreaForwardingAlgorithm = 1; //! GREEDY
      uint16_t m_GNAreaForwardingAlgorithm = 1;
      uint16_t m_GNCbfMinTime = 1;
      uint16_t m_GNCbfMaxTime = 100;
      uint16_t m_GnDefaultMaxCommunicationRange = 1000;
      uint16_t m_GnBroadcastCBFDefSectorAngle = 30;
      uint16_t m_GnUcForwardingPacketBufferSize = 256;
      uint16_t m_GnBcForwardingPacketBufferSize = 1024;
      uint16_t m_FnCbfPacketBufferSize = 256;
      uint16_t m_GnDefaultTrafficClass = 0;
      bool m_RSU_epv_set = false;

      Ptr<MetricSupervisor> m_metric_supervisor_ptr = nullptr;
      bool m_PRRsupervisor_beacons = true;

      bool m_EPVupdate_running = true;

  };
}
#endif // GEONET_H
