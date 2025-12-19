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
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include "geonet.h"
#include "ns3/log.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"
#include <cmath>
#include "ns3/ipv4-header.h"
#define SN_MAX 65536

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE("GeoNet");

  TypeId
  GeoNet::GetTypeId (void)
  {
    static TypeId tid =
        TypeId("ns3::GeoNet")
        .SetParent <Object>()
        .AddConstructor <GeoNet>();
    return tid;
  }

  GeoNet::~GeoNet ()
  {
    NS_LOG_FUNCTION(this);
  }

  GeoNet::GeoNet()
  {
    m_socket_tx = nullptr;
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_seqNumber = 0;
    m_GNAddress = GNAddress();
    // ETSI EN 302 636-4-1 [8.2.3] : At start-up all elements of egoPV should be set to 0
    m_egoPV = {};

    m_RSU_epv_set=false;
    m_metric_supervisor_ptr = nullptr;

    m_PRRsupervisor_beacons = true;
  }

  void
  GeoNet::cleanup()
  {
    Simulator::Cancel(m_event_EPVupdate);
    Simulator::Cancel(m_event_Beacon);
    m_EPVupdate_running=false;
    m_socket_tx->ShutdownRecv ();
  }

  void
  GeoNet::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype)
  {
    m_station_id=fixed_stationid;
    m_stationtype=fixed_stationtype;
    if(fixed_stationtype==StationType_roadSideUnit)
      {
        m_GnIsMobile=false;
      }
    // ETSI EN 302 636-4-1 [10.2.2] : the egoPV shall be updated with a minimum freq of th GN constant itsGNminUpdateFrequencyEPV
    m_GNAddress = m_GNAddress.MakeManagedconfiguredAddress (m_GnLocalGnAddr,m_stationtype); //! Initial address config on MANAGED(1) mode ETSI EN 302 636-4-1 [10.2.1.3]
    m_event_Beacon.Cancel();
    m_event_Beacon = Simulator::Schedule(MilliSeconds(1),&GeoNet::setBeacon,this); //Should be at start-up but set with a little delay
  }

  void
  GeoNet::setFixedPositionRSU(double latitude_deg, double longitude_deg)
  {
    if(m_stationtype==StationType_roadSideUnit)
    {
        //Set the position of the RSU
        m_egoPV.POS_EPV.lon = longitude_deg;
        m_egoPV.POS_EPV.lat = latitude_deg;
        m_RSU_epv_set = true;
        EPVupdate ();// Update EPV at startup, only for the timestamp
    }
    else
    {
      NS_FATAL_ERROR("Error: trying to set fixed RSU position on a non-RSU node");
    }
  }

  void
  GeoNet::setStationID(unsigned long fixed_stationid)
  {
    m_station_id=fixed_stationid;
  }

  void
  GeoNet::setStationType(long fixed_stationtype)
  {
    m_stationtype=fixed_stationtype;
    if(fixed_stationtype==StationType_roadSideUnit) m_GnIsMobile=false;
  }

  void
  GeoNet::setVDP (VDP* vdp)
  {
    m_vdp = vdp;
    EPVupdate ();// Update EPV at startup
  }

  void
  GeoNet::setVRUdp (VRUdp* vrudp)
  {
    m_vrudp = vrudp;
    EPVupdate ();// Update EPV at startup
  }

  void
  GeoNet::setSocketTx(Ptr<Socket> socket_tx)
  {
    m_socket_tx = socket_tx;
    // ETSI EN 302 636-4-1 [10.2.1.3.2]
    m_GnLocalGnAddr = getGNMac48(m_socket_tx->GetNode ()->GetDevice (0)->GetAddress ());
  }

  void
  GeoNet::setBeacon ()
  {
    //Setting GNdataRequest for beacon
    GNDataRequest_t dataRequest = {};
    dataRequest.upperProtocol = ANY_UP;
    dataRequest.GNType = BEACON;
    dataRequest.GNCommProfile = UNSPECIFIED;
    dataRequest.GNMaxLife = 1;
    dataRequest.GNMaxHL = 1;
    dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
    dataRequest.lenght = 0;
    Ptr<Packet> packet = Create<Packet>();
    dataRequest.data = packet;
    sendGN (dataRequest);
  }

  void
  GeoNet::EPVupdate ()
  {
    
    m_egoPV.TST_EPV = (compute_timestampIts (true) + 37000) % TS_MAX1;
    
    if(!(m_stationtype==StationType_roadSideUnit) && m_EPVupdate_running==true)
    {
      if(m_stationtype == StationType_pedestrian){
          m_egoPV.S_EPV = m_vrudpOCDA->getPedSpeedValue ();
          m_egoPV.H_EPV = m_vrudpOCDA->getPedHeadingValue ();
          m_egoPV.POS_EPV.lat = m_vrudpOCDA->getPedPosition().lat;
          m_egoPV.POS_EPV.lon = m_vrudpOCDA->getPedPosition().lon;
          m_egoPV.POS_EPV.alt = m_vrudpOCDA->getPedPosition().alt;
        } else{
          m_egoPV.S_EPV = m_vdp->getSpeedValue ();
          m_egoPV.H_EPV = m_vdp->getHeadingValue ();
          m_egoPV.POS_EPV = m_vdp->getPosition();
        }
    }
    // ETSI EN 302 636-4-1 [10.2.2]
    //Schedule the next egoPV update with GNMinUpdateFrequencyEPV protocol constant (1000ms)
    m_event_EPVupdate = Simulator::Schedule(MilliSeconds (m_GnMinUpdateFrequencyEPV),&GeoNet::EPVupdate,this); //! ETSI EN 302 636-4-1 [10.2.2.2]
  }

  uint8_t
  GeoNet::encodeLT (double seconds)
  {
    uint8_t base,multiplier,retval;
    //Encoding of lifeTime field for Basic Header as specified in ETSI EN 302 636-4-1 [9.6.4]
    if (seconds >= 630.0)
    {
      base = 0x03;
      multiplier = seconds / 100.0;
    }
    else if (seconds >= 63.0)
    {
      base = 0x02;
      multiplier = seconds / 10.0;
    }
    else if (seconds >= 3.15)
    {
      base = 0x01;
      multiplier = seconds / 1.0;
    }
    else
    {
      base = 0x00;
      multiplier = seconds / 0.050;
    }
    retval = (multiplier << 2) | base;
    return retval;
  }

  double
  GeoNet::decodeLT (uint8_t lifeTime)
  {
    uint8_t base,multiplier;
    double seconds;

    base = lifeTime & 0x03;
    multiplier = (lifeTime & 0xFC) >> 2;

    switch (base)
    {
      case 0:
          seconds = multiplier * 0.050;
          break;
      case 1:
          seconds = multiplier * 1; // Put 1 just for completion
          break;
      case 2:
          seconds = multiplier * 10.0;
          break;
      case 3:
          seconds = multiplier * 100.0;
          break;
      default:
          NS_LOG_ERROR("GeoNet: UNABLE TO DECODE LIFETIME FIELD ");
          break;
    };

    return seconds;
  }

  GNDataConfirm_t
  GeoNet::sendGN (GNDataRequest_t dataRequest)
  {
    GNDataConfirm_t dataConfirm = ACCEPTED;
    GNBasicHeader basicHeader;
    GNCommonHeader commonHeader;
    GNlpv_t longPV;

    if(m_stationtype==StationType_roadSideUnit && m_RSU_epv_set==false)
    {
      NS_FATAL_ERROR("Error: no position has been set for an RSU object. Please use setFixedPositionRSU() on the Facilities Layer object.");
    }

    if(dataRequest.lenght > m_GnMaxSduSize)
    {
      return MAX_LENGHT_EXCEEDED;
    }
    if(dataRequest.GNMaxLife > m_GNMaxPacketLifetime)
    {
       return MAX_LIFE_EXCEEDED;
    }
    if(dataRequest.GNRepInt != 0)
    {
      if(dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval)
      {
        return REP_INTERVAL_LOW;
      }
    }

    //Basic Header field setting according to ETSI EN 302 636-4-1 [10.3.2]
    basicHeader.SetVersion (m_GnPtotocolVersion);
    //Security option not implemented
    basicHeader.SetNextHeader (1);//! Next Header: Common Header (1)
    if(dataRequest.GNMaxLife != 0)
    {
      basicHeader.SetLifeTime(encodeLT(dataRequest.GNMaxLife));
    }
    else
    {
      basicHeader.SetLifeTime(encodeLT(m_GnDefaultPacketLifetime));
    }
    if(dataRequest.GNMaxHL != 0)
    {
      basicHeader.SetRemainingHL (dataRequest.GNMaxHL);
    }
    else
    {
      basicHeader.SetRemainingHL (m_GnDefaultHopLimit);
    }

    //Common Header field setting according to ETSI EN 302 636-4-1 [10.3.4]
    commonHeader.SetNextHeader(dataRequest.upperProtocol); //0 if ANY(beacon) 1 if btp-a or 2 if btp-b
    commonHeader.SetHeaderType(dataRequest.GNType);

    // Initialize the header subtype to 0
    commonHeader.SetHeaderSubType(0);

    if((dataRequest.GNType == GBC) || (dataRequest.GNType == GAC) )
      commonHeader.SetHeaderSubType(dataRequest.GnAddress.shape);
    else if(dataRequest.GNType == TSB)
      {
        if(dataRequest.GNMaxHL>1)
          commonHeader.SetHeaderSubType(1); //For now shouldn't happen
      }

    commonHeader.SetTrafficClass (dataRequest.GNTraClass);
    commonHeader.SetFlag(m_GnIsMobile);

    if(dataRequest.GNMaxHL != 0)// Equal to the remaining hop limit
    {
      commonHeader.SetMaxHL (dataRequest.GNMaxHL);
    }
    else
    {
      commonHeader.SetMaxHL(m_GnDefaultHopLimit);
    }

    commonHeader.SetPayload (dataRequest.lenght);

    longPV.GnAddress = m_GNAddress;
    longPV.TST = m_egoPV.TST_EPV;
    longPV.latitude = (int32_t) (m_egoPV.POS_EPV.lat*DOT_ONE_MICRO);
    longPV.longitude = (int32_t) (m_egoPV.POS_EPV.lon*DOT_ONE_MICRO);
    longPV.positionAccuracy = false;
    longPV.speed = (int16_t) (m_egoPV.S_EPV*100); // [m/s] to [0.01 m/s]
    longPV.heading = (uint16_t) (m_egoPV.H_EPV*10);// [degrees] to [0.1 degrees]

    switch(dataRequest.GNType)
    {
    case BEACON:
      if(commonHeader.GetHeaderSubType ()==0) dataConfirm = sendBeacon (dataRequest,commonHeader,basicHeader,longPV);
      break;
    case GBC:
      dataConfirm = sendGBC (dataRequest,commonHeader,basicHeader,longPV);
      break;
    case TSB:
      if(commonHeader.GetHeaderSubType ()==0) dataConfirm = sendSHB (dataRequest,commonHeader,basicHeader,longPV);
      break;
    default:
      NS_LOG_ERROR("GeoNet packet not supported");
      dataConfirm = UNSPECIFIED_ERROR;
    }
    return dataConfirm;
  }

  GNDataConfirm_t
  GeoNet::sendSHB (GNDataRequest_t dataRequest,GNCommonHeader commonHeader, GNBasicHeader basicHeader,GNlpv_t longPV)
  {
    SHBheader header;
    //1) Create SHB GN-PDU with SHB header setting according to ETSI EN 302 636-4-1 [10.3.10.2]
    //a) and b) already done
    //c) SHB extended header
    header.SetLongPositionV (longPV);
    dataRequest.data->AddHeader (header);

    dataRequest.data->AddHeader (commonHeader);
    dataRequest.data->AddHeader (basicHeader);

    //2)Security setting -not implemeted yet-
    //3)If not suitable neighbour exist in the LocT and the SCF for the traffic class is set:

    if((dataRequest.GNTraClass > 128) && (!hasNeighbour ()))
    {
      //a)Buffer the SHB packet in the BC forwarding buffer and omit execution of further steps
      return UNSUPPORTED_TRA_CLASS;//Not implemented yet
    }
    //4)If the optional repetition interval paramter in the GN-dataRequest parameter is set
    if(dataRequest.GNRepInt != 0)
    {
      if(dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) return REP_INTERVAL_LOW;
      //a)save the SHB packet
      saveRepPacket (dataRequest);
    }
    //5)Media dependent procedures -Omited-
    //6)Pass the GN-PDU to the LL protocol entity
    if(m_socket_tx==NULL)
    {
      NS_LOG_ERROR("GeoNet: SOCKET NOT FOUND ");
      return UNSPECIFIED_ERROR;
    }

    if(m_metric_supervisor_ptr!=nullptr)
    {
      uint8_t *buffer = new uint8_t[dataRequest.data->GetSize ()];

      dataRequest.data->CopyData (buffer,dataRequest.data->GetSize ());

      int messagetype = get_messageID_from_BTP_port (dataRequest._messagePort);

      m_metric_supervisor_ptr->signalSentPacket (MetricSupervisor::bufToString (buffer,dataRequest.data->GetSize ()),m_egoPV.POS_EPV.lat,m_egoPV.POS_EPV.lon,m_station_id, static_cast<MetricSupervisor::messageType_e>(messagetype));

      delete[] buffer;
    }

    if(m_socket_tx->Send (dataRequest.data)==-1)
      {
        NS_LOG_ERROR("Cannot send SHB packet ");
        return UNSPECIFIED_ERROR;
      }
    //7)reset beacon timer to prevent dissemination of unnecessary beacon packet
    m_event_Beacon.Cancel ();
    double T_beacon = m_GnBeaconServiceRetransmitTimer + (rand()% m_GnBeaconServiceMaxJItter);
    m_event_Beacon = Simulator::Schedule(MilliSeconds(T_beacon),&GeoNet::setBeacon,this);
    return ACCEPTED;
  }

  GNDataConfirm_t
  GeoNet::sendGBC (GNDataRequest_t dataRequest,GNCommonHeader commonHeader, GNBasicHeader basicHeader,GNlpv_t longPV)
  {
    GBCheader header;
    //1) Create SHB GN-PDU with GBC header setting according to ETSI EN 302 636-4-1 [10.3.11.2]
    //a) and b) already done
    //GBC extended header
    header.SetSeqNumber (m_seqNumber);
    header.SetLongPositionV (longPV);
    header.SetGeoArea(dataRequest.GnAddress);

    dataRequest.data->AddHeader (header);

    dataRequest.data->AddHeader (commonHeader);
    dataRequest.data->AddHeader (basicHeader);
    /*
     * 2)If not suitable neighbour exist in the LocT and the SCF for the traffic class is set:
     * a)Buffer the SHB packet in the BC forwarding buffer and omit execution of further steps
    */
    if((dataRequest.GNTraClass >= 128) && (!hasNeighbour ()))
    {
      return UNSUPPORTED_TRA_CLASS;//Not implemented yet
    }
    //!3)Execute forwarding algorithm selection procedure, not implemented because RSU in our case will always be inside the target area
    //4) if packet is buffered in any of the forwarding packets, omit further steps
    if(m_GNAreaForwardingAlgorithm == 2)
    {
      /* push packet into CBF buffer */
      return UNSPECIFIED_ERROR;
    }
    //!5)Security profile settings not implemented
    //6)If the optional repetition interval paramter in the GN-dataRequest parameter is set
    if(dataRequest.GNRepInt != 0)
    {
      if(dataRequest.GNRepInt < m_GNMinPacketRepetitionInterval) return REP_INTERVAL_LOW;
      //a)save the SHB packet
      saveRepPacket(dataRequest);
    }
    //!7)Media dependent procedures -Omited-
    //8)Pass the GN-PDU to the LL protocol entity
    if(m_socket_tx==NULL)
    {
      NS_LOG_ERROR("GeoNet: SOCKET NOT FOUND ");
      return UNSPECIFIED_ERROR;
    }

    if(m_metric_supervisor_ptr!=nullptr)
    {
      uint8_t *buffer = new uint8_t[dataRequest.data->GetSize ()];

      dataRequest.data->CopyData (buffer,dataRequest.data->GetSize ());

      int messagetype = get_messageID_from_BTP_port (dataRequest._messagePort);

      m_metric_supervisor_ptr->signalSentPacket (MetricSupervisor::bufToString (buffer,dataRequest.data->GetSize ()),m_egoPV.POS_EPV.lat,m_egoPV.POS_EPV.lon,m_station_id, static_cast<MetricSupervisor::messageType_e>(messagetype));

      delete[] buffer;
    }

    if(m_socket_tx->Send (dataRequest.data)==-1)
    {
      NS_LOG_ERROR("Cannot send GBC packet ");
      return UNSPECIFIED_ERROR;
    }

    //*reset beacon timer to prevent dissemination of unnecessary beacon packet
    m_event_Beacon.Cancel ();
    double T_beacon = m_GnBeaconServiceRetransmitTimer + (rand()% m_GnBeaconServiceMaxJItter);
    m_event_Beacon = Simulator::Schedule(MilliSeconds(T_beacon),&GeoNet::setBeacon,this);

    //Update sequence number
    m_seqNumber = (m_seqNumber+1)% SN_MAX;
    return ACCEPTED;
  }

  GNDataConfirm_t
  GeoNet::sendBeacon(GNDataRequest_t dataRequest,GNCommonHeader commonHeader, GNBasicHeader basicHeader,GNlpv_t longPV)
  {
    BeaconHeader header;
    //ETSI EN 302 636-4-1 [10.3.6]
    //1)Create GN-PDU, steps a) and b) already done in sendGN()
    //c) set the fields of the beacon extended header
    header.SetLongPositionV (longPV);
    dataRequest.data->AddHeader (header);

    dataRequest.data->AddHeader (commonHeader);
    dataRequest.data->AddHeader (basicHeader);
    //!2)Security profile settings not implemented
    //!3)Media-dependent procedures not implemented
    //4) pass the GN-PDU to LL protocol entity
    if(m_socket_tx==NULL)
    {
      NS_LOG_ERROR("GeoNet: SOCKET NOT FOUND ");
      return UNSPECIFIED_ERROR;
    }

    if(m_metric_supervisor_ptr!=nullptr && m_PRRsupervisor_beacons==true)
    {
      uint8_t *buffer = new uint8_t[dataRequest.data->GetSize ()];

      dataRequest.data->CopyData (buffer,dataRequest.data->GetSize ());

      m_metric_supervisor_ptr->signalSentPacket (MetricSupervisor::bufToString (buffer,dataRequest.data->GetSize ()),m_egoPV.POS_EPV.lat,m_egoPV.POS_EPV.lon,m_station_id, MetricSupervisor::messageType_GNbeacon);
      delete[] buffer;
    }

    //if(!m_GnIsMobile)return ACCEPTED;
    if(m_socket_tx->Send (dataRequest.data)==-1)
    {
      NS_LOG_ERROR("Cannot send BEACON packet ");
      return UNSPECIFIED_ERROR;
    }
    m_event_Beacon.Cancel ();
    //5) initialize timer for periodic retransmission of beacons
    double T_beacon = m_GnBeaconServiceRetransmitTimer + (rand()% m_GnBeaconServiceMaxJItter);
    m_event_Beacon = Simulator::Schedule(MilliSeconds(T_beacon),&GeoNet::setBeacon,this);
    return ACCEPTED;
  }

  bool
  GeoNet::isInsideGeoArea (GeoArea_t geoArea)
  {
    //forwarding algorithm selection as specified in  ETSI EN 302 636-4-1 [Annex D]
    //Compute function F specified in ETSI EN 302 931
    double x,y,r,f,geoLon,geoLat;
    VDP::VDP_position_cartesian_t egoPos, geoPos;
    if((m_egoPV.POS_EPV.lat==0)||(m_egoPV.POS_EPV.lon==0))return false;//In case egoPV hasnt updated for the first time yet

    egoPos = m_vdp->getXY(m_egoPV.POS_EPV.lon,m_egoPV.POS_EPV.lat); //Compute cartesian position of the vehicle
    geoLon = ((double) geoArea.posLong)/DOT_ONE_MICRO;
    geoLat = ((double)geoArea.posLat)/DOT_ONE_MICRO;
    geoPos = m_vdp->getXY(geoLon, geoLat); // Compute cartesian position of the geoArea center

    /*(x,y) is the cartesian position relative to the center of the geoArea shape -> if vehicle is indeed in the center of
     * the shape, (x,y)=(0,0)
    */
    x = egoPos.x - geoPos.x;
    y = egoPos.y - geoPos.y;

    if(geoArea.shape == 0)
      {
        //Circular area
        r = geoArea.distA;
        f = 1.0 - pow((x/r),2.0) - pow((y/r),2.0);
      }
    else
      {
        //Rotate (x,y) clockwise around the center of the shape by the zenith angle = 90 deg - azimuth
        double xr,yr,zenith;
        zenith = (90.0 - geoArea.angle)* (M_PI/180.0); // zenith in radians
        xr = x * cos(zenith) + y * sin(zenith);
        yr = -(x * sin(zenith)) + y * cos(zenith);
        if (geoArea.shape == 1)
        {
          //Rectangular area function ETSI EN 302 931
          f = std::min(1.0 - pow(xr/geoArea.distA,2.0), 1.0 - pow(yr/geoArea.distB,2.0));
        }
        else
          {
            //Ellipsoidal area
            f = 1.0 - pow((xr/geoArea.distA),2.0) - pow((yr/geoArea.distB),2.0);
          }
      }

    if(f>=0)
    {
      return true;// Local router inside the geographical target
    }
    else
    {
      return false; // Local router outside the geographical target
    }
  }

  bool
  GeoNet::hasNeighbour ()
  {
    bool retval = false;
    m_LocT_Mutex.lock ();
    std::map<GNAddress,GNLocTE>::iterator locT_it = m_GNLocT.begin ();
    while(locT_it != m_GNLocT.end ())
    {
      if(locT_it->second.IS_NEIGHBOUR)
      {
        retval = true;
        break;
      }
      locT_it++;
    }
    m_LocT_Mutex.unlock ();
    return retval;
  }

  void
  GeoNet::saveRepPacket (GNDataRequest_t dataRequest)
  {
    std::map<GNDataRequest_t, std::pair<Timer,Timer>>::iterator it = m_Repetition_packets.find (dataRequest);
    if(it == m_Repetition_packets.end ())
    {
      //Packet is not already stored
      m_Repetition_packets.emplace(dataRequest,std::pair<Timer,Timer>());
      //b)retransmit packet with a period as specified in the repetition interval parameter  until the maximim time of the packet is expired
      GeoNet::setRepInt(std::get<0>(m_Repetition_packets[dataRequest]),MilliSeconds (dataRequest.GNRepInt),&GeoNet::sendGN,dataRequest);
      GeoNet::setRepInt(std::get<1>(m_Repetition_packets[dataRequest]),MilliSeconds (dataRequest.GNMaxRepTime),&GeoNet::maxRepIntTimeout,dataRequest);
    }
    else
    {
      //Packet is already stored, reset repetition interval timer
      GeoNet::setRepInt(std::get<0>(m_Repetition_packets[dataRequest]),MilliSeconds (dataRequest.GNRepInt),&GeoNet::sendGN,dataRequest);
    }
  }

  template<typename MEM_PTR> void
   GeoNet::setRepInt(Timer &timer,Time delay,MEM_PTR callback_fcn,GNDataRequest_t dataRequest) {
    if(timer.IsRunning ())
    {
       timer.Cancel();
    }

    timer.SetFunction(callback_fcn,this);
    timer.SetArguments(dataRequest);
    timer.SetDelay (delay);

    timer.Schedule ();
  }

  void
  GeoNet::maxRepIntTimeout(GNDataRequest_t dataRequest)
  {
    //When the maximum repetition time expires the saved packet is erased and the repetition interval timer is canceled
    std::get<0>(m_Repetition_packets[dataRequest]).Cancel();
    m_Repetition_packets.erase (dataRequest);
  }

  void
  GeoNet::receiveGN(Ptr<Socket> socket)
  {
    GNDataIndication_t dataIndication = {};
    Address from;
    GNBasicHeader basicHeader;
    GNCommonHeader commonHeader;

    if(m_stationtype==StationType_roadSideUnit && m_RSU_epv_set==false)
    {
      NS_FATAL_ERROR("Error: no position has been set for an RSU object. Please use setFixedPositionRSU() on the Facilities Layer object.");
    }

    dataIndication.data = socket->RecvFrom (from);

    uint8_t *buffer;
    uint32_t dataSize;
    if(m_metric_supervisor_ptr!=nullptr)
    {
      buffer = new uint8_t[dataIndication.data->GetSize ()];

      dataSize = dataIndication.data->GetSize ();

      dataIndication.data->CopyData (buffer,dataSize);
    }

    dataIndication.data->RemoveHeader (basicHeader, 4);
    dataIndication.GNRemainingLife = basicHeader.GetLifeTime ();
    dataIndication.GNRemainingHL = basicHeader.GetRemainingHL ();

    // Basic Header Procesing according to ETSI EN 302 636-4-1 [10.3.3]
    // 1) Check version field
    if(basicHeader.GetVersion() != m_GnPtotocolVersion)
    {
      NS_LOG_ERROR("Incorrect version of GN protocol");
    }
    // 2) Check NH field
    if(basicHeader.GetNextHeader()==2) // a) if NH=0 or NH=1 proceed with common header procesing
    {
      // Secured packet
    }
    dataIndication.GNRemainingLife = decodeLT(basicHeader.GetLifeTime ());

    // Common Header Processing according to ETSI EN 302 636-4-1 [10.3.5]
    dataIndication.data->RemoveHeader (commonHeader, 8);
    dataIndication.upperProtocol = commonHeader.GetNextHeader (); //!Information needed for step 7
    dataIndication.GNTraClass = commonHeader.GetTrafficClass (); //!Information needed for step 7

    // 1) Check MHL field
    if(commonHeader.GetMaxHopLimit() < basicHeader.GetRemainingHL())
    {
      NS_LOG_ERROR("Max hop limit greater than remaining hop limit"); //a) if MHL<RHL discard packet and omit execution of further steps
      return;
    }
    // 2) process the BC forwarding buffer, for now not implemented (SCF in traffic class disabled)
    // 3) check HT field
    dataIndication.GNType = commonHeader.GetHeaderType();
    dataIndication.lenght = commonHeader.GetPayload ();

    if(m_metric_supervisor_ptr!=nullptr)
    {
        if(dataIndication.GNType!=BEACON || m_PRRsupervisor_beacons==true)
        {
          m_metric_supervisor_ptr->updateBytesReceived(dataSize);
            m_metric_supervisor_ptr->signalReceivedPacket(MetricSupervisor::bufToString (buffer,dataSize),m_station_id);
        }

        delete[] buffer;
    }

    switch(dataIndication.GNType)
    {
      case BEACON:
        if(commonHeader.GetHeaderSubType ()==0) processSHB (dataIndication,from);
        break;
      case GBC:
          processGBC (dataIndication,from,commonHeader.GetHeaderSubType ());
        break;
      case TSB:
            if((commonHeader.GetHeaderSubType ()==0)) processSHB (dataIndication,from);
        break;
      default:
        NS_LOG_ERROR("GeoNet packet not supported");
    }
  }

  void
  GeoNet::processGBC (GNDataIndication_t dataIndication,Address from, uint8_t shape)
  {
    // GBC Processing according to ETSI EN 302 636-4-1 [10.3.11.3] 1 and 2 already done in receiveGN method
    GBCheader header;
    dataIndication.data->RemoveHeader (header,56);
    dataIndication.SourcePV = header.GetLongPositionV ();
    dataIndication.GnAddressDest = header.GetGeoArea ();
    dataIndication.GnAddressDest.shape = shape;

    //3)Determine function F as specified in ETSI EN 302 931
    m_LocT_Mutex.lock ();
    std::map<GNAddress, GNLocTE>::iterator entry_map_it = m_GNLocT.find(dataIndication.SourcePV.GnAddress);
    if(entry_map_it != m_GNLocT.end())
    {
      //a)
      if((!isInsideGeoArea (dataIndication.GnAddressDest)) && ((m_GNNonAreaForwardingAlgorithm==0)||(m_GNNonAreaForwardingAlgorithm==1)))
      {
        //execute DPD as specified in A.2
        if(DPD(header.GetSeqNumber (),dataIndication.SourcePV.GnAddress))
        {
          NS_LOG_ERROR("Duplicate received");
          return;
        }
      }
      //b)
      if((isInsideGeoArea (dataIndication.GnAddressDest)) && ((m_GNAreaForwardingAlgorithm==0)||(m_GNAreaForwardingAlgorithm==1)))
      {
        //execute DPD as specified in A.2
        if(DPD(header.GetSeqNumber (),dataIndication.SourcePV.GnAddress))
        {
          NS_LOG_ERROR("Duplicate received");
          return;
        }
      }
    }
    //4) DAD
    if (m_GnLocalAddrCongMethod == 0)
    {
      if(DAD(dataIndication.SourcePV.GnAddress))
      {
         NS_LOG_ERROR("Duplicate address detected");
      }
    }
    //Check for LocTE existence
    if (entry_map_it == m_GNLocT.end())
    {
      //5) If LocTE doesn't exist
      newLocTE (dataIndication.SourcePV);//a) create PV with the SO PV in the extended header
      m_GNLocT[dataIndication.SourcePV.GnAddress].IS_NEIGHBOUR = false;//b) Set the IS_NEIGHBOUR flag to FALSE
      //c) PDR not implemented yet
    }
    else
    {
      //6)If the LocTe exist update LongPV, PDR not implemented yet
      LocTUpdate (dataIndication.SourcePV,entry_map_it);
    }
    m_LocT_Mutex.unlock ();
    //7)Determine function F(x,y) as specified in ETSI EN 302 931
    if(isInsideGeoArea (dataIndication.GnAddressDest))
    {
      //a) Pass the payload to the upper protocol entity
      dataIndication.GNType = GBC;
      dataIndication.lenght = dataIndication.data->GetSize ();
      m_ReceiveCallback(dataIndication,from);
    }
    else
    {
      NS_LOG_INFO("GBC packet discarded because GeoNetworking reported it as out-of-range");
    }
  }

  bool
  GeoNet::DAD(GNAddress address)
  {
    if((address == m_GNAddress) && (address.GetLLAddress ()==m_GNAddress.GetLLAddress ()))
    {
      //ETSI EN 302 636-4-1 [10.2.1.5] : If conflict is detected, request new MID field
      m_GnLocalGnAddr = getGNMac48(m_socket_tx->GetNode ()->GetDevice (0)->GetAddress ());
      m_GNAddress = m_GNAddress.MakeManagedconfiguredAddress (m_GnLocalGnAddr,m_stationtype);
      return true;
    }
    else
    {
      return false;
    }
  }

  bool
  GeoNet::DPD(uint16_t seqNumber,GNAddress address)
  {
    std::set<uint16_t>::iterator it = m_GNLocT[address].DPL.find(seqNumber);
    if(it == m_GNLocT[address].DPL.end ())
    {
      //If entry doesnt exist, the packet is not a duplicate and should be added to the list
      m_GNLocT[address].DPL.insert (seqNumber);
      return false;
    }
    else
    {
      return true;//Packet is a duplicate
    }
  }

  void
  GeoNet::processSHB (GNDataIndication_t dataIndication, Address from)
  {
    SHBheader shbHeader;
    BeaconHeader beaconHeader;
    if(dataIndication.GNType == BEACON)
    {
      dataIndication.data->RemoveHeader (beaconHeader, 24);
      dataIndication.SourcePV = beaconHeader.GetLongPositionV ();
    }
    else
    {
      dataIndication.data->RemoveHeader (shbHeader, 28);
      dataIndication.SourcePV = shbHeader.GetLongPositionV ();
    }
    // SHB Processing according to ETSI EN 302 636-4-1 [10.3.10.3] or Beacon processing according to [10.3.6.3]
    //3)execute DAD
    if(m_GnLocalAddrCongMethod == 0)
    {
      if(DAD(dataIndication.SourcePV.GnAddress))
      {
         NS_LOG_ERROR("Duplicate address detected");
      }
    }
    //4)update PV in the SO LocTE with the SO PV fields of the SHB extended header
    m_LocT_Mutex.lock ();
    std::map<GNAddress, GNLocTE>::iterator entry_map_it = m_GNLocT.find(dataIndication.SourcePV.GnAddress);

    //Not specified in the protocol but first check if LocTE exist in the LocTable
    if (entry_map_it == m_GNLocT.end())
    {
      newLocTE (dataIndication.SourcePV);
    }
    else
    {
      //Update LongPV
      LocTUpdate (dataIndication.SourcePV,entry_map_it);
      //6)Set IS_NEIGHBOUR flag to true
      entry_map_it->second.IS_NEIGHBOUR = true;
    }
    m_LocT_Mutex.unlock ();
    //7) Pass the payload to the upper protocol entity if it's not a beacon packet
    if(dataIndication.GNType != BEACON)
    {
      m_ReceiveCallback(dataIndication,from);
    }
  }

  void
  GeoNet::newLocTE (GNlpv_t lpv)
  {
    //Create new LocT entry according to ETSI EN 302 636-4-1 [8.1.2]
    GNLocTE new_entry;
    new_entry.GN_ADDR = lpv.GnAddress;
    new_entry.LL_ADDR = lpv.GnAddress.GetLLAddress();
    new_entry.version = 1;
    new_entry.lpv = lpv;
    new_entry.LS_PENDING = false;
    new_entry.IS_NEIGHBOUR = true;
    new_entry.DPL.clear ();//!Duplicate packet list
    new_entry.timestamp = compute_timestampIts (true);
    new_entry.PDR = 0; //!Packet data rate, yet to be implemented

    //Before storing the new entry, start the T(LocTE) as specified in [8.1.3]
    m_GNLocTTimer.emplace(lpv.GnAddress, Timer());
    GeoNet::setTLocT(m_GNLocTTimer[lpv.GnAddress],Seconds(m_GnLifeTimeLocTE),&GeoNet::LocTE_timeout,lpv.GnAddress);
    //!LS_PENDING timer [8.1.3] not implemented yet

    //Store new entry
    m_GNLocT.emplace(lpv.GnAddress,new_entry);
  }

  template<typename MEM_PTR> void
  GeoNet::setTLocT(Timer &timer,Time delay,MEM_PTR callback_fcn,GNAddress address)
  {
    if(timer.IsRunning ())
    {
       timer.Cancel();
    }

    timer.SetFunction(callback_fcn,this);
    timer.SetArguments(address);
    timer.SetDelay (delay);

    timer.Schedule ();
  }

  void
  GeoNet::LocTUpdate (GNlpv_t lpv, std::map<GNAddress,GNLocTE>::iterator locte_it)
  {
    //Update LongPV as especified in clause C.2
    long TSTpv_rp =  lpv.TST; //! Timestamp for the position vector in the received GN packet
    long TSTpv_locT = locte_it->second.timestamp; //! Timestamp for the position vector in the LocT to be updated
    if(((TSTpv_rp>TSTpv_locT) && ((TSTpv_rp-TSTpv_locT)<=TS_MAX/2)) ||
       ((TSTpv_locT>TSTpv_rp) && ((TSTpv_locT-TSTpv_rp)>TS_MAX/2)))
    {
      //TSTpv_rp greater than TSTpv_locT
      //Cancel T(LocTE) from previous entry even if this is done by setTLocT
      m_GNLocTTimer[lpv.GnAddress].Cancel();
      //Before updating entry, start the T(LocTE) as specified in [8.1.3]
      m_GNLocTTimer.emplace(lpv.GnAddress, Timer ());
      GeoNet::setTLocT(m_GNLocTTimer[lpv.GnAddress],Seconds(m_GnLifeTimeLocTE),&GeoNet::LocTE_timeout,lpv.GnAddress);
      //PVlocT <- PVrp
      locte_it->second.lpv = lpv;
    }
    else
    {
      NS_LOG_ERROR("Timestamp of received packet is not greater than the one from LocT !");
    }
  }

  void
  GeoNet::LocTE_timeout (GNAddress entry_address)
  {
      m_LocT_Mutex.lock ();
      m_GNLocTTimer.erase(entry_address);
      m_GNLocT.erase(entry_address);
      m_LocT_Mutex.unlock ();
  }

  Ptr<Socket>
  GeoNet::createGNPacketSocket(Ptr<Node> node_ptr)
  {
      Ptr<Socket> gn_sock;
      TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");

      if(node_ptr==nullptr)
      {
        NS_FATAL_ERROR ("Error. Called createGNPacketSocket() with a null Ptr<Node>!");
      }

      gn_sock = Socket::CreateSocket (node_ptr, tid);
      PacketSocketAddress local = getGNAddress(node_ptr->GetDevice(0)->GetIfIndex (), node_ptr->GetDevice(0)->GetAddress());
      if (gn_sock->Bind (local) == -1)
      {
        NS_FATAL_ERROR ("Failed to create GeoNet Packet Socket. Cannot Bind().");
      }
      PacketSocketAddress remote = getGNAddress(node_ptr->GetDevice(0)->GetIfIndex(),node_ptr->GetDevice(0)->GetBroadcast());
      gn_sock->Connect (remote);

      return gn_sock;
  }

  int
  GeoNet::get_messageID_from_BTP_port(int16_t port) {
      switch(port) {
          case 2001:
                  return 2;
          case 2002:
                  return 1;
          case 2003:
                  return 5;
          case 2004:
                  return 4;
          case 2005:
                  return 12;
          case 2006:
                  return 6;
          case 2007:
                  return 9;
          case 2008:
                  return 10;
          case 2009:
                  return 14;
          case 2010:
                  return 11;
          case 2012:
                  return 7;
          case 2013:
                  return 13;
          case 2017:
                  return 20;
          case 2018:
                  return 16;
          case 2019:
                  return 15;
          case 2200:
                  return 200;
          default:
                  return 0;
      }
      return 0;
  }
}
