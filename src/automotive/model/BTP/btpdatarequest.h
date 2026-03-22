#ifndef BTPDATAREQUEST_H
#define BTPDATAREQUEST_H
#include <stdint.h>
#include "ns3/packet.h"
#include "ns3/gn-address.h"
#include "ns3/longpositionvector.h"


#define CIRCULAR 0
#define RECTANGULAR 1
#define ELLIPSOIDAL 2

namespace ns3 {

  typedef enum {
    ANY_TT=0,
    BEACON=1,
    GUC=2,
    GAC=3,
    GBC=4,
    TSB=5,
    LS=6,
  } TransportType_t;

  typedef enum {
    UNSPECIFIED=0,
    ITS_G5=1
  } CommProfile_t;

  typedef enum {
    ANY_UP=0,
    BTP_A=1,
    BTP_B=2,
    GN6ASL=3
  } BTPType_t;

  typedef struct _geoarea {
    int32_t posLong;
    int32_t posLat;
    uint16_t distA;
    uint16_t distB;
    uint16_t angle;
    uint8_t shape;
  }GeoArea_t;

  typedef enum {
    ACCEPTED=0,
    MAX_LENGHT_EXCEEDED = 1,
    MAX_LIFE_EXCEEDED = 2,
    REP_INTERVAL_LOW = 3,
    UNSUPPORTED_TRA_CLASS = 4,
    MAX_GEOAREA_EXCEEDED = 5,
    UNSPECIFIED_ERROR =6,
    BLOCKED_BY_GK = 7,
    CONFIRM_UNKNOWN = 8,
  }GNDataConfirm_t;

  typedef struct _btpdataRequest {
    BTPType_t BTPType;
    int16_t destPort;
    int16_t sourcePort;
    int16_t destPInfo;

    TransportType_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    GeoArea_t GnAddress; // GN destination address -- destination address(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
    CommProfile_t GNCommProfile; // GN Communication Profile -- determines de LL protocol entity

    int16_t GNSecurityP; // GN Security Profile /OPTIONAL/
    double GNMaxLife; //GN Maximum Packet Lifetime in [s] /OPTIONAL/
    int16_t GNRepInt; // GN Repetition Interval /OPTIONAL/
    int16_t GNMaxRepInt; // GN maximum repetition Interval /OPTIONAL/
    int16_t GNMaxHL; // GN Max Hop Limit /OPTIONAL/
    uint8_t GNTraClass; // GN Traffic Class

    uint32_t lenght; // Payload size
    Ptr<Packet> data; // Payload

  } BTPDataRequest_t;

  typedef struct _dataIndication {
    uint8_t BTPType;
    int16_t destPort;
    int16_t sourcePort;
    int16_t destPInfo;

    uint8_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    GeoArea_t GnAddress; // GN destination adress -- destination adress(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
    GNlpv_t GNPositionV; // GN Posistion vector

    int16_t GNSecurityR; // GN Security Report /OPCIONAL/
    int16_t GNCertID; //GN Certificate ID /OPCIONAL/
    int16_t GNPermissions; // GN Permissions /OPCIONAL/
    int16_t GNMaxRepInt; // GN maximum repetition Interval /OPCIONAL/
    uint8_t GNTraClass; // GN Traffic Class
    double GNRemPLife; // GN Reamianing Packet Lifetime /OPCIONAL/

    GNAddress GNAddressSource;

    uint32_t lenght;
    Ptr<Packet> data;
  } BTPDataIndication_t;

  typedef struct _gndataRequest {
    BTPType_t upperProtocol;
    TransportType_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    GeoArea_t GnAddress; // GN destination adress -- destination address(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast)
    CommProfile_t GNCommProfile; // GN Communication Profile -- determines de LL protocol entity

    int16_t GNSecurityP; // GN Security Profile /OPCIONAL/
    int32_t GNITS_AIDL;//Length of the value provided in the ITS-AID parameter /OPCIONAL/
    int32_t GNITS_AID;// ITS-AID for the payload to be sent/OPCIONAL/
    int32_t GNSecurityPermL;// Length of the security permissions parameter/OPCIONAL/
    int32_t GNSecurityPerm;// SSP associated with the ITS-AID/OPCIONAL/
    int32_t GNSecurityContInfo;// Information to be used to selecting properties of the security protocol/OPCIONAL/
    int32_t GNSecurityTargetIDListL; //Length for the value in the security target id list parameter /OPCIONAL/
    int32_t GNSecurityTargetIDList;// Unordered collection of target IDs used by the security entity for specifying multiple recipients /OPCIONAL/
    double GNMaxLife; //GN Maximum Packet Lifetime in [s] /OPCIONAL/
    int16_t GNRepInt; // GN Repetition Interval /OPCIONAL/
    int16_t GNMaxRepTime; // GN maximum repetition time /OPCIONAL/
    int16_t GNMaxHL; // GN Max Hop Limit /OPCIONAL/
    uint8_t GNTraClass; // GN Traffic Class

    uint32_t lenght; // Payload size
    Ptr<Packet> data; // Payload

    int16_t _messagePort; //BTP port for PRR supervisor (used for internal computation, not specified in the standard)

    bool operator <( const _gndataRequest &rhs ) const
        {
            return ( data < rhs.data );
        }
  } GNDataRequest_t;

  typedef struct _gndataIndication {
    uint8_t upperProtocol;
    uint8_t GNType; // GN Packet transport type -- GeoUnicast, SHB, TSB, GeoBroadcast or GeoAnycast
    GeoArea_t GnAddressDest; // GN destination adress -- destination adress(GeoUnicast) or geo. area (GeoBroadcast or GeoAnycast) with which the packet was generated with
    GNlpv_t SourcePV;
    //Security report /OPCIONAL/
    //Certificate ID/OPCIONAL/
    int32_t GNITS_AIDL;//Length of the value provided in the ITS-AID parameter /OPCIONAL/
    int32_t GNITS_AID;// ITS-AID for the payload to be sent/OPCIONAL/
    int32_t GNSecurityPermL;// Length of the security permissions parameter/OPCIONAL/
    int32_t GNSecurityPerm;// SSP associated with the ITS-AID/OPCIONAL/
    int32_t GNSecurityContInfo;// Information to be used to selecting properties of the security protocol/OPCIONAL/

    uint8_t GNTraClass; // GN Traffic Class
    double GNRemainingLife; //GN Remaining Packet Lifetime in [s] /OPCIONAL/
    int16_t GNRemainingHL; // GN Remaining Hop Limit /OPCIONAL/

    uint32_t lenght; // Payload size
    Ptr<Packet> data; // Payload
  } GNDataIndication_t;
}

#endif // BTPDATAREQUEST_H
