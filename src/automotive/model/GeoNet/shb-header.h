#ifndef SHBHEADER_H
#define SHBHEADER_H

#include <stdint.h>
#include <string>
#include "ns3/header.h"
#include "ns3/gn-address.h"
#include "ns3/mac48-address.h"
#include "ns3/geonet.h"
#include "ns3/basic-header.h"
#include "ns3/common-header.h"
#include "ns3/longpositionvector.h"
#include "ns3/wifi-phy.h"
#include "ns3/DCC.h"

namespace ns3
{
  class SHBheader : public Header
  {
    public:
      SHBheader();
      ~SHBheader();
      static TypeId GetTypeId (void);
      virtual TypeId GetInstanceTypeId (void) const;
      virtual void Print (std::ostream &os) const;
      virtual uint32_t GetSerializedSize (void) const;
      virtual void Serialize (Buffer::Iterator start) const;
      virtual uint32_t Deserialize (Buffer::Iterator start);

      //Setters
      void SetLongPositionV(GNlpv_t longPositionVector) {m_sourcePV = longPositionVector;}


      //Getters
      GNlpv_t GetLongPositionV(void) const {return m_sourcePV;}
      double GetCBRR0Hop() {return m_CBR_R0_Hop;};
      double GetCBRR1Hop() {return m_CBR_R1_Hop;};

      void setDCC(Ptr<DCC> dcc) {m_dcc = dcc;};
      void setPhy(Ptr<WifiPhy> phy) {m_phy = phy;};

    private:
      GNlpv_t m_sourcePV;  //!Source long position vector
      uint8_t m_reserved; //! aux variable for reading reserved fields
      uint32_t m_reserved32;
      Ptr<WifiPhy> m_phy = nullptr;
      Ptr<DCC> m_dcc = nullptr;
      double m_CBR_R0_Hop;
      double m_CBR_R1_Hop;

  };
}


#endif // SHBHEADER_H
