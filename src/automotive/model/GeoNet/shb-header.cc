#include "shb-header.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE ("SHBheader");

  NS_OBJECT_ENSURE_REGISTERED (SHBheader);

  SHBheader::SHBheader()
  {
    NS_LOG_FUNCTION (this);
  }

  SHBheader::~SHBheader()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  SHBheader::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::SHBheader")
      .SetParent<Header> ()
      .SetGroupName ("Automotive")
      .AddConstructor<SHBheader> ();
    return tid;
  }

  TypeId
  SHBheader::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  SHBheader::GetSerializedSize (void) const
  {
    return 28;
  }

  void
  SHBheader::Print (std::ostream &os) const
  {
    os << m_sourcePV.GnAddress;
  }

  void
  SHBheader::Serialize (Buffer::Iterator start) const
  {
    //ETSI EN 302 636-4-1 [9.8.4]
    Buffer::Iterator i = start;
    //Source long position vector aux varaibles
    uint16_t pai_speed = 0;
    pai_speed = (m_sourcePV.positionAccuracy << 15) | (((m_sourcePV.speed>>1)*2)&0x7FFF); //Speed >> 1 so that sign isnt lost but multiplied by 2 to compensate bit shift

    //Source long position vector
    WriteTo (i,m_sourcePV.GnAddress.ConvertTo ());
    i.WriteHtonU32 (m_sourcePV.TST);
    i.WriteHtonU32 (m_sourcePV.latitude);
    i.WriteHtonU32 (m_sourcePV.longitude);
    i.WriteHtonU16 (pai_speed);
    i.WriteHtonU16 (m_sourcePV.heading);
    if (m_dcc == nullptr)
      {
        //Reserved
        i.WriteHtonU32 (0x00000000);
      }
    else
      {
        // ---- DCC/MCO block ----
        double cbr0 = m_dcc->getCBRR0();
        if (cbr0 < 0.0f) cbr0 = 0.0f;
        if (cbr0 > 1.0f) cbr0 = 1.0f;
        uint8_t cbr0_enc = static_cast<uint8_t>(std::floor(cbr0 * 255.0f));
        double cbr1 = m_dcc->getCBRR1();
        uint8_t cbr1_enc = static_cast<uint8_t>(std::floor(cbr1 * 255.0f));
        int tp = 0;
        if (m_phy != nullptr)
          {
            tp = std::round(m_phy->GetTxPowerEnd()); // 0..31
            if (tp < 0)  tp = 0;
            if (tp > 31) tp = 31;
          }
        uint8_t txp_byte = static_cast<uint8_t>(tp & 0x1F); // bits 0–4
        uint8_t reserved = 0;
        i.WriteU8(cbr0_enc);  // Octet 40
        i.WriteU8(cbr1_enc);  // Octet 41
        i.WriteU8(txp_byte);  // Octet 42
        i.WriteU8(reserved);  // Octet 43
      }
  }

  uint32_t
  SHBheader::Deserialize (Buffer::Iterator start)
  {
    Buffer::Iterator i = start;

    //Source long position vector
    uint8_t addr[8];
    i.Read (addr,8);
    m_sourcePV.GnAddress.Set (addr);
    m_sourcePV.TST = i.ReadNtohU32 ();
    m_sourcePV.latitude = i.ReadNtohU32 ();
    m_sourcePV.longitude = i.ReadNtohU32 ();
    uint16_t pai_speed = 0;
    pai_speed = i.ReadU16 ();
    m_sourcePV.positionAccuracy = pai_speed >> 15;
    m_sourcePV.speed = pai_speed & 0x7fff;
    m_sourcePV.heading = i.ReadNtohU16 ();

    // ---- DCC/MCO block ----
    uint8_t cbr0_enc = i.ReadU8();  // Octet 40
    uint8_t cbr1_enc = i.ReadU8();  // Octet 41
    uint8_t txp_byte = i.ReadU8();  // Octet 42
    uint8_t reserved = i.ReadU8();  // Octet 43

    // Decode fields
    double cbr0 = static_cast<double>(cbr0_enc) / 255.0f;
    m_CBR_R0_Hop = cbr0;
    double cbr1 = static_cast<double>(cbr1_enc) / 255.0f;
    m_CBR_R1_Hop = cbr1;
    // Tx power in bits 0–4
    float txPower = static_cast<uint8_t>(txp_byte & 0x1F);

    //Reserved
    // m_reserved32 = i.ReadNtohU32 ();
    return GetSerializedSize ();
  }

}


