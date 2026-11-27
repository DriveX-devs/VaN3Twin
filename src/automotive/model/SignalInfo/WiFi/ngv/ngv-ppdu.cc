/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ngv-ppdu.h"

#include "ns3/log.h"
#include "ns3/wifi-psdu.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NgvPpdu");

NgvPpdu::NgvPpdu (Ptr<const WifiPsdu> psdu,
                  const WifiTxVector& txVector,
                  WifiPhyBand band,
                  uint64_t uid,
                  Time txDuration)
  : WifiPpdu (psdu, txVector, uid),
    m_txVector (txVector),
    m_band (band),
    m_txDuration (txDuration)
{
  NS_LOG_FUNCTION (this << psdu << txVector << band << uid << txDuration);
}

NgvPpdu::~NgvPpdu ()
{
}

WifiTxVector
NgvPpdu::DoGetTxVector (void) const
{
  return m_txVector;
}

Time
NgvPpdu::GetTxDuration (void) const
{
  return m_txDuration;
}

Ptr<WifiPpdu>
NgvPpdu::Copy (void) const
{
  return Create<NgvPpdu> (GetPsdu (), GetTxVector (), m_band, m_uid, m_txDuration);
}

} // namespace ns3
