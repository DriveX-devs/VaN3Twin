/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef NGV_PPDU_H
#define NGV_PPDU_H

#include "ns3/wifi-phy-band.h"
#include "ns3/wifi-ppdu.h"

namespace ns3 {

/**
 * \brief NGV PPDU (IEEE 802.11bd Clause 32).
 * \ingroup wifi
 *
 * This PPDU stores the full 802.11bd TXVECTOR metadata instead of
 * reconstructing it from the legacy L-SIG field.
 */
class NgvPpdu : public WifiPpdu
{
public:
  NgvPpdu (Ptr<const WifiPsdu> psdu,
           const WifiTxVector& txVector,
           WifiPhyBand band,
           uint64_t uid,
           Time txDuration);
  ~NgvPpdu () override;

  Time GetTxDuration (void) const override;
  Ptr<WifiPpdu> Copy (void) const override;

private:
  WifiTxVector DoGetTxVector (void) const override;

  WifiTxVector m_txVector; //!< TXVECTOR used for the NGV PPDU
  WifiPhyBand m_band;      //!< band used by this PPDU
  Time m_txDuration;       //!< cached NGV TX duration
};

} // namespace ns3

#endif /* NGV_PPDU_H */
