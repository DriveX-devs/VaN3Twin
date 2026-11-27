/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef NGV_PHY_H
#define NGV_PHY_H

#include "ns3/ofdm-phy.h"

namespace ns3 {

/**
 * \brief PHY entity for 802.11bd NGV PPDUs.
 * \ingroup wifi
 *
 * This first NGV entity models the 10 MHz, NSS=1 PPDU timing path used by
 * VaN3Twin's vehicular 802.11bd profile.
 */
class NgvPhy : public OfdmPhy
{
public:
  NgvPhy ();
  ~NgvPhy () override;

  WifiMode GetSigMode (WifiPpduField field, const WifiTxVector& txVector) const override;
  const PpduFormats& GetPpduFormats (void) const override;
  Time GetDuration (WifiPpduField field, const WifiTxVector& txVector) const override;
  Time GetPayloadDuration (uint32_t size, const WifiTxVector& txVector, WifiPhyBand band, MpduType mpdutype,
                           bool incFlag, uint32_t& totalAmpduSize, double& totalAmpduNumSymbols,
                           uint16_t staId) const override;
  Ptr<WifiPpdu> BuildPpdu (const WifiConstPsduMap& psdus, const WifiTxVector& txVector, Time ppduDuration) override;

  /**
   * Return the number of coded Data bits per NGV OFDM symbol for the current
   * 10 MHz / NSS=1 implementation subset.
   *
   * \param txVector the NGV TXVECTOR
   * \return NGV Data bits per symbol
   */
  static uint16_t GetDataBitsPerSymbol (const WifiTxVector& txVector);

  /**
   * Return the NGV Data field rate in bit/s for the current 10 MHz / NSS=1
   * implementation subset.
   *
   * \param txVector the NGV TXVECTOR
   * \return NGV Data rate in bit/s
   */
  static uint64_t GetDataRate (const WifiTxVector& txVector);

protected:
  PhyFieldRxStatus DoEndReceiveField (WifiPpduField field, Ptr<Event> event) override;
  bool IsConfigSupported (Ptr<const WifiPpdu> ppdu) const override;

private:
  static Time GetNgvLtfSymbolDuration (const WifiTxVector& txVector);
  static uint16_t GetNgvLtfSymbolCount (const WifiTxVector& txVector);
  static uint16_t GetMidamblePeriodicity (const WifiTxVector& txVector);
  static uint32_t GetNgvDataSymbolCount (uint32_t size, const WifiTxVector& txVector);

  static const PpduFormats m_ngvPpduFormats; //!< NGV PPDU format
};

} // namespace ns3

#endif /* NGV_PHY_H */
