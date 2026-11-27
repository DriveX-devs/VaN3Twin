/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ngv-phy.h"
#include "ngv-ppdu.h"

#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/interference-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-psdu.h"

#include <array>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NgvPhy");

namespace {

struct NgvMcsParams
{
  uint8_t index;
  uint16_t channelWidth;
  uint8_t nss;
  uint16_t dataBitsPerSymbol;
};

// Keep in sync with VehicularNgvMcs for the supported 10/20 MHz profile slices.
constexpr std::array<NgvMcsParams, 40> g_ngvMcs = {{
    {0, 10, 1, 26},
    {1, 10, 1, 52},
    {2, 10, 1, 78},
    {3, 10, 1, 104},
    {4, 10, 1, 156},
    {5, 10, 1, 208},
    {6, 10, 1, 234},
    {7, 10, 1, 260},
    {8, 10, 1, 312},
    {15, 10, 1, 13},
    {0, 10, 2, 52},
    {1, 10, 2, 104},
    {2, 10, 2, 156},
    {3, 10, 2, 208},
    {4, 10, 2, 312},
    {5, 10, 2, 416},
    {6, 10, 2, 468},
    {7, 10, 2, 520},
    {8, 10, 2, 624},
    {0, 20, 1, 54},
    {1, 20, 1, 108},
    {2, 20, 1, 162},
    {3, 20, 1, 216},
    {4, 20, 1, 324},
    {5, 20, 1, 432},
    {6, 20, 1, 486},
    {7, 20, 1, 540},
    {8, 20, 1, 648},
    {9, 20, 1, 720},
    {15, 20, 1, 27},
    {0, 20, 2, 108},
    {1, 20, 2, 216},
    {2, 20, 2, 324},
    {3, 20, 2, 432},
    {4, 20, 2, 648},
    {5, 20, 2, 864},
    {6, 20, 2, 972},
    {7, 20, 2, 1080},
    {8, 20, 2, 1296},
    {9, 20, 2, 1440},
}};

constexpr uint64_t NGV_SYMBOL_NS = 8000;
constexpr uint64_t NGV_LTF_1X_NS = 4800;
constexpr uint64_t NGV_LTF_2X_NS = 8000;
constexpr uint64_t NGV_LTF_2X_REPEAT_NS = 14400;
constexpr uint16_t NGV_SERVICE_BITS = 16;
constexpr uint16_t NGV_TAIL_BITS = 6;

} // namespace

const PhyEntity::PpduFormats NgvPhy::m_ngvPpduFormats {
  { WIFI_PREAMBLE_LONG, { WIFI_PPDU_FIELD_PREAMBLE,      // L-STF + L-LTF
                          WIFI_PPDU_FIELD_NON_HT_HEADER, // L-SIG + RL-SIG
                          WIFI_PPDU_FIELD_SIG_A,         // NGV-SIG + RNGV-SIG
                          WIFI_PPDU_FIELD_TRAINING,      // NGV-STF + NGV-LTF
                          WIFI_PPDU_FIELD_DATA } }
};

NgvPhy::NgvPhy ()
  : OfdmPhy (OFDM_PHY_10_MHZ, false)
{
  NS_LOG_FUNCTION (this);
}

NgvPhy::~NgvPhy ()
{
  NS_LOG_FUNCTION (this);
}

WifiMode
NgvPhy::GetSigMode (WifiPpduField field, const WifiTxVector& txVector) const
{
  switch (field)
    {
      case WIFI_PPDU_FIELD_PREAMBLE:
      case WIFI_PPDU_FIELD_NON_HT_HEADER:
      case WIFI_PPDU_FIELD_SIG_A:
      case WIFI_PPDU_FIELD_TRAINING:
        return GetHeaderMode (txVector);
      default:
        return OfdmPhy::GetSigMode (field, txVector);
    }
}

const PhyEntity::PpduFormats&
NgvPhy::GetPpduFormats (void) const
{
  return m_ngvPpduFormats;
}

Time
NgvPhy::GetDuration (WifiPpduField field, const WifiTxVector& txVector) const
{
  if (!txVector.IsNgv ())
    {
      return OfdmPhy::GetDuration (field, txVector);
    }

  switch (field)
    {
      case WIFI_PPDU_FIELD_PREAMBLE:
        return MicroSeconds (32); // L-STF + L-LTF for CBW10
      case WIFI_PPDU_FIELD_NON_HT_HEADER:
        return MicroSeconds (16); // L-SIG + RL-SIG
      case WIFI_PPDU_FIELD_SIG_A:
        return MicroSeconds (16); // NGV-SIG + RNGV-SIG
      case WIFI_PPDU_FIELD_TRAINING:
        return MicroSeconds (8) + GetNgvLtfSymbolDuration (txVector) * GetNgvLtfSymbolCount (txVector);
      case WIFI_PPDU_FIELD_HT_SIG:
      case WIFI_PPDU_FIELD_SIG_B:
        return NanoSeconds (0);
      default:
        return OfdmPhy::GetDuration (field, txVector);
    }
}

Time
NgvPhy::GetPayloadDuration (uint32_t size, const WifiTxVector& txVector, WifiPhyBand /* band */,
                            MpduType /* mpdutype */, bool /* incFlag */, uint32_t& /* totalAmpduSize */,
                            double& /* totalAmpduNumSymbols */, uint16_t /* staId */) const
{
  NS_ABORT_MSG_IF (!txVector.IsNgv (), "NgvPhy can only calculate NGV payload duration");

  const uint32_t numSymbols = GetNgvDataSymbolCount (size, txVector);
  const uint16_t midamblePeriodicity = GetMidamblePeriodicity (txVector);
  const uint32_t numMidambles = (midamblePeriodicity == 0 || numSymbols == 0)
                                    ? 0
                                    : (numSymbols - 1) / midamblePeriodicity;
  const Time midambleDuration = GetNgvLtfSymbolDuration (txVector) * GetNgvLtfSymbolCount (txVector);

  return NanoSeconds (numSymbols * NGV_SYMBOL_NS) + numMidambles * midambleDuration;
}

uint16_t
NgvPhy::GetDataBitsPerSymbol (const WifiTxVector& txVector)
{
  const uint16_t channelWidth = txVector.GetChannelWidth ();
  const uint8_t nss = txVector.GetNss ();
  for (const auto& mcs : g_ngvMcs)
    {
      if (mcs.index == txVector.GetNgvMcs () && mcs.channelWidth == channelWidth && mcs.nss == nss)
        {
          return mcs.dataBitsPerSymbol;
        }
    }
  NS_ABORT_MSG ("Invalid or reserved NGV-MCS index for " << channelWidth << " MHz, NSS=" << +nss);
  return 0;
}

uint64_t
NgvPhy::GetDataRate (const WifiTxVector& txVector)
{
  return static_cast<uint64_t> (GetDataBitsPerSymbol (txVector) * (1000000000ull / NGV_SYMBOL_NS));
}

Ptr<WifiPpdu>
NgvPhy::BuildPpdu (const WifiConstPsduMap& psdus, const WifiTxVector& txVector, Time ppduDuration)
{
  NS_LOG_FUNCTION (this << psdus << txVector << ppduDuration);
  NS_ABORT_MSG_IF (!txVector.IsNgv (), "NgvPhy can only build NGV PPDUs");
  NS_ABORT_MSG_IF (psdus.size () != 1, "NGV MU PPDUs are not implemented");
  return Create<NgvPpdu> (psdus.begin ()->second,
                          txVector,
                          m_wifiPhy->GetPhyBand (),
                          ObtainNextUid (txVector),
                          ppduDuration);
}

PhyEntity::PhyFieldRxStatus
NgvPhy::DoEndReceiveField (WifiPpduField field, Ptr<Event> event)
{
  NS_LOG_FUNCTION (this << field << *event);
  switch (field)
    {
      case WIFI_PPDU_FIELD_PREAMBLE:
        return OfdmPhy::DoEndReceiveField (field, event);
      case WIFI_PPDU_FIELD_NON_HT_HEADER:
      case WIFI_PPDU_FIELD_SIG_A:
      case WIFI_PPDU_FIELD_TRAINING:
        {
          SnrPer snrPer = GetPhyHeaderSnrPer (field, event);
          PhyFieldRxStatus status (GetRandomValue () > snrPer.per);
          if (status.isSuccess)
            {
              if (!IsAllConfigSupported (field, event->GetPpdu ()))
                {
                  status = PhyFieldRxStatus (false, UNSUPPORTED_SETTINGS, DROP);
                }
            }
          else
            {
              status.reason = (field == WIFI_PPDU_FIELD_NON_HT_HEADER) ? L_SIG_FAILURE : SIG_A_FAILURE;
              status.actionIfFailure = ABORT;
            }
          return status;
        }
      default:
        return OfdmPhy::DoEndReceiveField (field, event);
    }
}

bool
NgvPhy::IsConfigSupported (Ptr<const WifiPpdu> ppdu) const
{
  const WifiTxVector txVector = ppdu->GetTxVector ();
  return txVector.IsNgv () && txVector.IsValid () && txVector.GetModulationClass () == WIFI_MOD_CLASS_NGV;
}

Time
NgvPhy::GetNgvLtfSymbolDuration (const WifiTxVector& txVector)
{
  switch (txVector.GetNgvLtfType ())
    {
      case WifiNgvLtfType::NGV_LTF_1X:
        return NanoSeconds (NGV_LTF_1X_NS);
      case WifiNgvLtfType::NGV_LTF_2X:
        return NanoSeconds (NGV_LTF_2X_NS);
      case WifiNgvLtfType::NGV_LTF_2X_REPEAT:
        return NanoSeconds (NGV_LTF_2X_REPEAT_NS);
      default:
        NS_ABORT_MSG ("Unknown NGV-LTF type");
    }
  return NanoSeconds (NGV_LTF_2X_NS);
}

uint16_t
NgvPhy::GetNgvLtfSymbolCount (const WifiTxVector& txVector)
{
  NS_ABORT_MSG_IF (txVector.GetNss () == 0 || txVector.GetNss () > 2,
                   "Only NGV NSS=1 and NSS=2 are implemented");
  return txVector.GetNss ();
}

uint16_t
NgvPhy::GetMidamblePeriodicity (const WifiTxVector& txVector)
{
  const uint16_t periodicity = txVector.GetNgvMidamblePeriodicity ();
  if (periodicity == 4 || periodicity == 8 || periodicity == 16)
    {
      return periodicity;
    }
  return 0;
}

uint32_t
NgvPhy::GetNgvDataSymbolCount (uint32_t size, const WifiTxVector& txVector)
{
  const uint16_t dataBitsPerSymbol = GetDataBitsPerSymbol (txVector);
  const double codedBits = NGV_SERVICE_BITS + size * 8.0 + NGV_TAIL_BITS;
  return static_cast<uint32_t> (std::ceil (codedBits / dataBitsPerSymbol));
}

static class NgvPhyConstructor
{
public:
  NgvPhyConstructor ()
  {
    WifiPhy::AddStaticPhyEntity (WIFI_MOD_CLASS_NGV, Create<NgvPhy> ());
  }
} g_ngvPhyConstructor;

} // namespace ns3
