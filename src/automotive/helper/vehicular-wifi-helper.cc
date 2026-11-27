#include "vehicular-wifi-helper.h"

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mode.h"
#include "ns3/wifi-remote-station-manager.h"

#include <utility>

namespace ns3
{

VehicularWifiProfile::VehicularWifiProfile (Standard standard,
                                            std::string dataMode,
                                            std::string controlMode,
                                            uint16_t channelWidthMHz,
                                            double txPowerDbm,
                                            double rxSensitivityDbm,
                                            double snrThresholdDb,
                                            double dccBitRate,
                                            VehicularWifiPpduFormat ppduFormat,
                                            uint8_t ngvMcsIndex,
                                            uint8_t ngvSpatialStreams,
                                            uint8_t nonNgvRepetitions)
  : m_standard (standard),
    m_dataMode (std::move (dataMode)),
    m_controlMode (std::move (controlMode)),
    m_channelWidthMHz (channelWidthMHz),
    m_txPowerDbm (txPowerDbm),
    m_rxSensitivityDbm (rxSensitivityDbm),
    m_snrThresholdDb (snrThresholdDb),
    m_dccBitRate (dccBitRate),
    m_ppduFormat (ppduFormat),
    m_ngvMcsIndex (ngvMcsIndex),
    m_ngvSpatialStreams (ngvSpatialStreams),
    m_nonNgvRepetitions (nonNgvRepetitions)
{
}

VehicularWifiProfile
VehicularWifiProfile::Ieee80211p (const std::string& dataMode,
                                  double txPowerDbm,
                                  double rxSensitivityDbm,
                                  double snrThresholdDb)
{
  return VehicularWifiProfile (Standard::IEEE_80211P,
                               dataMode,
                               dataMode,
                               10,
                               txPowerDbm,
                               rxSensitivityDbm,
                               snrThresholdDb,
                               6e6,
                               VehicularWifiPpduFormat::NON_NGV_10,
                               0,
                               1,
                               0);
}

VehicularWifiProfile
VehicularWifiProfile::Ieee80211bd (const std::string& dataMode,
                                   double txPowerDbm,
                                   double rxSensitivityDbm,
                                   double snrThresholdDb,
                                   uint8_t ngvMcsIndex,
                                   uint8_t ngvSpatialStreams,
                                   uint16_t channelWidthMHz)
{
  const auto& mcs = GetVehicularNgvMcs (ngvMcsIndex, channelWidthMHz, ngvSpatialStreams);
  return VehicularWifiProfile (Standard::IEEE_80211BD,
                               dataMode,
                               channelWidthMHz == 20 ? "OfdmRate6Mbps" : "OfdmRate6MbpsBW10MHz",
                               channelWidthMHz,
                               txPowerDbm,
                               rxSensitivityDbm,
                               snrThresholdDb,
                               mcs.dataRateMbps * 1e6,
                               VehicularWifiPpduFormat::NGV,
                               ngvMcsIndex,
                               ngvSpatialStreams,
                               0);
}

VehicularWifiProfile
VehicularWifiProfile::Ieee80211bd20 (const std::string& dataMode,
                                     double txPowerDbm,
                                     double rxSensitivityDbm,
                                     double snrThresholdDb,
                                     uint8_t ngvMcsIndex,
                                     uint8_t ngvSpatialStreams)
{
  return Ieee80211bd (dataMode,
                      txPowerDbm,
                      rxSensitivityDbm,
                      snrThresholdDb,
                      ngvMcsIndex,
                      ngvSpatialStreams,
                      20);
}

VehicularWifiProfile
VehicularWifiProfile::Ieee80211bdNonNgv10 (const std::string& dataMode,
                                           double txPowerDbm,
                                           double rxSensitivityDbm,
                                           double snrThresholdDb,
                                           uint8_t nonNgvRepetitions)
{
  return VehicularWifiProfile (Standard::IEEE_80211BD,
                               dataMode,
                               dataMode,
                               10,
                               txPowerDbm,
                               rxSensitivityDbm,
                               snrThresholdDb,
                               6e6,
                               VehicularWifiPpduFormat::NON_NGV_10,
                               0,
                               1,
                               nonNgvRepetitions);
}

VehicularWifiProfile
VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate (const std::string& dataMode,
                                                    double txPowerDbm,
                                                    double rxSensitivityDbm,
                                                    double snrThresholdDb)
{
  return VehicularWifiProfile (Standard::IEEE_80211BD,
                               dataMode,
                               dataMode,
                               20,
                               txPowerDbm,
                               rxSensitivityDbm,
                               snrThresholdDb,
                               6e6,
                               VehicularWifiPpduFormat::NON_NGV_20_DUPLICATE,
                               0,
                               1,
                               0);
}

void
VehicularWifiProfile::ConfigurePhy (YansWifiPhyHelper& wifiPhy) const
{
  wifiPhy.Set ("ChannelSettings",
               StringValue ("{0, " + std::to_string (m_channelWidthMHz) + ", BAND_5GHZ, 0}"));
  wifiPhy.Set ("RxSensitivity", DoubleValue (m_rxSensitivityDbm));
  wifiPhy.Set ("TxPowerStart", DoubleValue (m_txPowerDbm));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txPowerDbm));
  if (m_standard == Standard::IEEE_80211BD)
    {
      const uint8_t spatialStreams = m_ppduFormat == VehicularWifiPpduFormat::NGV
                                         ? m_ngvSpatialStreams
                                         : 1;
      wifiPhy.Set ("Antennas", UintegerValue (spatialStreams));
      wifiPhy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (spatialStreams));
      wifiPhy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (spatialStreams));
      wifiPhy.Set ("CcaEdThreshold", DoubleValue (-65.0));
    }
  wifiPhy.SetPreambleDetectionModel ("ns3::ThresholdPreambleDetectionModel",
                                     "MinimumRssi", DoubleValue (m_rxSensitivityDbm),
                                     "Threshold", DoubleValue (m_snrThresholdDb));
}

void
VehicularWifiProfile::ConfigureRemoteStationManager (Wifi80211pHelper& wifiHelper) const
{
  wifiHelper.SetStandard (m_standard == Standard::IEEE_80211BD ? WIFI_STANDARD_80211bd
                                                               : WIFI_STANDARD_80211p);
  uint8_t ppduFormatValue = 0;
  if (m_ppduFormat == VehicularWifiPpduFormat::NGV)
    {
      ppduFormatValue = 1;
    }
  else if (m_ppduFormat == VehicularWifiPpduFormat::NON_NGV_20_DUPLICATE)
    {
      ppduFormatValue = 2;
    }
  wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (m_dataMode),
                                      "ControlMode", StringValue (m_controlMode),
                                      "NonUnicastMode", StringValue (m_dataMode),
                                      "NgvPpduFormat", UintegerValue (ppduFormatValue),
                                      "NgvMcs", UintegerValue (m_ngvMcsIndex),
                                      "NgvSpatialStreams", UintegerValue (m_ngvSpatialStreams),
                                      "NgvMidamblePeriodicity", UintegerValue (0),
                                      "NgvLtfType", UintegerValue ((m_ppduFormat == VehicularWifiPpduFormat::NGV &&
                                                                    m_channelWidthMHz == 10 &&
                                                                    m_ngvSpatialStreams == 1 &&
                                                                    m_ngvMcsIndex == 15)
                                                                       ? 2
                                                                       : (m_ppduFormat == VehicularWifiPpduFormat::NGV ? 1 : 0)),
                                      "NgvPpduRepetitions", UintegerValue (m_nonNgvRepetitions));
}

void
VehicularWifiProfile::ConfigureMetricSupervisor (Ptr<MetricSupervisor> metricSupervisor) const
{
  if (metricSupervisor != nullptr)
    {
      metricSupervisor->setChannelTechnology (GetTechnologyName ());
    }
}

VehicularWifiProfile::Standard
VehicularWifiProfile::GetStandard () const
{
  return m_standard;
}

std::string
VehicularWifiProfile::GetTechnologyName () const
{
  return m_standard == Standard::IEEE_80211BD ? "80211bd" : "80211p";
}

std::string
VehicularWifiProfile::GetDataMode () const
{
  return m_dataMode;
}

std::string
VehicularWifiProfile::GetControlMode () const
{
  return m_controlMode;
}

uint16_t
VehicularWifiProfile::GetChannelWidthMHz () const
{
  return m_channelWidthMHz;
}

double
VehicularWifiProfile::GetTxPowerDbm () const
{
  return m_txPowerDbm;
}

double
VehicularWifiProfile::GetRxSensitivityDbm () const
{
  return m_rxSensitivityDbm;
}

double
VehicularWifiProfile::GetSnrThresholdDb () const
{
  return m_snrThresholdDb;
}

double
VehicularWifiProfile::GetDccBitRate () const
{
  return m_dccBitRate;
}

VehicularWifiPpduFormat
VehicularWifiProfile::GetPpduFormat () const
{
  return m_ppduFormat;
}

std::string
VehicularWifiProfile::GetPpduFormatName () const
{
  return VehicularWifiPpduFormatName (m_ppduFormat);
}

uint8_t
VehicularWifiProfile::GetNgvMcsIndex () const
{
  return m_ngvMcsIndex;
}

uint8_t
VehicularWifiProfile::GetNgvSpatialStreams () const
{
  return m_ngvSpatialStreams;
}

uint8_t
VehicularWifiProfile::GetNonNgvRepetitions () const
{
  return m_nonNgvRepetitions;
}

std::string
VehicularWifiFallbackModeName (VehicularWifiFallbackMode mode)
{
  switch (mode)
    {
    case VehicularWifiFallbackMode::NGV_20:
      return "NGV_20";
    case VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE:
      return "NON_NGV_20_DUPLICATE";
    case VehicularWifiFallbackMode::NON_NGV_10:
      return "NON_NGV_10";
    }
  return "UNKNOWN";
}

namespace
{

uint8_t
NormalizeNgvSpatialStreams (uint8_t spatialStreams)
{
  if (spatialStreams < 1)
    {
      return 1;
    }
  if (spatialStreams > 2)
    {
      return 2;
    }
  return spatialStreams;
}

uint8_t
SelectValidNgvMcs (uint8_t requestedMcs, uint16_t channelWidthMHz, uint8_t spatialStreams)
{
  if (IsVehicularNgvMcsValid (requestedMcs, channelWidthMHz, spatialStreams))
    {
      return requestedMcs;
    }

  const int highestCandidate = requestedMcs > 15 ? 15 : requestedMcs;
  for (int candidate = highestCandidate; candidate >= 0; --candidate)
    {
      if (IsVehicularNgvMcsValid (static_cast<uint8_t> (candidate),
                                  channelWidthMHz,
                                  spatialStreams))
        {
          return static_cast<uint8_t> (candidate);
        }
    }

  for (uint8_t candidate = requestedMcs + 1; candidate <= 15; ++candidate)
    {
      if (IsVehicularNgvMcsValid (candidate, channelWidthMHz, spatialStreams))
        {
          return candidate;
        }
    }
  return 0;
}

std::string
Default11bdDataMode (VehicularWifiFallbackMode mode)
{
  return mode == VehicularWifiFallbackMode::NGV_20 ? "OfdmRate24Mbps"
                                                   : "OfdmRate6MbpsBW10MHz";
}

} // namespace

VehicularWifiFallbackMode
VehicularWifiFallbackController::Select (bool legacyReceiverPresent,
                                         bool duplicate20Allowed,
                                         bool forcePrimaryOnly)
{
  if (!legacyReceiverPresent)
    {
      return VehicularWifiFallbackMode::NGV_20;
    }
  if (forcePrimaryOnly || !duplicate20Allowed)
    {
      return VehicularWifiFallbackMode::NON_NGV_10;
    }
  return VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE;
}

VehicularWifiFallbackMode
VehicularWifiFallbackController::SelectForLinkQuality (const LinkQualityPolicy& policy)
{
  if (policy.forcePrimaryOnly)
    {
      return VehicularWifiFallbackMode::NON_NGV_10;
    }

  if (policy.legacyReceiverPresent)
    {
      if (!policy.duplicate20Allowed)
        {
          return VehicularWifiFallbackMode::NON_NGV_10;
        }
      return policy.estimatedSnrDb >= policy.minDuplicate20SnrDb
                 ? VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE
                 : VehicularWifiFallbackMode::NON_NGV_10;
    }

  if (policy.estimatedSnrDb >= policy.minNgv20SnrDb)
    {
      return VehicularWifiFallbackMode::NGV_20;
    }
  if (policy.duplicate20Allowed && policy.estimatedSnrDb >= policy.minDuplicate20SnrDb)
    {
      return VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE;
    }
  return VehicularWifiFallbackMode::NON_NGV_10;
}

void
VehicularWifiFallbackController::ResetObservationState (ObservationState& state)
{
  ResetObservationState (state, LinkQualityPolicy ());
}

void
VehicularWifiFallbackController::ResetObservationState (ObservationState& state,
                                                        const LinkQualityPolicy& initialPolicy)
{
  const double ewmaAlpha = state.ewmaAlpha;
  const uint32_t failureThreshold = state.failureThreshold;

  state = ObservationState ();
  state.policy = initialPolicy;
  state.ewmaSnrDb = initialPolicy.estimatedSnrDb;
  state.ewmaAlpha = ewmaAlpha;
  state.failureThreshold = failureThreshold;
}

VehicularWifiFallbackMode
VehicularWifiFallbackController::UpdateFromObservation (ObservationState& state,
                                                        double observedSnrDb,
                                                        bool deliverySucceeded,
                                                        bool legacyReceiverPresent)
{
  double alpha = state.ewmaAlpha;
  if (alpha < 0.0)
    {
      alpha = 0.0;
    }
  else if (alpha > 1.0)
    {
      alpha = 1.0;
    }

  if (!state.initialized)
    {
      state.ewmaSnrDb = observedSnrDb;
      state.initialized = true;
    }
  else
    {
      state.ewmaSnrDb = alpha * observedSnrDb + (1.0 - alpha) * state.ewmaSnrDb;
    }

  state.sampleCount++;
  if (deliverySucceeded)
    {
      state.consecutiveFailures = 0;
    }
  else
    {
      state.consecutiveFailures++;
    }

  state.policy.estimatedSnrDb = state.ewmaSnrDb;
  state.policy.legacyReceiverPresent = legacyReceiverPresent;

  LinkQualityPolicy decisionPolicy = state.policy;
  if (state.failureThreshold > 0 && state.consecutiveFailures >= state.failureThreshold)
    {
      decisionPolicy.forcePrimaryOnly = true;
    }

  return SelectForLinkQuality (decisionPolicy);
}

double
VehicularWifiFallbackController::EstimateSnrDb (double signalDbm, double noiseDbm)
{
  return signalDbm - noiseDbm;
}

VehicularWifiFallbackMode
VehicularWifiFallbackController::UpdateFromSignalNoise (ObservationState& state,
                                                        double signalDbm,
                                                        double noiseDbm,
                                                        bool deliverySucceeded,
                                                        bool legacyReceiverPresent)
{
  return UpdateFromObservation (state,
                                EstimateSnrDb (signalDbm, noiseDbm),
                                deliverySucceeded,
                                legacyReceiverPresent);
}

VehicularWifiProfile
VehicularWifiFallbackController::MakeProfile (VehicularWifiFallbackMode mode,
                                              uint8_t ngvMcsIndex,
                                              uint8_t ngvSpatialStreams)
{
  switch (mode)
    {
    case VehicularWifiFallbackMode::NGV_20:
      return VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps",
                                                  23.0,
                                                  -92.0,
                                                  4.0,
                                                  ngvMcsIndex,
                                                  ngvSpatialStreams);
    case VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE:
      return VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate ();
    case VehicularWifiFallbackMode::NON_NGV_10:
      return VehicularWifiProfile::Ieee80211bdNonNgv10 ();
    }
  return VehicularWifiProfile::Ieee80211bd20 ();
}

void
VehicularWifiFallbackController::ConfigureRemoteStationManager (Ptr<WifiRemoteStationManager> manager,
                                                                VehicularWifiFallbackMode mode,
                                                                uint8_t ngvMcsIndex,
                                                                uint8_t ngvSpatialStreams)
{
  if (manager == nullptr)
    {
      return;
    }

  manager->SetAttribute ("NgvMidamblePeriodicity", UintegerValue (0));
  manager->SetAttribute ("NgvPpduRepetitions", UintegerValue (0));

  switch (mode)
    {
    case VehicularWifiFallbackMode::NGV_20:
      manager->SetAttribute ("NgvPpduFormat", UintegerValue (1));
      manager->SetAttribute ("NgvMcs", UintegerValue (ngvMcsIndex));
      manager->SetAttribute ("NgvSpatialStreams", UintegerValue (ngvSpatialStreams));
      manager->SetAttribute ("NgvLtfType", UintegerValue (1));
      manager->SetAttribute ("DataMode", WifiModeValue (WifiMode ("OfdmRate24Mbps")));
      manager->SetAttribute ("ControlMode", WifiModeValue (WifiMode ("OfdmRate6Mbps")));
      manager->SetAttribute ("NonUnicastMode", WifiModeValue (WifiMode ("OfdmRate24Mbps")));
      break;
    case VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE:
      manager->SetAttribute ("NgvPpduFormat", UintegerValue (2));
      manager->SetAttribute ("NgvMcs", UintegerValue (0));
      manager->SetAttribute ("NgvSpatialStreams", UintegerValue (1));
      manager->SetAttribute ("NgvLtfType", UintegerValue (0));
      manager->SetAttribute ("DataMode", WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      manager->SetAttribute ("ControlMode", WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      manager->SetAttribute ("NonUnicastMode",
                             WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      break;
    case VehicularWifiFallbackMode::NON_NGV_10:
      manager->SetAttribute ("NgvPpduFormat", UintegerValue (0));
      manager->SetAttribute ("NgvMcs", UintegerValue (0));
      manager->SetAttribute ("NgvSpatialStreams", UintegerValue (1));
      manager->SetAttribute ("NgvLtfType", UintegerValue (0));
      manager->SetAttribute ("DataMode", WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      manager->SetAttribute ("ControlMode", WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      manager->SetAttribute ("NonUnicastMode",
                             WifiModeValue (WifiMode ("OfdmRate6MbpsBW10MHz")));
      break;
    }
}

VehicularWifiRateController::Policy::Policy ()
{
  linkQuality.minNgv20SnrDb = minLowNgvMcsSnrDb;
  linkQuality.minDuplicate20SnrDb = 6.0;
}

VehicularWifiRateController::Decision
VehicularWifiRateController::Select (const Policy& policy)
{
  return policy.standard == VehicularWifiProfile::Standard::IEEE_80211BD
             ? SelectFor11bd (policy)
             : SelectFor11p (policy);
}

VehicularWifiRateController::Decision
VehicularWifiRateController::SelectFor11bd (const Policy& policy)
{
  VehicularWifiFallbackController::LinkQualityPolicy linkPolicy = policy.linkQuality;
  linkPolicy.minNgv20SnrDb = policy.minLowNgvMcsSnrDb;

  Decision decision;
  decision.standard = VehicularWifiProfile::Standard::IEEE_80211BD;
  decision.fallbackMode = VehicularWifiFallbackController::SelectForLinkQuality (linkPolicy);
  decision.ngvSpatialStreams = NormalizeNgvSpatialStreams (policy.ngvSpatialStreams);
  decision.dataMode = Default11bdDataMode (decision.fallbackMode);

  if (decision.fallbackMode != VehicularWifiFallbackMode::NGV_20)
    {
      decision.ngvMcsIndex = 0;
      decision.ngvSpatialStreams = 1;
      return decision;
    }

  uint8_t requestedMcs = policy.lowNgvMcsIndex;
  if (linkPolicy.estimatedSnrDb >= policy.minHighNgvMcsSnrDb)
    {
      requestedMcs = policy.highNgvMcsIndex;
    }
  else if (linkPolicy.estimatedSnrDb >= policy.minMediumNgvMcsSnrDb)
    {
      requestedMcs = policy.mediumNgvMcsIndex;
    }

  decision.ngvMcsIndex = SelectValidNgvMcs (requestedMcs, 20, decision.ngvSpatialStreams);
  return decision;
}

VehicularWifiRateController::Decision
VehicularWifiRateController::SelectFor11p (const Policy& policy)
{
  Decision decision;
  decision.standard = VehicularWifiProfile::Standard::IEEE_80211P;
  decision.fallbackMode = VehicularWifiFallbackMode::NON_NGV_10;
  decision.ngvMcsIndex = 0;
  decision.ngvSpatialStreams = 1;

  if (policy.linkQuality.estimatedSnrDb >= policy.minHigh11pRateSnrDb)
    {
      decision.dataMode = policy.high11pDataMode;
    }
  else if (policy.linkQuality.estimatedSnrDb >= policy.minMedium11pRateSnrDb)
    {
      decision.dataMode = policy.medium11pDataMode;
    }
  else
    {
      decision.dataMode = policy.low11pDataMode;
    }
  return decision;
}

std::string
VehicularWifiRateController::DecisionName (const Decision& decision)
{
  if (decision.standard == VehicularWifiProfile::Standard::IEEE_80211P)
    {
      return "80211p:" + decision.dataMode;
    }
  return "80211bd:" + VehicularWifiFallbackModeName (decision.fallbackMode) + ":MCS" +
         std::to_string (decision.ngvMcsIndex) + ":NSS" +
         std::to_string (decision.ngvSpatialStreams);
}

VehicularWifiProfile
VehicularWifiRateController::MakeProfile (const Decision& decision)
{
  if (decision.standard == VehicularWifiProfile::Standard::IEEE_80211P)
    {
      return VehicularWifiProfile::Ieee80211p (decision.dataMode);
    }
  return VehicularWifiFallbackController::MakeProfile (decision.fallbackMode,
                                                       decision.ngvMcsIndex,
                                                       decision.ngvSpatialStreams);
}

void
VehicularWifiRateController::ConfigureRemoteStationManager (Ptr<WifiRemoteStationManager> manager,
                                                            const Decision& decision)
{
  if (manager == nullptr)
    {
      return;
    }

  if (decision.standard == VehicularWifiProfile::Standard::IEEE_80211P)
    {
      const WifiMode mode (decision.dataMode);
      manager->SetAttribute ("DataMode", WifiModeValue (mode));
      manager->SetAttribute ("ControlMode", WifiModeValue (mode));
      manager->SetAttribute ("NonUnicastMode", WifiModeValue (mode));
      manager->SetAttribute ("NgvPpduFormat", UintegerValue (0));
      manager->SetAttribute ("NgvMcs", UintegerValue (0));
      manager->SetAttribute ("NgvSpatialStreams", UintegerValue (1));
      manager->SetAttribute ("NgvLtfType", UintegerValue (0));
      manager->SetAttribute ("NgvPpduRepetitions", UintegerValue (0));
      return;
    }

  VehicularWifiFallbackController::ConfigureRemoteStationManager (manager,
                                                                  decision.fallbackMode,
                                                                  decision.ngvMcsIndex,
                                                                  decision.ngvSpatialStreams);
}

} // namespace ns3
