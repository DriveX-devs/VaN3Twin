#ifndef VEHICULAR_WIFI_HELPER_H
#define VEHICULAR_WIFI_HELPER_H

#include "ns3/MetricSupervisor.h"
#include "ns3/ptr.h"
#include "ns3/vehicular-ngv-wifi.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/yans-wifi-helper.h"

#include <cstdint>
#include <string>

namespace ns3
{

class WifiRemoteStationManager;

class VehicularWifiProfile
{
public:
  enum class Standard
  {
    IEEE_80211P,
    IEEE_80211BD
  };

  static VehicularWifiProfile Ieee80211p (const std::string& dataMode = "OfdmRate6MbpsBW10MHz",
                                          double txPowerDbm = 23.0,
                                          double rxSensitivityDbm = -93.0,
                                          double snrThresholdDb = 4.0);
  static VehicularWifiProfile Ieee80211bd (const std::string& dataMode = "OfdmRate12MbpsBW10MHz",
                                           double txPowerDbm = 23.0,
                                           double rxSensitivityDbm = -95.0,
                                           double snrThresholdDb = 4.0,
                                           uint8_t ngvMcsIndex = 3,
                                           uint8_t ngvSpatialStreams = 1,
                                           uint16_t channelWidthMHz = 10);
  static VehicularWifiProfile Ieee80211bd20 (const std::string& dataMode = "OfdmRate24Mbps",
                                             double txPowerDbm = 23.0,
                                             double rxSensitivityDbm = -92.0,
                                             double snrThresholdDb = 4.0,
                                             uint8_t ngvMcsIndex = 3,
                                             uint8_t ngvSpatialStreams = 1);
  static VehicularWifiProfile Ieee80211bdNonNgv10 (const std::string& dataMode = "OfdmRate6MbpsBW10MHz",
                                                   double txPowerDbm = 23.0,
                                                   double rxSensitivityDbm = -93.0,
                                                   double snrThresholdDb = 4.0,
                                                   uint8_t nonNgvRepetitions = 0);
  static VehicularWifiProfile Ieee80211bdNonNgv20Duplicate (const std::string& dataMode = "OfdmRate6MbpsBW10MHz",
                                                            double txPowerDbm = 23.0,
                                                            double rxSensitivityDbm = -90.0,
                                                            double snrThresholdDb = 4.0);

  void ConfigurePhy (YansWifiPhyHelper& wifiPhy) const;
  void ConfigureRemoteStationManager (Wifi80211pHelper& wifiHelper) const;
  void ConfigureMetricSupervisor (Ptr<MetricSupervisor> metricSupervisor) const;

  Standard GetStandard () const;
  std::string GetTechnologyName () const;
  std::string GetDataMode () const;
  std::string GetControlMode () const;
  uint16_t GetChannelWidthMHz () const;
  double GetTxPowerDbm () const;
  double GetRxSensitivityDbm () const;
  double GetSnrThresholdDb () const;
  double GetDccBitRate () const;
  VehicularWifiPpduFormat GetPpduFormat () const;
  std::string GetPpduFormatName () const;
  uint8_t GetNgvMcsIndex () const;
  uint8_t GetNgvSpatialStreams () const;
  uint8_t GetNonNgvRepetitions () const;

private:
  VehicularWifiProfile (Standard standard,
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
                        uint8_t nonNgvRepetitions);

  Standard m_standard;
  std::string m_dataMode;
  std::string m_controlMode;
  uint16_t m_channelWidthMHz;
  double m_txPowerDbm;
  double m_rxSensitivityDbm;
  double m_snrThresholdDb;
  double m_dccBitRate;
  VehicularWifiPpduFormat m_ppduFormat;
  uint8_t m_ngvMcsIndex;
  uint8_t m_ngvSpatialStreams;
  uint8_t m_nonNgvRepetitions;
};

enum class VehicularWifiFallbackMode
{
  NGV_20,
  NON_NGV_20_DUPLICATE,
  NON_NGV_10
};

std::string VehicularWifiFallbackModeName (VehicularWifiFallbackMode mode);

class VehicularWifiFallbackController
{
public:
  struct LinkQualityPolicy
  {
    bool legacyReceiverPresent = false;
    bool duplicate20Allowed = true;
    bool forcePrimaryOnly = false;
    double estimatedSnrDb = 30.0;
    double minNgv20SnrDb = 18.0;
    double minDuplicate20SnrDb = 6.0;
  };

  struct ObservationState
  {
    LinkQualityPolicy policy;
    bool initialized = false;
    double ewmaSnrDb = 30.0;
    double ewmaAlpha = 0.5;
    uint32_t sampleCount = 0;
    uint32_t consecutiveFailures = 0;
    uint32_t failureThreshold = 2;
  };

  static VehicularWifiFallbackMode Select (bool legacyReceiverPresent,
                                           bool duplicate20Allowed = true,
                                           bool forcePrimaryOnly = false);
  static VehicularWifiFallbackMode SelectForLinkQuality (const LinkQualityPolicy& policy);
  static void ResetObservationState (ObservationState& state);
  static void ResetObservationState (ObservationState& state,
                                     const LinkQualityPolicy& initialPolicy);
  static VehicularWifiFallbackMode UpdateFromObservation (ObservationState& state,
                                                          double observedSnrDb,
                                                          bool deliverySucceeded,
                                                          bool legacyReceiverPresent);
  static double EstimateSnrDb (double signalDbm, double noiseDbm);
  static VehicularWifiFallbackMode UpdateFromSignalNoise (ObservationState& state,
                                                          double signalDbm,
                                                          double noiseDbm,
                                                          bool deliverySucceeded,
                                                          bool legacyReceiverPresent);
  static VehicularWifiProfile MakeProfile (VehicularWifiFallbackMode mode,
                                           uint8_t ngvMcsIndex = 3,
                                           uint8_t ngvSpatialStreams = 1);
  static void ConfigureRemoteStationManager (Ptr<WifiRemoteStationManager> manager,
                                             VehicularWifiFallbackMode mode,
                                             uint8_t ngvMcsIndex = 3,
                                             uint8_t ngvSpatialStreams = 1);
};

class VehicularWifiRateController
{
public:
  struct Policy
  {
    Policy ();

    VehicularWifiProfile::Standard standard = VehicularWifiProfile::Standard::IEEE_80211BD;
    VehicularWifiFallbackController::LinkQualityPolicy linkQuality;
    double minLowNgvMcsSnrDb = 12.0;
    double minMediumNgvMcsSnrDb = 18.0;
    double minHighNgvMcsSnrDb = 30.0;
    uint8_t lowNgvMcsIndex = 0;
    uint8_t mediumNgvMcsIndex = 3;
    uint8_t highNgvMcsIndex = 7;
    uint8_t ngvSpatialStreams = 1;
    double minMedium11pRateSnrDb = 8.0;
    double minHigh11pRateSnrDb = 18.0;
    std::string low11pDataMode = "OfdmRate3MbpsBW10MHz";
    std::string medium11pDataMode = "OfdmRate6MbpsBW10MHz";
    std::string high11pDataMode = "OfdmRate12MbpsBW10MHz";
  };

  struct Decision
  {
    VehicularWifiProfile::Standard standard = VehicularWifiProfile::Standard::IEEE_80211BD;
    VehicularWifiFallbackMode fallbackMode = VehicularWifiFallbackMode::NGV_20;
    uint8_t ngvMcsIndex = 3;
    uint8_t ngvSpatialStreams = 1;
    std::string dataMode = "OfdmRate6MbpsBW10MHz";
  };

  static Decision Select (const Policy& policy);
  static Decision SelectFor11bd (const Policy& policy);
  static Decision SelectFor11p (const Policy& policy);
  static std::string DecisionName (const Decision& decision);
  static VehicularWifiProfile MakeProfile (const Decision& decision);
  static void ConfigureRemoteStationManager (Ptr<WifiRemoteStationManager> manager,
                                             const Decision& decision);
};

} // namespace ns3

#endif // VEHICULAR_WIFI_HELPER_H
