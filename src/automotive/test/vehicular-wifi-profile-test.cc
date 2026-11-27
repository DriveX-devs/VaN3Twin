/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/test.h"
#include "ns3/DCC.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/config.h"
#include "ns3/geonet.h"
#include "ns3/vehicular-wifi-helper.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device.h"
#include "ns3/ngv-calibration.h"
#include "ns3/node-container.h"
#include "ns3/ofdm-phy.h"
#include "ns3/packet.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-phy-state.h"
#include "ns3/wifi-ppdu.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-standards.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/yans-error-rate-model.h"

#include <cstddef>
#include <cmath>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

class LinearSnrErrorRateModel : public ErrorRateModel
{
private:
  double DoGetChunkSuccessRate (WifiMode mode,
                                const WifiTxVector& txVector,
                                double snr,
                                uint64_t nbits,
                                uint8_t numRxAntennas,
                                WifiPpduField field,
                                uint16_t staId) const override
  {
    return snr >= 1.0 ? 1.0 : snr;
  }
};

class VehicularWifiProfileTestCase : public TestCase
{
public:
  VehicularWifiProfileTestCase ()
    : TestCase ("Validate vehicular Wi-Fi profile metadata")
  {
  }

private:
  void DoRun () override
  {
    VehicularWifiProfile profile11bd =
        VehicularWifiProfile::Ieee80211bd ("OfdmRate12MbpsBW10MHz", 24.0, -95.0, 4.0);

    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetTechnologyName (), "80211bd", "Unexpected 802.11bd label");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetDataMode (), "OfdmRate12MbpsBW10MHz", "Unexpected 802.11bd data mode");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetControlMode (), "OfdmRate6MbpsBW10MHz", "Unexpected 802.11bd control mode");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetChannelWidthMHz (), 10, "Unexpected 802.11bd channel width");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetPpduFormatName (), "NGV", "Unexpected 802.11bd PPDU format label");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetNgvMcsIndex (), 3, "Unexpected default 802.11bd NGV-MCS");
    NS_TEST_ASSERT_MSG_EQ (profile11bd.GetNgvSpatialStreams (), 1, "Unexpected default 802.11bd spatial stream count");
    NS_TEST_ASSERT_MSG_EQ_TOL (profile11bd.GetDccBitRate (), 13e6, 1.0, "Unexpected 802.11bd DCC bitrate");

    VehicularWifiProfile p =
        VehicularWifiProfile::Ieee80211p ("OfdmRate6MbpsBW10MHz", 23.0, -93.0, 4.0);
    NS_TEST_ASSERT_MSG_EQ (p.GetTechnologyName (), "80211p", "Unexpected 802.11p label");
    NS_TEST_ASSERT_MSG_EQ (p.GetPpduFormatName (), "NON_NGV_10", "Unexpected 802.11p PPDU format");
    NS_TEST_ASSERT_MSG_EQ_TOL (p.GetDccBitRate (), 6e6, 1.0, "Unexpected 802.11p DCC bitrate");

    VehicularWifiProfile compatible11bd =
        VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz", 23.0, -93.0, 4.0, 2);
    NS_TEST_ASSERT_MSG_EQ (compatible11bd.GetTechnologyName (), "80211bd", "Unexpected compatible 802.11bd label");
    NS_TEST_ASSERT_MSG_EQ (compatible11bd.GetPpduFormatName (), "NON_NGV_10", "Unexpected compatible 802.11bd PPDU format");
    NS_TEST_ASSERT_MSG_EQ (compatible11bd.GetNonNgvRepetitions (), 2, "Unexpected NON_NGV_10 repetition count");

    VehicularWifiProfile profile11bd20 =
        VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps", 23.0, -92.0, 4.0, 9);
    NS_TEST_ASSERT_MSG_EQ (profile11bd20.GetTechnologyName (), "80211bd", "Unexpected 20 MHz 802.11bd label");
    NS_TEST_ASSERT_MSG_EQ (profile11bd20.GetChannelWidthMHz (), 20, "Unexpected 20 MHz 802.11bd channel width");
    NS_TEST_ASSERT_MSG_EQ (profile11bd20.GetNgvMcsIndex (), 9, "Unexpected 20 MHz 802.11bd NGV-MCS");
    NS_TEST_ASSERT_MSG_EQ_TOL (profile11bd20.GetDccBitRate (), 90e6, 1.0, "Unexpected 20 MHz NGV-MCS 9 bitrate");

    VehicularWifiProfile duplicate11bd =
        VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate ();
    NS_TEST_ASSERT_MSG_EQ (duplicate11bd.GetPpduFormatName (),
                           "NON_NGV_20_DUPLICATE",
                           "Unexpected 20 MHz duplicate PPDU format");
    NS_TEST_ASSERT_MSG_EQ (duplicate11bd.GetChannelWidthMHz (),
                           20,
                           "Unexpected 20 MHz duplicate channel width");
  }
};

class VehicularNgvMcsTestCase : public TestCase
{
public:
  VehicularNgvMcsTestCase ()
    : TestCase ("Validate 802.11bd NGV-MCS metadata")
  {
  }

private:
  void DoRun () override
  {
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (0), true, "NGV-MCS 0 should be valid for 10 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (8), true, "NGV-MCS 8 should be valid for 10 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (15), true, "NGV-MCS 15 should be valid for 10 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (9), false, "NGV-MCS 9 should be invalid for 10 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (10), false, "NGV-MCS 10 should be reserved for 10 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (9, 20, 1), true, "NGV-MCS 9 should be valid for 20 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (15, 20, 1), true, "NGV-MCS 15 should be valid for 20 MHz, NSS=1");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (15, 20, 2), false, "NGV-MCS 15 should be invalid for NSS=2");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (8, 10, 2), true, "NGV-MCS 8 should be valid for 10 MHz, NSS=2");
    NS_TEST_ASSERT_MSG_EQ (IsVehicularNgvMcsValid (9, 10, 2), false, "NGV-MCS 9 should be invalid for 10 MHz, NSS=2");

    const auto& mcs3 = GetVehicularNgvMcs (3);
    NS_TEST_ASSERT_MSG_EQ (mcs3.index, 3, "Unexpected NGV-MCS index");
    NS_TEST_ASSERT_MSG_EQ (std::string (mcs3.modulation), "16-QAM", "Unexpected NGV-MCS modulation");
    NS_TEST_ASSERT_MSG_EQ (mcs3.codeRateNumerator, 1, "Unexpected NGV-MCS code rate numerator");
    NS_TEST_ASSERT_MSG_EQ (mcs3.codeRateDenominator, 2, "Unexpected NGV-MCS code rate denominator");
    NS_TEST_ASSERT_MSG_EQ (mcs3.dataBitsPerSymbol, 104, "Unexpected NGV-MCS NDBPS");
    NS_TEST_ASSERT_MSG_EQ_TOL (mcs3.dataRateMbps, 13.0, 0.001, "Unexpected NGV-MCS data rate");

    const auto& mcs15 = GetVehicularNgvMcs (15);
    NS_TEST_ASSERT_MSG_EQ (mcs15.dcm, true, "NGV-MCS 15 should use DCM");
    NS_TEST_ASSERT_MSG_EQ_TOL (mcs15.dataRateMbps, 1.6, 0.001, "Unexpected NGV-MCS 15 data rate");

    const auto& mcs9_20 = GetVehicularNgvMcs (9, 20, 1);
    NS_TEST_ASSERT_MSG_EQ (mcs9_20.dataBitsPerSymbol, 720, "Unexpected 20 MHz NGV-MCS 9 NDBPS");
    NS_TEST_ASSERT_MSG_EQ_TOL (mcs9_20.dataRateMbps, 90.0, 0.001, "Unexpected 20 MHz NGV-MCS 9 data rate");

    const auto& mcs3_20nss2 = GetVehicularNgvMcs (3, 20, 2);
    NS_TEST_ASSERT_MSG_EQ (mcs3_20nss2.dataBitsPerSymbol, 432, "Unexpected 20 MHz NSS=2 NGV-MCS 3 NDBPS");
    NS_TEST_ASSERT_MSG_EQ_TOL (mcs3_20nss2.dataRateMbps, 54.0, 0.001, "Unexpected 20 MHz NSS=2 data rate");
  }
};

class VehicularWifiProfileInstallTestCase : public TestCase
{
public:
  VehicularWifiProfileInstallTestCase ()
    : TestCase ("Validate installed 802.11bd PHY profile")
  {
  }

private:
  void DoRun () override
  {
    VehicularWifiProfile profile11bd =
        VehicularWifiProfile::Ieee80211bd ("OfdmRate12MbpsBW10MHz", 24.0, -95.0, 4.0);

    NodeContainer nodes;
    nodes.Create (2);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiPhyHelper phy;
    profile11bd.ConfigurePhy (phy);
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    phy.SetChannel (channel.Create ());

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile11bd.ConfigureRemoteStationManager (wifi);

    NetDeviceContainer devices = wifi.Install (phy, mac, nodes);
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_TEST_ASSERT_MSG_NE (device, nullptr, "Expected a Wi-Fi net device");

    Ptr<WifiPhy> installedPhy = device->GetPhy ();
    NS_TEST_ASSERT_MSG_EQ (installedPhy->GetStandard (),
                           WIFI_STANDARD_80211bd,
                           "Installed PHY standard does not match 802.11bd profile");
    NS_TEST_ASSERT_MSG_EQ (installedPhy->GetChannelWidth (),
                           profile11bd.GetChannelWidthMHz (),
                           "Installed PHY channel width does not match profile");
    NS_TEST_ASSERT_MSG_EQ (installedPhy->GetNumberOfAntennas (),
                           profile11bd.GetNgvSpatialStreams (),
                           "Installed 802.11bd PHY antenna count does not match profile NSS");
    NS_TEST_ASSERT_MSG_EQ (installedPhy->GetMaxSupportedTxSpatialStreams (),
                           profile11bd.GetNgvSpatialStreams (),
                           "Installed 802.11bd PHY TX spatial stream support does not match profile NSS");
    NS_TEST_ASSERT_MSG_EQ (installedPhy->GetMaxSupportedRxSpatialStreams (),
                           profile11bd.GetNgvSpatialStreams (),
                           "Installed 802.11bd PHY RX spatial stream support does not match profile NSS");
    NS_TEST_ASSERT_MSG_EQ_TOL (installedPhy->GetRxSensitivity (),
                               profile11bd.GetRxSensitivityDbm (),
                               0.001,
                               "Installed PHY RxSensitivity does not match profile");
    NS_TEST_ASSERT_MSG_EQ_TOL (installedPhy->GetCcaEdThreshold (),
                               -65.0,
                               0.001,
                               "Installed 802.11bd PHY CCA ED threshold should match the primary 10 MHz NGV threshold");
    NS_TEST_ASSERT_MSG_EQ_TOL (installedPhy->GetTxPowerStart (),
                               profile11bd.GetTxPowerDbm (),
                               0.001,
                               "Installed PHY TxPowerStart does not match profile");
    NS_TEST_ASSERT_MSG_EQ_TOL (installedPhy->GetTxPowerEnd (),
                               profile11bd.GetTxPowerDbm (),
                               0.001,
                               "Installed PHY TxPowerEnd does not match profile");

    Ptr<WifiRemoteStationManager> stationManager = device->GetRemoteStationManager ();
    NS_TEST_ASSERT_MSG_NE (stationManager, nullptr, "Expected a Wi-Fi remote station manager");

    WifiMacHeader broadcastData;
    broadcastData.SetType (WIFI_MAC_DATA);
    broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
    broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
    broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());

    WifiTxVector broadcastTxVector = stationManager->GetDataTxVector (broadcastData);
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.IsNgv (), true, "802.11bd broadcast data TXVECTOR should be NGV");
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.GetNgvMcs (),
                           profile11bd.GetNgvMcsIndex (),
                           "802.11bd broadcast data TXVECTOR lost NGV-MCS metadata");
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.GetChannelWidth (),
                           profile11bd.GetChannelWidthMHz (),
                           "802.11bd broadcast data TXVECTOR channel width does not match profile");
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.GetNss (),
                           profile11bd.GetNgvSpatialStreams (),
                           "802.11bd broadcast data TXVECTOR NSS does not match profile");
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.IsLdpc (), true, "802.11bd NGV data TXVECTOR should request LDPC");
    NS_TEST_ASSERT_MSG_EQ (broadcastTxVector.IsValid (), true, "802.11bd broadcast data TXVECTOR should be valid");

    VehicularWifiProfile compatible11bd =
        VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz", 23.0, -93.0, 4.0, 2);
    YansWifiPhyHelper compatiblePhy;
    compatible11bd.ConfigurePhy (compatiblePhy);
    compatiblePhy.SetChannel (channel.Create ());

    Wifi80211pHelper compatibleWifi = Wifi80211pHelper::Default ();
    compatible11bd.ConfigureRemoteStationManager (compatibleWifi);

    NetDeviceContainer compatibleDevices = compatibleWifi.Install (compatiblePhy, mac, nodes);
    Ptr<WifiNetDevice> compatibleDevice = DynamicCast<WifiNetDevice> (compatibleDevices.Get (0));
    NS_TEST_ASSERT_MSG_NE (compatibleDevice, nullptr, "Expected a compatible 802.11bd Wi-Fi net device");

    WifiTxVector compatibleBroadcastTxVector =
        compatibleDevice->GetRemoteStationManager ()->GetDataTxVector (broadcastData);
    NS_TEST_ASSERT_MSG_EQ (compatibleBroadcastTxVector.IsNonNgv10 (),
                           true,
                           "Compatible 802.11bd broadcast data TXVECTOR should be NON_NGV_10");
    NS_TEST_ASSERT_MSG_EQ (compatibleBroadcastTxVector.GetNgvPpduRepetitions (),
                           compatible11bd.GetNonNgvRepetitions (),
                           "Compatible 802.11bd broadcast data TXVECTOR lost N_PPDU_REP metadata");
    NS_TEST_ASSERT_MSG_EQ (compatibleBroadcastTxVector.IsLdpc (),
                           false,
                           "NON_NGV_10 broadcast data TXVECTOR should not request NGV LDPC");

    VehicularWifiProfile profile11bd20 =
        VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps", 23.0, -92.0, 4.0, 9, 2);
    YansWifiPhyHelper phy20;
    profile11bd20.ConfigurePhy (phy20);
    phy20.SetChannel (channel.Create ());

    Wifi80211pHelper wifi20 = Wifi80211pHelper::Default ();
    profile11bd20.ConfigureRemoteStationManager (wifi20);

    NetDeviceContainer devices20 = wifi20.Install (phy20, mac, nodes);
    Ptr<WifiNetDevice> device20 = DynamicCast<WifiNetDevice> (devices20.Get (0));
    NS_TEST_ASSERT_MSG_NE (device20, nullptr, "Expected a 20 MHz 802.11bd Wi-Fi net device");
    NS_TEST_ASSERT_MSG_EQ (device20->GetPhy ()->GetChannelWidth (),
                           20,
                           "Installed 20 MHz 802.11bd PHY lost channel width");
    NS_TEST_ASSERT_MSG_EQ (device20->GetPhy ()->GetNumberOfAntennas (),
                           2,
                           "Installed 20 MHz NSS=2 PHY lost antenna count");
    NS_TEST_ASSERT_MSG_EQ (device20->GetPhy ()->GetMaxSupportedTxSpatialStreams (),
                           2,
                           "Installed 20 MHz NSS=2 PHY lost TX spatial stream support");
    NS_TEST_ASSERT_MSG_EQ (device20->GetPhy ()->GetMaxSupportedRxSpatialStreams (),
                           2,
                           "Installed 20 MHz NSS=2 PHY lost RX spatial stream support");

    WifiTxVector txVector20 =
        device20->GetRemoteStationManager ()->GetDataTxVector (broadcastData);
    NS_TEST_ASSERT_MSG_EQ (txVector20.IsNgv (), true, "20 MHz 802.11bd broadcast TXVECTOR should be NGV");
    NS_TEST_ASSERT_MSG_EQ (txVector20.GetChannelWidth (), 20, "20 MHz NGV TXVECTOR lost channel width");
    NS_TEST_ASSERT_MSG_EQ (txVector20.GetNgvMcs (), 9, "20 MHz NGV TXVECTOR lost NGV-MCS");
    NS_TEST_ASSERT_MSG_EQ (txVector20.GetNss (), 2, "20 MHz NGV TXVECTOR lost NSS=2 metadata");
    NS_TEST_ASSERT_MSG_EQ (txVector20.IsValid (), true, "20 MHz NGV TXVECTOR should be valid");

    VehicularWifiProfile duplicate11bd =
        VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate ();
    YansWifiPhyHelper duplicatePhy;
    duplicate11bd.ConfigurePhy (duplicatePhy);
    duplicatePhy.SetChannel (channel.Create ());

    Wifi80211pHelper duplicateWifi = Wifi80211pHelper::Default ();
    duplicate11bd.ConfigureRemoteStationManager (duplicateWifi);

    NetDeviceContainer duplicateDevices = duplicateWifi.Install (duplicatePhy, mac, nodes);
    Ptr<WifiNetDevice> duplicateDevice = DynamicCast<WifiNetDevice> (duplicateDevices.Get (0));
    NS_TEST_ASSERT_MSG_NE (duplicateDevice, nullptr, "Expected a 20 MHz duplicate 802.11bd Wi-Fi net device");
    WifiTxVector duplicateTxVector =
        duplicateDevice->GetRemoteStationManager ()->GetDataTxVector (broadcastData);
    NS_TEST_ASSERT_MSG_EQ (duplicateTxVector.IsNonNgv20Duplicate (),
                           true,
                           "20 MHz duplicate profile should transmit NON_NGV_20_DUPLICATE");
    NS_TEST_ASSERT_MSG_EQ (duplicateTxVector.GetChannelWidth (),
                           20,
                           "20 MHz duplicate TXVECTOR lost channel width");
    NS_TEST_ASSERT_MSG_EQ (duplicateTxVector.IsLdpc (),
                           false,
                           "20 MHz duplicate TXVECTOR should not request NGV LDPC");

    Simulator::Destroy ();
  }
};

class VehicularWifiFallbackControllerTestCase : public TestCase
{
public:
  VehicularWifiFallbackControllerTestCase ()
    : TestCase ("Validate 802.11bd fallback policy selection")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  WifiMacHeader MakeBroadcastDataHeader () const
  {
    WifiMacHeader broadcastData;
    broadcastData.SetType (WIFI_MAC_DATA);
    broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
    broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
    broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());
    return broadcastData;
  }

  void CheckAppliedMode (const std::string& scenario,
                         VehicularWifiFallbackMode mode,
                         const VehicularWifiProfile& installProfile)
  {
    NodeContainer nodes;
    nodes.Create (1);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();
    NetDeviceContainer devices = InstallVehicularDevice (nodes, channel, installProfile);
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_TEST_ASSERT_MSG_NE (device, nullptr, scenario << " expected a Wi-Fi net device");

    Ptr<WifiRemoteStationManager> manager = device->GetRemoteStationManager ();
    VehicularWifiFallbackController::ConfigureRemoteStationManager (manager, mode);
    WifiTxVector txVector = manager->GetDataTxVector (MakeBroadcastDataHeader ());

    if (mode == VehicularWifiFallbackMode::NGV_20)
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNgv (), true, scenario << " should select native NGV");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (), 20, scenario << " should use 20 MHz NGV");
      }
    else if (mode == VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE)
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNonNgv20Duplicate (),
                               true,
                               scenario << " should select NON_NGV_20_DUPLICATE");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (),
                               20,
                               scenario << " should occupy a 20 MHz duplicate channel");
      }
    else
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNonNgv10 (),
                               true,
                               scenario << " should select primary-only NON_NGV_10");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (),
                               10,
                               scenario << " should use the 10 MHz primary channel");
      }

    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, scenario << " selected TXVECTOR should be valid");
    Simulator::Destroy ();
  }

  void DoRun () override
  {
    const VehicularWifiFallbackMode nativeMode =
        VehicularWifiFallbackController::Select (false, true, false);
    const VehicularWifiFallbackMode duplicateMode =
        VehicularWifiFallbackController::Select (true, true, false);
    const VehicularWifiFallbackMode primaryMode =
        VehicularWifiFallbackController::Select (true, false, false);
    const VehicularWifiFallbackMode forcedPrimaryMode =
        VehicularWifiFallbackController::Select (true, true, true);
    VehicularWifiFallbackController::LinkQualityPolicy highQualityPolicy;
    highQualityPolicy.legacyReceiverPresent = false;
    highQualityPolicy.estimatedSnrDb = 25.0;
    VehicularWifiFallbackController::LinkQualityPolicy midQualityPolicy;
    midQualityPolicy.legacyReceiverPresent = false;
    midQualityPolicy.estimatedSnrDb = 12.0;
    VehicularWifiFallbackController::LinkQualityPolicy lowQualityPolicy;
    lowQualityPolicy.legacyReceiverPresent = false;
    lowQualityPolicy.estimatedSnrDb = 2.0;
    VehicularWifiFallbackController::LinkQualityPolicy legacyHighQualityPolicy;
    legacyHighQualityPolicy.legacyReceiverPresent = true;
    legacyHighQualityPolicy.estimatedSnrDb = 25.0;
    VehicularWifiFallbackController::LinkQualityPolicy legacyLowQualityPolicy;
    legacyLowQualityPolicy.legacyReceiverPresent = true;
    legacyLowQualityPolicy.estimatedSnrDb = 2.0;

    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (nativeMode),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
                           "Policy without legacy receivers should select 20 MHz NGV");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (duplicateMode),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
                           "Policy with legacy receivers should prefer 20 MHz duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (primaryMode),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
                           "Policy should fall back to primary-only when duplicate is disabled");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (forcedPrimaryMode),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
                           "Policy should honor forced primary-only fallback");
    NS_TEST_ASSERT_MSG_EQ (VehicularWifiFallbackModeName (duplicateMode),
                           "NON_NGV_20_DUPLICATE",
                           "Unexpected fallback mode name");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (VehicularWifiFallbackController::SelectForLinkQuality (highQualityPolicy)),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
        "High-quality non-legacy policy should select 20 MHz NGV");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (VehicularWifiFallbackController::SelectForLinkQuality (midQualityPolicy)),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "Mid-quality non-legacy policy should select 20 MHz duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (VehicularWifiFallbackController::SelectForLinkQuality (lowQualityPolicy)),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
        "Low-quality non-legacy policy should select primary-only fallback");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (VehicularWifiFallbackController::SelectForLinkQuality (legacyHighQualityPolicy)),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "High-quality legacy policy should select 20 MHz duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (VehicularWifiFallbackController::SelectForLinkQuality (legacyLowQualityPolicy)),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
        "Low-quality legacy policy should select primary-only fallback");

    VehicularWifiFallbackController::ObservationState observationState;
    observationState.ewmaAlpha = 0.5;
    observationState.policy.minNgv20SnrDb = 18.0;
    observationState.policy.minDuplicate20SnrDb = 6.0;
    VehicularWifiFallbackMode observedHigh =
        VehicularWifiFallbackController::UpdateFromObservation (observationState,
                                                                25.0,
                                                                true,
                                                                false);
    VehicularWifiFallbackMode observedMid =
        VehicularWifiFallbackController::UpdateFromObservation (observationState,
                                                                9.0,
                                                                true,
                                                                false);
    VehicularWifiFallbackMode observedLowFailureOne =
        VehicularWifiFallbackController::UpdateFromObservation (observationState,
                                                                2.0,
                                                                false,
                                                                false);
    VehicularWifiFallbackMode observedLowFailureTwo =
        VehicularWifiFallbackController::UpdateFromObservation (observationState,
                                                                2.0,
                                                                false,
                                                                false);
    VehicularWifiFallbackMode observedLegacyRecovery =
        VehicularWifiFallbackController::UpdateFromObservation (observationState,
                                                                25.0,
                                                                true,
                                                                true);
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (observedHigh),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
                           "High-SNR observed policy should select 20 MHz NGV");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (observedMid),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "EWMA mid-SNR observed policy should select 20 MHz duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (observedLowFailureOne),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "First low-SNR failure should not force primary-only before the failure threshold");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (observedLowFailureTwo),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
        "Failure-threshold crossing should force primary-only for the current decision");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (observedLegacyRecovery),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "Legacy recovery observation should clear the failure streak and select duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ_TOL (observationState.ewmaSnrDb,
                               15.375,
                               0.001,
                               "Unexpected observed-policy EWMA SNR");
    NS_TEST_ASSERT_MSG_EQ (observationState.sampleCount,
                           5,
                           "Unexpected observed-policy sample count");
    NS_TEST_ASSERT_MSG_EQ (observationState.consecutiveFailures,
                           0,
                           "Successful observation should clear consecutive failures");

    NS_TEST_ASSERT_MSG_EQ_TOL (VehicularWifiFallbackController::EstimateSnrDb (-70.0, -95.0),
                               25.0,
                               0.001,
                               "Signal/noise adapter should calculate SNR in dB");

    VehicularWifiFallbackController::ObservationState traceObservationState;
    traceObservationState.ewmaAlpha = 0.5;
    traceObservationState.policy.minNgv20SnrDb = 18.0;
    traceObservationState.policy.minDuplicate20SnrDb = 6.0;
    VehicularWifiFallbackMode traceHigh =
        VehicularWifiFallbackController::UpdateFromSignalNoise (traceObservationState,
                                                                -70.0,
                                                                -95.0,
                                                                true,
                                                                false);
    VehicularWifiFallbackMode traceLegacy =
        VehicularWifiFallbackController::UpdateFromSignalNoise (traceObservationState,
                                                                -70.0,
                                                                -95.0,
                                                                true,
                                                                true);
    VehicularWifiFallbackMode traceLowFailure =
        VehicularWifiFallbackController::UpdateFromSignalNoise (traceObservationState,
                                                                -93.0,
                                                                -95.0,
                                                                false,
                                                                true);
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (traceHigh),
                           static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
                           "Trace SNR adapter should keep high-quality non-legacy links on NGV");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (traceLegacy),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "Trace SNR adapter should select duplicate fallback for high-quality legacy links");
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (traceLowFailure),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "First trace-observed failure should not force primary-only before the threshold");
    NS_TEST_ASSERT_MSG_EQ (traceObservationState.sampleCount,
                           3,
                           "Trace SNR adapter should update observation sample count");
    NS_TEST_ASSERT_MSG_EQ (traceObservationState.consecutiveFailures,
                           1,
                           "Trace SNR adapter should preserve the observed failure streak");

    VehicularWifiFallbackController::LinkQualityPolicy resetPolicy;
    resetPolicy.estimatedSnrDb = 12.0;
    observationState.ewmaAlpha = 0.25;
    observationState.failureThreshold = 3;
    VehicularWifiFallbackController::ResetObservationState (observationState, resetPolicy);
    NS_TEST_ASSERT_MSG_EQ (observationState.initialized,
                           false,
                           "Reset observed-policy state should be uninitialized");
    NS_TEST_ASSERT_MSG_EQ_TOL (observationState.ewmaSnrDb,
                               12.0,
                               0.001,
                               "Reset observed-policy EWMA should use the initial policy SNR");
    NS_TEST_ASSERT_MSG_EQ_TOL (observationState.ewmaAlpha,
                               0.25,
                               0.001,
                               "Reset observed-policy state should preserve EWMA alpha");
    NS_TEST_ASSERT_MSG_EQ (observationState.failureThreshold,
                           3,
                           "Reset observed-policy state should preserve failure threshold");

    VehicularWifiProfile nativeProfile =
        VehicularWifiFallbackController::MakeProfile (VehicularWifiFallbackMode::NGV_20);
    VehicularWifiProfile duplicateProfile =
        VehicularWifiFallbackController::MakeProfile (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE);
    VehicularWifiProfile primaryProfile =
        VehicularWifiFallbackController::MakeProfile (VehicularWifiFallbackMode::NON_NGV_10);
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (nativeProfile.GetPpduFormat ()),
                           static_cast<uint32_t> (VehicularWifiPpduFormat::NGV),
                           "Native policy profile should use NGV");
    NS_TEST_ASSERT_MSG_EQ (nativeProfile.GetChannelWidthMHz (),
                           20,
                           "Native policy profile should use 20 MHz");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (duplicateProfile.GetPpduFormat ()),
                           static_cast<uint32_t> (VehicularWifiPpduFormat::NON_NGV_20_DUPLICATE),
                           "Duplicate policy profile should use NON_NGV_20_DUPLICATE");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (primaryProfile.GetPpduFormat ()),
                           static_cast<uint32_t> (VehicularWifiPpduFormat::NON_NGV_10),
                           "Primary-only policy profile should use NON_NGV_10");
    NS_TEST_ASSERT_MSG_EQ (primaryProfile.GetChannelWidthMHz (),
                           10,
                           "Primary-only policy profile should use 10 MHz");

    CheckAppliedMode ("20 MHz NGV fallback policy",
                      VehicularWifiFallbackMode::NGV_20,
                      nativeProfile);
    CheckAppliedMode ("20 MHz duplicate fallback policy",
                      VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE,
                      duplicateProfile);
    CheckAppliedMode ("10 MHz primary-only fallback policy",
                      VehicularWifiFallbackMode::NON_NGV_10,
                      primaryProfile);
  }
};

class VehicularWifiRateControllerTestCase : public TestCase
{
public:
  VehicularWifiRateControllerTestCase ()
    : TestCase ("Validate modular 802.11bd/802.11p rate control decisions")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  WifiMacHeader MakeBroadcastDataHeader () const
  {
    WifiMacHeader broadcastData;
    broadcastData.SetType (WIFI_MAC_DATA);
    broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
    broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
    broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());
    return broadcastData;
  }

  void CheckAppliedDecision (const std::string& scenario,
                             const VehicularWifiRateController::Decision& decision)
  {
    NodeContainer nodes;
    nodes.Create (1);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();
    NetDeviceContainer devices =
        InstallVehicularDevice (nodes, channel, VehicularWifiRateController::MakeProfile (decision));
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_TEST_ASSERT_MSG_NE (device, nullptr, scenario << " expected a Wi-Fi net device");

    Ptr<WifiRemoteStationManager> manager = device->GetRemoteStationManager ();
    VehicularWifiRateController::ConfigureRemoteStationManager (manager, decision);
    WifiTxVector txVector = manager->GetDataTxVector (MakeBroadcastDataHeader ());

    if (decision.standard == VehicularWifiProfile::Standard::IEEE_80211P)
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.GetMode ().GetUniqueName (),
                               decision.dataMode,
                               scenario << " did not apply the 802.11p data mode");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (),
                               10,
                               scenario << " should keep 802.11p on a 10 MHz channel");
      }
    else if (decision.fallbackMode == VehicularWifiFallbackMode::NGV_20)
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNgv (), true, scenario << " should apply NGV");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (),
                               20,
                               scenario << " should apply a 20 MHz NGV channel");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetNgvMcs (),
                               decision.ngvMcsIndex,
                               scenario << " did not apply the selected NGV-MCS");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetNss (),
                               decision.ngvSpatialStreams,
                               scenario << " did not apply the selected NGV NSS");
      }
    else if (decision.fallbackMode == VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE)
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNonNgv20Duplicate (),
                               true,
                               scenario << " should apply NON_NGV_20_DUPLICATE");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetNgvMcs (),
                               0,
                               scenario << " duplicate fallback should clear NGV-MCS");
      }
    else
      {
        NS_TEST_ASSERT_MSG_EQ (txVector.IsNonNgv10 (),
                               true,
                               scenario << " should apply NON_NGV_10");
        NS_TEST_ASSERT_MSG_EQ (txVector.GetChannelWidth (),
                               10,
                               scenario << " should apply a 10 MHz primary channel");
      }

    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, scenario << " selected TXVECTOR should be valid");
    Simulator::Destroy ();
  }

  void DoRun () override
  {
    VehicularWifiRateController::Policy high11bd;
    high11bd.linkQuality.estimatedSnrDb = 35.0;
    VehicularWifiRateController::Decision high11bdDecision =
        VehicularWifiRateController::Select (high11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (high11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
        "High-SNR 802.11bd rate control should keep native NGV");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (high11bdDecision.ngvMcsIndex),
                           7,
                           "High-SNR 802.11bd rate control should select NGV-MCS 7");
    NS_TEST_ASSERT_MSG_EQ (VehicularWifiRateController::DecisionName (high11bdDecision),
                           "80211bd:NGV_20:MCS7:NSS1",
                           "Unexpected high-SNR 802.11bd decision name");

    VehicularWifiRateController::Policy medium11bd;
    medium11bd.linkQuality.estimatedSnrDb = 22.0;
    VehicularWifiRateController::Decision medium11bdDecision =
        VehicularWifiRateController::Select (medium11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (medium11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
        "Medium-SNR 802.11bd rate control should stay on native NGV");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (medium11bdDecision.ngvMcsIndex),
                           3,
                           "Medium-SNR 802.11bd rate control should select NGV-MCS 3");

    VehicularWifiRateController::Policy low11bd;
    low11bd.linkQuality.estimatedSnrDb = 14.0;
    VehicularWifiRateController::Decision low11bdDecision =
        VehicularWifiRateController::Select (low11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (low11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NGV_20),
        "Low-but-usable 802.11bd SNR should use the lowest NGV-MCS before fallback");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (low11bdDecision.ngvMcsIndex),
                           0,
                           "Low-but-usable 802.11bd SNR should select NGV-MCS 0");

    VehicularWifiRateController::Policy duplicate11bd;
    duplicate11bd.linkQuality.estimatedSnrDb = 8.0;
    VehicularWifiRateController::Decision duplicate11bdDecision =
        VehicularWifiRateController::Select (duplicate11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (duplicate11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "Sub-NGV 802.11bd SNR should select duplicate fallback when it is still usable");

    VehicularWifiRateController::Policy primary11bd;
    primary11bd.linkQuality.estimatedSnrDb = 2.0;
    VehicularWifiRateController::Decision primary11bdDecision =
        VehicularWifiRateController::Select (primary11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (primary11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_10),
        "Very low 802.11bd SNR should select primary-only fallback");

    VehicularWifiRateController::Policy legacy11bd;
    legacy11bd.linkQuality.estimatedSnrDb = 35.0;
    legacy11bd.linkQuality.legacyReceiverPresent = true;
    VehicularWifiRateController::Decision legacy11bdDecision =
        VehicularWifiRateController::Select (legacy11bd);
    NS_TEST_ASSERT_MSG_EQ (
        static_cast<uint32_t> (legacy11bdDecision.fallbackMode),
        static_cast<uint32_t> (VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE),
        "802.11bd rate control should keep legacy receivers on duplicate fallback");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (legacy11bdDecision.ngvMcsIndex),
                           0,
                           "Legacy fallback should not retain a native NGV-MCS");

    VehicularWifiRateController::Policy invalidMcs11bd;
    invalidMcs11bd.linkQuality.estimatedSnrDb = 35.0;
    invalidMcs11bd.highNgvMcsIndex = 15;
    invalidMcs11bd.ngvSpatialStreams = 2;
    VehicularWifiRateController::Decision clamped11bdDecision =
        VehicularWifiRateController::Select (invalidMcs11bd);
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (clamped11bdDecision.ngvSpatialStreams),
                           2,
                           "Valid NSS=2 request should be preserved");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (clamped11bdDecision.ngvMcsIndex),
                           9,
                           "Invalid 20 MHz NSS=2 NGV-MCS request should degrade to MCS 9");

    VehicularWifiRateController::Policy high11p;
    high11p.standard = VehicularWifiProfile::Standard::IEEE_80211P;
    high11p.linkQuality.estimatedSnrDb = 20.0;
    VehicularWifiRateController::Decision high11pDecision =
        VehicularWifiRateController::Select (high11p);
    NS_TEST_ASSERT_MSG_EQ (high11pDecision.dataMode,
                           "OfdmRate12MbpsBW10MHz",
                           "High-SNR 802.11p rate control should select the high data mode");
    NS_TEST_ASSERT_MSG_EQ (VehicularWifiRateController::DecisionName (high11pDecision),
                           "80211p:OfdmRate12MbpsBW10MHz",
                           "Unexpected high-SNR 802.11p decision name");

    VehicularWifiRateController::Policy medium11p;
    medium11p.standard = VehicularWifiProfile::Standard::IEEE_80211P;
    medium11p.linkQuality.estimatedSnrDb = 10.0;
    VehicularWifiRateController::Decision medium11pDecision =
        VehicularWifiRateController::Select (medium11p);
    NS_TEST_ASSERT_MSG_EQ (medium11pDecision.dataMode,
                           "OfdmRate6MbpsBW10MHz",
                           "Medium-SNR 802.11p rate control should select the middle data mode");

    VehicularWifiRateController::Policy low11p;
    low11p.standard = VehicularWifiProfile::Standard::IEEE_80211P;
    low11p.linkQuality.estimatedSnrDb = 2.0;
    VehicularWifiRateController::Decision low11pDecision =
        VehicularWifiRateController::Select (low11p);
    NS_TEST_ASSERT_MSG_EQ (low11pDecision.dataMode,
                           "OfdmRate3MbpsBW10MHz",
                           "Low-SNR 802.11p rate control should select the low data mode");

    CheckAppliedDecision ("High-SNR 802.11bd rate control", high11bdDecision);
    CheckAppliedDecision ("Medium-SNR 802.11bd rate control", medium11bdDecision);
    CheckAppliedDecision ("Legacy-compatible 802.11bd rate control", legacy11bdDecision);
    CheckAppliedDecision ("Primary-only 802.11bd rate control", primary11bdDecision);
    CheckAppliedDecision ("High-SNR 802.11p rate control", high11pDecision);
  }
};

class VehicularWifiCore11bdTestCase : public TestCase
{
public:
  VehicularWifiCore11bdTestCase ()
    : TestCase ("Validate core 802.11bd Wi-Fi metadata")
  {
  }

private:
  void DoRun () override
  {
    NS_TEST_ASSERT_MSG_EQ (GetFrequencyChannelType (WIFI_STANDARD_80211bd),
                           WIFI_PHY_80211p_CHANNEL,
                           "802.11bd should use the vehicular operating channel type");
    NS_TEST_ASSERT_MSG_EQ (GetMaximumChannelWidth (WIFI_STANDARD_80211bd),
                           20,
                           "Unexpected 802.11bd maximum channel width");
    NS_TEST_ASSERT_MSG_EQ (GetDefaultChannelWidth (WIFI_STANDARD_80211bd, WIFI_PHY_BAND_5GHZ),
                           10,
                           "Unexpected 802.11bd default channel width");
    NS_TEST_ASSERT_MSG_EQ (GetDefaultPhyBand (WIFI_STANDARD_80211bd),
                           WIFI_PHY_BAND_5GHZ,
                           "Unexpected 802.11bd default band");

    WifiTxVector txVector (OfdmPhy::GetOfdmRate12MbpsBW10MHz (),
                           0,
                           WIFI_PREAMBLE_LONG,
                           800,
                           1,
                           1,
                           0,
                           10,
                           false,
                           false,
                           true);
    txVector.SetNgvPpduFormat (WifiNgvPpduFormat::NGV);
    txVector.SetNgvMcs (3);
    txVector.SetNgvMidamblePeriodicity (16);
    txVector.SetNgvLtfType (WifiNgvLtfType::NGV_LTF_2X);
    txVector.SetNgvPpduRepetitions (0);

    NS_TEST_ASSERT_MSG_EQ (txVector.IsNgv (), true, "Expected an NGV PPDU");
    NS_TEST_ASSERT_MSG_EQ (txVector.IsNonNgv10 (), false, "NGV PPDU should not be NON_NGV_10");
    NS_TEST_ASSERT_MSG_EQ (txVector.GetNgvMcs (), 3, "Unexpected NGV-MCS index");
    NS_TEST_ASSERT_MSG_EQ (txVector.GetNgvMidamblePeriodicity (), 16, "Unexpected NGV midamble periodicity");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (txVector.GetNgvLtfType ()),
                           static_cast<uint32_t> (WifiNgvLtfType::NGV_LTF_2X),
                           "Unexpected NGV-LTF type");
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, "Expected valid 802.11bd NGV TXVECTOR metadata");

    WifiTxVector copiedTxVector (txVector);
    NS_TEST_ASSERT_MSG_EQ (copiedTxVector.GetNgvMcs (), 3, "Copy lost NGV-MCS metadata");
    NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (copiedTxVector.GetNgvLtfType ()),
                           static_cast<uint32_t> (WifiNgvLtfType::NGV_LTF_2X),
                           "Copy lost NGV-LTF metadata");

    NS_TEST_ASSERT_MSG_EQ (txVector.GetModulationClass (),
                           WIFI_MOD_CLASS_NGV,
                           "NGV TXVECTOR should use the NGV modulation class");
    NS_TEST_ASSERT_MSG_EQ (WifiPhy::CalculateTxDuration (128, txVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           168000,
                           "Unexpected 802.11bd NGV TXTIME for 128-byte MCS3 PPDU");

    txVector.SetNgvMidamblePeriodicity (4);
    NS_TEST_ASSERT_MSG_EQ (WifiPhy::CalculateTxDuration (2048, txVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           1656000,
                           "Unexpected 802.11bd NGV TXTIME with midambles");
    txVector.SetNgvMidamblePeriodicity (0);

    WifiTxVector nonNgvTxVector (OfdmPhy::GetOfdmRate6MbpsBW10MHz (),
                                 0,
                                 WIFI_PREAMBLE_LONG,
                                 800,
                                 1,
                                 1,
                                 0,
                                 10,
                                 false);
    nonNgvTxVector.SetNgvPpduFormat (WifiNgvPpduFormat::NON_NGV_10);
    NS_TEST_ASSERT_MSG_EQ (nonNgvTxVector.GetModulationClass (),
                           WIFI_MOD_CLASS_OFDM,
                           "NON_NGV_10 TXVECTOR should remain OFDM");

    txVector.SetNgvMcs (15);
    txVector.SetNgvLtfType (WifiNgvLtfType::NGV_LTF_2X);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), false, "NGV-MCS 15 should require NGV_LTF_2X_REPEAT");
    txVector.SetNgvLtfType (WifiNgvLtfType::NGV_LTF_2X_REPEAT);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, "NGV-MCS 15 should allow NGV_LTF_2X_REPEAT");
    txVector.SetNgvMcs (3);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), false, "NGV_LTF_2X_REPEAT should be restricted to NGV-MCS 15");
    txVector.SetNgvLtfType (WifiNgvLtfType::NGV_LTF_2X);

    txVector.SetNgvMcs (10);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), false, "Reserved NGV-MCS should be rejected");

    txVector.SetNgvMcs (3);
    txVector.SetChannelWidth (20);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, "20 MHz NGV channel width should be valid");
    NS_TEST_ASSERT_MSG_EQ (WifiPhy::CalculateTxDuration (128, txVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           120000,
                           "Unexpected 20 MHz 802.11bd NGV TXTIME for 128-byte MCS3 PPDU");
    txVector.SetNgvMcs (9);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, "NGV-MCS 9 should be valid for 20 MHz");

    txVector.SetChannelWidth (10);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), false, "NGV-MCS 9 should be invalid for 10 MHz");
    txVector.SetNgvMcs (3);
    txVector.SetNss (2);
    NS_TEST_ASSERT_MSG_EQ (txVector.IsValid (), true, "NGV NSS=2 should be valid for the supported abstraction");
    txVector.SetChannelWidth (20);
    NS_TEST_ASSERT_MSG_EQ (WifiPhy::CalculateTxDuration (128, txVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           112000,
                           "Unexpected 20 MHz NSS=2 NGV TXTIME for 128-byte MCS3 PPDU");

    WifiTxVector duplicateTxVector (OfdmPhy::GetOfdmRate6MbpsBW10MHz (),
                                    0,
                                    WIFI_PREAMBLE_LONG,
                                    800,
                                    1,
                                    1,
                                    0,
                                    20,
                                    false);
    duplicateTxVector.SetNgvPpduFormat (WifiNgvPpduFormat::NON_NGV_20_DUPLICATE);
    WifiTxVector primaryOnlyTxVector (duplicateTxVector);
    primaryOnlyTxVector.SetNgvPpduFormat (WifiNgvPpduFormat::NON_NGV_10);
    primaryOnlyTxVector.SetChannelWidth (10);
    NS_TEST_ASSERT_MSG_EQ (duplicateTxVector.IsNonNgv20Duplicate (),
                           true,
                           "Expected a 20 MHz duplicated non-NGV PPDU");
    NS_TEST_ASSERT_MSG_EQ (WifiPhy::CalculateTxDuration (128, duplicateTxVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           WifiPhy::CalculateTxDuration (128, primaryOnlyTxVector, WIFI_PHY_BAND_5GHZ).GetNanoSeconds (),
                           "20 MHz non-NGV duplicate PPDU should use primary 10 MHz non-NGV duration");
  }
};

class VehicularWifiActualTxVectorTestCase : public TestCase
{
public:
  VehicularWifiActualTxVectorTestCase ()
    : TestCase ("Validate transmitted 802.11bd TXVECTOR metadata and NON_NGV_10 repetition")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  void NotifyTxPsduBegin (WifiConstPsduMap psduMap, WifiTxVector txVector, double txPowerW)
  {
    m_traceFired = true;
    m_observedPsduCounts.push_back (psduMap.size ());
    m_observedTxVectors.push_back (txVector);
    m_observedTxPowersW.push_back (txPowerW);
    m_observedTxStartTimes.push_back (Simulator::Now ());
    m_observedTxDurations.push_back (
        WifiPhy::CalculateTxDuration (psduMap, txVector, m_tracePhy->GetPhyBand ()));
  }

  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t protocol, const Address& sender)
  {
    m_rxPackets++;
    m_lastProtocol = protocol;
    return true;
  }

  void SendPacket (Ptr<NetDevice> device)
  {
    static const uint16_t wsmpProtocol = 0x88DC;
    m_txSucceeded = device->Send (Create<Packet> (128), Mac48Address::GetBroadcast (), wsmpProtocol);
  }

  void RunOneTransmission (const std::string& scenario,
                           const VehicularWifiProfile& senderProfile,
                           const VehicularWifiProfile& receiverProfile)
  {
    m_traceFired = false;
    m_txSucceeded = false;
    m_rxPackets = 0;
    m_lastProtocol = 0;
    m_tracePhy = nullptr;
    m_observedPsduCounts.clear ();
    m_observedTxVectors.clear ();
    m_observedTxPowersW.clear ();
    m_observedTxStartTimes.clear ();
    m_observedTxDurations.clear ();

    NodeContainer nodes;
    nodes.Create (2);

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
    positions->Add (Vector (0.0, 0.0, 0.0));
    positions->Add (Vector (5.0, 0.0, 0.0));
    MobilityHelper mobility;
    mobility.SetPositionAllocator (positions);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();

    NodeContainer senderNode;
    senderNode.Add (nodes.Get (0));
    NodeContainer receiverNode;
    receiverNode.Add (nodes.Get (1));

    NetDeviceContainer senderDevices = InstallVehicularDevice (senderNode, channel, senderProfile);
    NetDeviceContainer receiverDevices = InstallVehicularDevice (receiverNode, channel, receiverProfile);
    receiverDevices.Get (0)->SetReceiveCallback (
        MakeCallback (&VehicularWifiActualTxVectorTestCase::Receive, this));

    Ptr<WifiNetDevice> senderWifi = DynamicCast<WifiNetDevice> (senderDevices.Get (0));
    NS_TEST_ASSERT_MSG_NE (senderWifi, nullptr, scenario << " expected a Wi-Fi sender device");
    NS_TEST_ASSERT_MSG_EQ (senderWifi->GetPhy ()->GetStandard (),
	                           WIFI_STANDARD_80211bd,
	                           scenario << " sender PHY standard should be 802.11bd");
    m_tracePhy = senderWifi->GetPhy ();

    bool traceConnected = m_tracePhy->TraceConnectWithoutContext (
        "PhyTxPsduBegin",
        MakeCallback (&VehicularWifiActualTxVectorTestCase::NotifyTxPsduBegin, this));
    NS_TEST_ASSERT_MSG_EQ (traceConnected, true, scenario << " failed to connect PhyTxPsduBegin trace");

    Simulator::Schedule (MilliSeconds (100),
                         &VehicularWifiActualTxVectorTestCase::SendPacket,
                         this,
                         senderDevices.Get (0));
    Simulator::Stop (Seconds (1));
    Simulator::Run ();

    NS_TEST_ASSERT_MSG_EQ (m_txSucceeded, true, scenario << " send failed");
    NS_TEST_ASSERT_MSG_EQ (m_traceFired, true, scenario << " did not emit PhyTxPsduBegin");
    NS_TEST_ASSERT_MSG_EQ (m_rxPackets,
                           1u,
                           scenario << " repeated physical PPDUs should deliver one MAC payload");
    NS_TEST_ASSERT_MSG_EQ (m_lastProtocol, 0x88DC, scenario << " received packet with unexpected protocol");

    const std::size_t expectedPpduTraces =
        1 + (senderProfile.GetPpduFormat () == VehicularWifiPpduFormat::NON_NGV_10
                 ? senderProfile.GetNonNgvRepetitions ()
                 : 0);
    NS_TEST_ASSERT_MSG_EQ (m_observedTxVectors.size (),
                           expectedPpduTraces,
                           scenario << " unexpected transmitted PPDU count");
    NS_TEST_ASSERT_MSG_EQ (m_observedTxPowersW.size (),
                           expectedPpduTraces,
                           scenario << " unexpected transmitted power trace count");
    NS_TEST_ASSERT_MSG_EQ (m_observedTxStartTimes.size (),
                           expectedPpduTraces,
                           scenario << " unexpected transmitted timestamp count");
    NS_TEST_ASSERT_MSG_EQ (m_observedTxDurations.size (),
                           expectedPpduTraces,
                           scenario << " unexpected transmitted duration count");

    for (std::size_t i = 0; i < expectedPpduTraces; ++i)
      {
        const WifiTxVector& observedTxVector = m_observedTxVectors.at (i);
        NS_TEST_ASSERT_MSG_EQ (m_observedPsduCounts.at (i), 1u, scenario << " should transmit one PSDU per PPDU");
        NS_TEST_ASSERT_MSG_GT (m_observedTxPowersW.at (i), 0.0, scenario << " should transmit with positive power");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.GetChannelWidth (),
                               senderProfile.GetChannelWidthMHz (),
                               scenario << " lost channel width in transmitted TXVECTOR");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.GetNss (),
                               senderProfile.GetNgvSpatialStreams (),
                               scenario << " lost NSS in transmitted TXVECTOR");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.IsValid (),
                               true,
                               scenario << " transmitted TXVECTOR should be valid");
      }

    for (std::size_t i = 1; i < expectedPpduTraces; ++i)
      {
        const Time observedInterval = m_observedTxStartTimes.at (i) - m_observedTxStartTimes.at (i - 1);
        const Time expectedInterval = m_observedTxDurations.at (i - 1) + m_tracePhy->GetSifs ();
        NS_TEST_ASSERT_MSG_EQ (observedInterval.GetNanoSeconds (),
                               expectedInterval.GetNanoSeconds (),
                               scenario << " repeated NON_NGV_10 PPDU should start after previous PPDU plus SIFS");
      }

    if (senderProfile.GetPpduFormat () == VehicularWifiPpduFormat::NGV)
      {
        const WifiTxVector& observedTxVector = m_observedTxVectors.front ();
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.IsNgv (),
                               true,
                               scenario << " transmitted TXVECTOR should be NGV");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.GetNgvMcs (),
                               senderProfile.GetNgvMcsIndex (),
                               scenario << " lost NGV-MCS in transmitted TXVECTOR");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.IsLdpc (),
                               true,
                               scenario << " NGV transmitted TXVECTOR should request LDPC");
        NS_TEST_ASSERT_MSG_EQ (observedTxVector.GetNgvMidamblePeriodicity (),
                               0,
                               scenario << " unexpected NGV midamble periodicity");
        NS_TEST_ASSERT_MSG_EQ (static_cast<uint32_t> (observedTxVector.GetNgvLtfType ()),
                               static_cast<uint32_t> (WifiNgvLtfType::NGV_LTF_2X),
                               scenario << " unexpected NGV-LTF type");
      }
    else
      {
        for (const auto& observedTxVector : m_observedTxVectors)
          {
            NS_TEST_ASSERT_MSG_EQ ((senderProfile.GetPpduFormat () == VehicularWifiPpduFormat::NON_NGV_20_DUPLICATE)
                                       ? observedTxVector.IsNonNgv20Duplicate ()
                                       : observedTxVector.IsNonNgv10 (),
                                   true,
                                   scenario << " transmitted TXVECTOR carried unexpected non-NGV format");
            NS_TEST_ASSERT_MSG_EQ (observedTxVector.GetNgvPpduRepetitions (),
                                   senderProfile.GetNonNgvRepetitions (),
                                   scenario << " lost N_PPDU_REP in transmitted TXVECTOR");
            NS_TEST_ASSERT_MSG_EQ (observedTxVector.IsLdpc (),
                                   false,
                                   scenario << " NON_NGV_10 transmitted TXVECTOR should not request LDPC");
          }
      }

    m_tracePhy = nullptr;
    Simulator::Destroy ();
  }

  void DoRun () override
  {
    RunOneTransmission ("802.11bd NGV",
                        VehicularWifiProfile::Ieee80211bd (),
                        VehicularWifiProfile::Ieee80211bd ());
    RunOneTransmission ("802.11bd 20 MHz NGV",
                        VehicularWifiProfile::Ieee80211bd20 (),
                        VehicularWifiProfile::Ieee80211bd20 ());
    RunOneTransmission ("802.11bd NGV NSS=2",
                        VehicularWifiProfile::Ieee80211bd ("OfdmRate12MbpsBW10MHz",
                                                           23.0,
                                                           -92.0,
                                                           4.0,
                                                           3,
                                                           2),
                        VehicularWifiProfile::Ieee80211bd ("OfdmRate12MbpsBW10MHz",
                                                           23.0,
                                                           -92.0,
                                                           4.0,
                                                           3,
                                                           2));
    RunOneTransmission ("802.11bd 20 MHz NGV NSS=2",
                        VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps",
                                                             23.0,
                                                             -92.0,
                                                             4.0,
                                                             3,
                                                             2),
                        VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps",
                                                             23.0,
                                                             -92.0,
                                                             4.0,
                                                             3,
                                                             2));
    RunOneTransmission ("802.11bd NON_NGV_10",
                        VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz",
                                                                    23.0,
                                                                    -93.0,
                                                                    4.0,
                                                                    2),
                        VehicularWifiProfile::Ieee80211p ());
    RunOneTransmission ("802.11bd NON_NGV_20_DUPLICATE to 802.11bd",
                        VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate (),
                        VehicularWifiProfile::Ieee80211bd20 ());
    RunOneTransmission ("802.11bd NON_NGV_20_DUPLICATE to 802.11p",
                        VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate (),
                        VehicularWifiProfile::Ieee80211p ());
  }

  bool m_traceFired = false;
  bool m_txSucceeded = false;
  uint32_t m_rxPackets = 0;
  uint16_t m_lastProtocol = 0;
  Ptr<WifiPhy> m_tracePhy;
  std::vector<std::size_t> m_observedPsduCounts;
  std::vector<double> m_observedTxPowersW;
  std::vector<Time> m_observedTxStartTimes;
  std::vector<Time> m_observedTxDurations;
  std::vector<WifiTxVector> m_observedTxVectors;
};

class VehicularWifiInteroperabilityTestCase : public TestCase
{
public:
  VehicularWifiInteroperabilityTestCase ()
    : TestCase ("Validate 802.11p/802.11bd interoperability and NGV isolation")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t protocol, const Address& sender)
  {
    m_rxPackets++;
    m_lastProtocol = protocol;
    return true;
  }

  void SendPacket (Ptr<NetDevice> device)
  {
    static const uint16_t wsmpProtocol = 0x88DC;
    m_txSucceeded = device->Send (Create<Packet> (128), Mac48Address::GetBroadcast (), wsmpProtocol);
  }

  void RunOneWay (const std::string& scenario,
                  const VehicularWifiProfile& senderProfile,
                  const VehicularWifiProfile& receiverProfile,
                  uint32_t expectedRxPackets)
  {
    m_rxPackets = 0;
    m_lastProtocol = 0;
    m_txSucceeded = false;

    NodeContainer nodes;
    nodes.Create (2);

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
    positions->Add (Vector (0.0, 0.0, 0.0));
    positions->Add (Vector (5.0, 0.0, 0.0));
    MobilityHelper mobility;
    mobility.SetPositionAllocator (positions);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();

    NodeContainer senderNode;
    senderNode.Add (nodes.Get (0));
    NodeContainer receiverNode;
    receiverNode.Add (nodes.Get (1));

    NetDeviceContainer senderDevices = InstallVehicularDevice (senderNode, channel, senderProfile);
    NetDeviceContainer receiverDevices = InstallVehicularDevice (receiverNode, channel, receiverProfile);

    receiverDevices.Get (0)->SetReceiveCallback (
        MakeCallback (&VehicularWifiInteroperabilityTestCase::Receive, this));

    Simulator::Schedule (MilliSeconds (100),
                         &VehicularWifiInteroperabilityTestCase::SendPacket,
                         this,
                         senderDevices.Get (0));
    Simulator::Stop (Seconds (1));
    Simulator::Run ();

    NS_TEST_ASSERT_MSG_EQ (m_txSucceeded, true, scenario << " send failed");
    NS_TEST_ASSERT_MSG_EQ (m_rxPackets,
                           expectedRxPackets,
                           scenario << " received unexpected packet count");
    if (expectedRxPackets > 0)
      {
        NS_TEST_ASSERT_MSG_EQ (m_lastProtocol, 0x88DC, scenario << " received packet with unexpected protocol");
      }

    Simulator::Destroy ();
  }

  void DoRun () override
  {
    RunOneWay ("802.11p to 802.11bd",
               VehicularWifiProfile::Ieee80211p (),
               VehicularWifiProfile::Ieee80211bd (),
               1);
    RunOneWay ("802.11bd NON_NGV_10 to 802.11p",
               VehicularWifiProfile::Ieee80211bdNonNgv10 (),
               VehicularWifiProfile::Ieee80211p (),
               1);
    RunOneWay ("802.11bd NGV to 802.11p",
               VehicularWifiProfile::Ieee80211bd (),
               VehicularWifiProfile::Ieee80211p (),
               0);
  }

  uint32_t m_rxPackets = 0;
  uint16_t m_lastProtocol = 0;
  bool m_txSucceeded = false;
};

class VehicularWifiNgvErrorRateTestCase : public TestCase
{
public:
  VehicularWifiNgvErrorRateTestCase ()
    : TestCase ("Validate 802.11bd NGV error-rate model")
  {
  }

private:
  WifiTxVector MakeNgvTxVector (uint8_t mcs,
                                uint16_t channelWidthMHz = 10,
                                uint8_t nss = 1,
                                uint16_t midamblePeriodicity = 0) const
  {
    WifiTxVector txVector (channelWidthMHz == 20 ? OfdmPhy::GetOfdmRate24Mbps ()
                                                 : OfdmPhy::GetOfdmRate12MbpsBW10MHz (),
                           0,
                           WIFI_PREAMBLE_LONG,
                           800,
                           nss,
                           nss,
                           0,
                           channelWidthMHz,
                           false,
                           false,
                           true);
    txVector.SetNgvPpduFormat (WifiNgvPpduFormat::NGV);
    txVector.SetNgvMcs (mcs);
    txVector.SetNgvMidamblePeriodicity (midamblePeriodicity);
    txVector.SetNgvLtfType ((channelWidthMHz == 10 && nss == 1 && mcs == 15)
                                ? WifiNgvLtfType::NGV_LTF_2X_REPEAT
                                : WifiNgvLtfType::NGV_LTF_2X);
    return txVector;
  }

  double GetNgvPer (Ptr<ErrorRateModel> errorModel,
                    uint8_t mcs,
                    uint16_t channelWidthMHz,
                    uint8_t nss,
                    double snrDb,
                    uint32_t bytes,
                    uint16_t midamblePeriodicity = 0) const
  {
    WifiTxVector txVector = MakeNgvTxVector (mcs, channelWidthMHz, nss, midamblePeriodicity);
    const double snr = std::pow (10.0, snrDb / 10.0);
    const double psr = errorModel->GetChunkSuccessRate (txVector.GetMode (),
                                                        txVector,
                                                        snr,
                                                        bytes * 8,
                                                        1,
                                                        WIFI_PPDU_FIELD_DATA);
    return 1.0 - psr;
  }

  void DoRun () override
  {
    Ptr<ErrorRateModel> errorModel = CreateObject<YansErrorRateModel> ();

    for (const auto& point : NgvCalibration::PER_REFERENCES)
      {
        const double below = GetNgvPer (errorModel,
                                        point.mcs,
                                        point.channelWidthMHz,
                                        point.nss,
                                        point.snr10PercentPerDb - 6.0,
                                        point.referenceBytes);
        const double atTarget = GetNgvPer (errorModel,
                                           point.mcs,
                                           point.channelWidthMHz,
                                           point.nss,
                                           point.snr10PercentPerDb,
                                           point.referenceBytes);
        const double above = GetNgvPer (errorModel,
                                        point.mcs,
                                        point.channelWidthMHz,
                                        point.nss,
                                        point.snr10PercentPerDb + 6.0,
                                        point.referenceBytes);

        NS_TEST_ASSERT_MSG_GT (below,
                               atTarget,
                               "NGV-MCS " << static_cast<uint32_t> (point.mcs)
                                          << " " << point.channelWidthMHz
                                          << " MHz NSS=" << +point.nss
                                          << " PER should decrease as SNR increases");
        NS_TEST_ASSERT_MSG_GT (atTarget,
                               above,
                               "NGV-MCS " << static_cast<uint32_t> (point.mcs)
                                          << " " << point.channelWidthMHz
                                          << " MHz NSS=" << +point.nss
                                          << " PER should keep decreasing above the target SNR");
        NS_TEST_ASSERT_MSG_EQ_TOL (atTarget,
                                   0.1,
                                   0.000001,
                                   "NGV-MCS " << static_cast<uint32_t> (point.mcs)
                                              << " " << point.channelWidthMHz
                                              << " MHz NSS=" << +point.nss
                                              << " target PER calibration drifted");
      }

    NS_TEST_ASSERT_MSG_LT (GetNgvPer (errorModel, 15, 10, 1, 12.0, 2048),
                           GetNgvPer (errorModel, 0, 10, 1, 12.0, 4096),
                           "NGV-MCS 15 BPSK-DCM should be more robust than NGV-MCS 0 at 12 dB");
    NS_TEST_ASSERT_MSG_LT (GetNgvPer (errorModel, 3, 10, 1, 26.0, 4096),
                           GetNgvPer (errorModel, 8, 10, 1, 26.0, 4096),
                           "NGV-MCS 3 should be more robust than 256-QAM NGV-MCS 8 at 26 dB");
    NS_TEST_ASSERT_MSG_LT (GetNgvPer (errorModel, 8, 20, 1, 37.0, 4096),
                           GetNgvPer (errorModel, 9, 20, 1, 37.0, 4096),
                           "20 MHz NGV-MCS 8 should be more robust than NGV-MCS 9 at 37 dB");

    const double longNoMidamble = GetNgvPer (errorModel, 3, 10, 1, 20.0, 4096, 0);
    const double longMidamble16 = GetNgvPer (errorModel, 3, 10, 1, 20.0, 4096, 16);
    const double longMidamble4 = GetNgvPer (errorModel, 3, 10, 1, 20.0, 4096, 4);
    NS_TEST_ASSERT_MSG_LT (longMidamble16,
                           longNoMidamble,
                           "NGV midambles should improve high-mobility PER for long PPDUs");
    NS_TEST_ASSERT_MSG_LT (longMidamble4,
                           longMidamble16,
                           "Shorter NGV midamble periodicity should give stronger Doppler tracking gain");

    const double shortNoMidamble = GetNgvPer (errorModel, 3, 10, 1, 20.0, 128, 0);
    const double shortMidamble4 = GetNgvPer (errorModel, 3, 10, 1, 20.0, 128, 4);
    NS_TEST_ASSERT_MSG_EQ_TOL (shortMidamble4,
                               shortNoMidamble,
                               0.000000000001,
                               "Short NGV PPDUs should not receive midamble/Doppler gain");
  }
};

class VehicularWifiNonNgv10CombiningTestCase : public TestCase
{
public:
  VehicularWifiNonNgv10CombiningTestCase ()
    : TestCase ("Validate 802.11bd NON_NGV_10 repetition combining")
  {
  }

private:
  WifiTxVector MakeNonNgv10TxVector (uint8_t repetitions) const
  {
    WifiTxVector txVector (OfdmPhy::GetOfdmRate6MbpsBW10MHz (),
                           0,
                           WIFI_PREAMBLE_LONG,
                           800,
                           1,
                           1,
                           0,
                           10,
                           false);
    txVector.SetNgvPpduFormat (WifiNgvPpduFormat::NON_NGV_10);
    txVector.SetNgvPpduRepetitions (repetitions);
    return txVector;
  }

  double GetNonNgv10Per (uint8_t repetitions, WifiPpduField field) const
  {
    LinearSnrErrorRateModel errorModel;
    WifiTxVector txVector = MakeNonNgv10TxVector (repetitions);
    const double psr = errorModel.GetChunkSuccessRate (txVector.GetMode (),
                                                       txVector,
                                                       0.2,
                                                       1024 * 8,
                                                       1,
                                                       field);
    return 1.0 - psr;
  }

  void DoRun () override
  {
    const double payloadNoCombining = GetNonNgv10Per (0, WIFI_PPDU_FIELD_DATA);
    const double payloadWithCombining = GetNonNgv10Per (2, WIFI_PPDU_FIELD_DATA);
    NS_TEST_ASSERT_MSG_LT (payloadWithCombining,
                           payloadNoCombining,
                           "NON_NGV_10 repeated payload should benefit from repetition combining");

    const double headerNoCombining = GetNonNgv10Per (0, WIFI_PPDU_FIELD_NON_HT_HEADER);
    const double headerWithRepeats = GetNonNgv10Per (2, WIFI_PPDU_FIELD_NON_HT_HEADER);
    NS_TEST_ASSERT_MSG_EQ_TOL (headerWithRepeats,
                               headerNoCombining,
                               0.000000000001,
                               "NON_NGV_10 repetition combining should not alter header decoding");
  }
};

class VehicularWifiMarginalSnrGainTestCase : public TestCase
{
public:
  VehicularWifiMarginalSnrGainTestCase ()
    : TestCase ("Validate 802.11bd marginal-SNR reliability gains from installed profiles")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  WifiMacHeader MakeBroadcastDataHeader () const
  {
    WifiMacHeader broadcastData;
    broadcastData.SetType (WIFI_MAC_DATA);
    broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
    broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
    broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());
    return broadcastData;
  }

  WifiTxVector GetDataTxVector (const VehicularWifiProfile& profile)
  {
    NodeContainer nodes;
    nodes.Create (1);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();
    NetDeviceContainer devices = InstallVehicularDevice (nodes, channel, profile);
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_ABORT_MSG_IF (device == nullptr, "Expected an installed Wi-Fi device");

    WifiTxVector txVector =
        device->GetRemoteStationManager ()->GetDataTxVector (MakeBroadcastDataHeader ());
    Simulator::Destroy ();
    return txVector;
  }

  WifiTxVector GetDataTxVectorWithMidamble (uint16_t midamblePeriodicity)
  {
    NodeContainer nodes;
    nodes.Create (1);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();
    NetDeviceContainer devices =
        InstallVehicularDevice (nodes, channel, VehicularWifiProfile::Ieee80211bd ());
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_ABORT_MSG_IF (device == nullptr, "Expected an installed 802.11bd Wi-Fi device");

    device->GetRemoteStationManager ()->SetAttribute ("NgvMidamblePeriodicity",
                                                      UintegerValue (midamblePeriodicity));
    WifiTxVector txVector =
        device->GetRemoteStationManager ()->GetDataTxVector (MakeBroadcastDataHeader ());
    Simulator::Destroy ();
    return txVector;
  }

  double GetPer (Ptr<ErrorRateModel> errorModel,
                 const WifiTxVector& txVector,
                 double snrDb,
                 uint32_t bytes,
                 WifiPpduField field = WIFI_PPDU_FIELD_DATA) const
  {
    const double snr = std::pow (10.0, snrDb / 10.0);
    const double psr = errorModel->GetChunkSuccessRate (txVector.GetMode (),
                                                        txVector,
                                                        snr,
                                                        bytes * 8,
                                                        1,
                                                        field);
    return 1.0 - psr;
  }

  void DoRun () override
  {
    Ptr<ErrorRateModel> yansErrorModel = CreateObject<YansErrorRateModel> ();

    const WifiTxVector ngvNoMidamble = GetDataTxVector (VehicularWifiProfile::Ieee80211bd ());
    const WifiTxVector ngvMidamble4 = GetDataTxVectorWithMidamble (4);
    NS_TEST_ASSERT_MSG_EQ (ngvNoMidamble.IsNgv (), true, "Expected installed 802.11bd profile to use native NGV");
    NS_TEST_ASSERT_MSG_EQ (ngvNoMidamble.GetNgvMidamblePeriodicity (),
                           0,
                           "Default installed 802.11bd profile should keep midambles disabled");
    NS_TEST_ASSERT_MSG_EQ (ngvMidamble4.GetNgvMidamblePeriodicity (),
                           4,
                           "Installed 802.11bd manager did not apply configured midamble periodicity");
    NS_TEST_ASSERT_MSG_LT (GetPer (yansErrorModel, ngvMidamble4, 20.0, 4096),
                           GetPer (yansErrorModel, ngvNoMidamble, 20.0, 4096),
                           "Installed 802.11bd NGV midambles should improve long-PPDU PER at marginal SNR");

    Ptr<ErrorRateModel> linearErrorModel = CreateObject<LinearSnrErrorRateModel> ();
    const WifiTxVector nonNgvNoRepeats =
        GetDataTxVector (VehicularWifiProfile::Ieee80211bdNonNgv10 ());
    const WifiTxVector nonNgvRepeated =
        GetDataTxVector (VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz",
                                                                    23.0,
                                                                    -93.0,
                                                                    4.0,
                                                                    2));
    NS_TEST_ASSERT_MSG_EQ (nonNgvNoRepeats.IsNonNgv10 (),
                           true,
                           "Expected compatible 802.11bd profile to use NON_NGV_10");
    NS_TEST_ASSERT_MSG_EQ (nonNgvRepeated.GetNgvPpduRepetitions (),
                           2,
                           "Installed 802.11bd manager did not apply NON_NGV_10 repetitions");
    NS_TEST_ASSERT_MSG_LT (GetPer (linearErrorModel, nonNgvRepeated, -6.9897, 1024),
                           GetPer (linearErrorModel, nonNgvNoRepeats, -6.9897, 1024),
                           "Installed repeated NON_NGV_10 profile should improve Data-field PER at marginal SNR");
    NS_TEST_ASSERT_MSG_EQ_TOL (
        GetPer (linearErrorModel, nonNgvRepeated, -6.9897, 1024, WIFI_PPDU_FIELD_NON_HT_HEADER),
        GetPer (linearErrorModel, nonNgvNoRepeats, -6.9897, 1024, WIFI_PPDU_FIELD_NON_HT_HEADER),
        0.000000000001,
        "Installed repeated NON_NGV_10 profile should not change non-HT header PER");
  }
};

class VehicularWifiDccAirtimeTestCase : public TestCase
{
public:
  VehicularWifiDccAirtimeTestCase ()
    : TestCase ("Validate 802.11bd DCC uses PHY airtime")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  WifiMacHeader MakeBroadcastDataHeader () const
  {
    WifiMacHeader broadcastData;
    broadcastData.SetType (WIFI_MAC_DATA);
    broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
    broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
    broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());
    return broadcastData;
  }

  Time GetExpectedDccAirtime (Ptr<WifiNetDevice> device, uint32_t psduSize) const
  {
    WifiTxVector txVector = device->GetRemoteStationManager ()->GetDataTxVector (MakeBroadcastDataHeader ());
    Time txDuration = WifiPhy::CalculateTxDuration (psduSize, txVector, device->GetPhy ()->GetPhyBand ());
    if (txVector.IsNonNgv10 () && txVector.GetNgvPpduRepetitions () > 0)
      {
        const uint8_t repetitions = txVector.GetNgvPpduRepetitions ();
        txDuration = txDuration * (1 + repetitions) + device->GetPhy ()->GetSifs () * repetitions;
      }
    return txDuration;
  }

  void CheckProfile (const std::string& scenario,
                     const VehicularWifiProfile& profile,
                     bool expectDifferentFromNominalBitrate)
  {
    constexpr uint32_t psduSize = 128;

    NodeContainer nodes;
    nodes.Create (1);
    MobilityHelper mobility;
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();
    NetDeviceContainer devices = InstallVehicularDevice (nodes, channel, profile);
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice> (devices.Get (0));
    NS_TEST_ASSERT_MSG_NE (device, nullptr, scenario << " expected a Wi-Fi device");

    Ptr<DCC> dcc = CreateObject<DCC> ();
    Ptr<MetricSupervisor> metricSupervisor = CreateObject<MetricSupervisor> ();
    dcc->SetupDCC ("0", metricSupervisor, nodes.Get (0), "adaptive", 200);
    dcc->setBitRate (static_cast<long> (profile.GetDccBitRate ()));
    dcc->updateTonpp (psduSize);

    const double expectedMs = GetExpectedDccAirtime (device, psduSize).GetSeconds () * 1000.0;
    NS_TEST_ASSERT_MSG_EQ_TOL (dcc->getTonPpMs (),
                               expectedMs,
                               0.00001,
                               scenario << " DCC Ton_pp should match PHY airtime");

    if (expectDifferentFromNominalBitrate)
      {
        const double nominalMs = ((psduSize * 8.0) / profile.GetDccBitRate () + 68e-6) * 1000.0;
        NS_TEST_ASSERT_MSG_GT (std::abs (dcc->getTonPpMs () - nominalMs),
                               0.001,
                               scenario << " DCC Ton_pp should not use the legacy nominal bitrate formula");
      }

    Simulator::Destroy ();
  }

  void DoRun () override
  {
    CheckProfile ("802.11bd NGV", VehicularWifiProfile::Ieee80211bd (), true);
    CheckProfile ("802.11bd 20 MHz NGV", VehicularWifiProfile::Ieee80211bd20 (), true);
    CheckProfile ("802.11bd NON_NGV_10",
                  VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz",
                                                              23.0,
                                                              -93.0,
                                                              4.0,
                                                              2),
                  true);
    CheckProfile ("802.11bd NON_NGV_20_DUPLICATE",
                  VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate (),
                  true);
  }
};

class VehicularWifiCbrOracleTestCase : public TestCase
{
public:
  VehicularWifiCbrOracleTestCase ()
    : TestCase ("Validate 802.11bd CBR matches PHY busy-time oracle")
  {
  }

private:
  NetDeviceContainer InstallVehicularDevice (NodeContainer nodes,
                                             Ptr<YansWifiChannel> channel,
                                             const VehicularWifiProfile& profile)
  {
    YansWifiPhyHelper phy;
    profile.ConfigurePhy (phy);
    phy.SetChannel (channel);

    QosWaveMacHelper mac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
    profile.ConfigureRemoteStationManager (wifi);
    return wifi.Install (phy, mac, nodes);
  }

  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t protocol, const Address& sender)
  {
    m_rxPackets++;
    return true;
  }

  void SendPacket (Ptr<NetDevice> device)
  {
    static const uint16_t wsmpProtocol = 0x88DC;
    m_txAttempts++;
    if (device->Send (Create<Packet> (128), Mac48Address::GetBroadcast (), wsmpProtocol))
      {
        m_txSucceededPackets++;
      }
  }

  static bool IsBusyForOracle (WifiPhyState state)
  {
    return state != WifiPhyState::SLEEP && state != WifiPhyState::IDLE;
  }

  void NotifyState (std::string context, Time start, Time duration, WifiPhyState state)
  {
    if (!IsBusyForOracle (state))
      {
        return;
      }

    std::size_t first = context.find ("/NodeList/");
    if (first == std::string::npos)
      {
        m_oracleTraceContextError = true;
        return;
      }
    first += std::string ("/NodeList/").size ();
    std::size_t last = context.find ("/", first);
    if (last == std::string::npos)
      {
        m_oracleTraceContextError = true;
        return;
      }
    const std::string nodeId = context.substr (first, last - first);

    const Time end = start + duration;
    if (end <= m_windowStart || start >= m_windowEnd)
      {
        return;
      }

    const Time clippedStart = start < m_windowStart ? m_windowStart : start;
    const Time clippedEnd = end > m_windowEnd ? m_windowEnd : end;
    const Time clippedDuration = clippedEnd - clippedStart;
    if (clippedDuration.IsStrictlyPositive ())
      {
        m_oracleBusy[nodeId] += clippedDuration;
      }
  }

  void RunScenario (const std::string& scenario,
                    const std::vector<Time>& transmissionTimes,
                    const std::set<uint32_t>& comparedNodeIndexes)
  {
    m_txAttempts = 0;
    m_txSucceededPackets = 0;
    m_rxPackets = 0;
    m_oracleTraceContextError = false;
    m_oracleBusy.clear ();

    NodeContainer nodes;
    nodes.Create (2);

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
    positions->Add (Vector (0.0, 0.0, 0.0));
    positions->Add (Vector (5.0, 0.0, 0.0));
    MobilityHelper mobility;
    mobility.SetPositionAllocator (positions);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = channelHelper.Create ();

    NodeContainer senderNode;
    senderNode.Add (nodes.Get (0));
    NodeContainer receiverNode;
    receiverNode.Add (nodes.Get (1));

    NetDeviceContainer senderDevices =
        InstallVehicularDevice (senderNode, channel, VehicularWifiProfile::Ieee80211bd ());
    NetDeviceContainer receiverDevices =
        InstallVehicularDevice (receiverNode, channel, VehicularWifiProfile::Ieee80211bd ());
    receiverDevices.Get (0)->SetReceiveCallback (
        MakeCallback (&VehicularWifiCbrOracleTestCase::Receive, this));

    for (uint32_t i = 0; i < nodes.GetN (); ++i)
      {
        std::ostringstream oss;
        oss << "/NodeList/" << nodes.Get (i)->GetId ()
            << "/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State";
        Config::Connect (oss.str (),
                         MakeCallback (&VehicularWifiCbrOracleTestCase::NotifyState, this));
      }

    Ptr<MetricSupervisor> metricSupervisor = CreateObject<MetricSupervisor> ();
    metricSupervisor->setNodeContainer (nodes);
    metricSupervisor->setChannelTechnology ("80211bd");
    metricSupervisor->setCBRWindowValue (m_cbrWindowMs);
    metricSupervisor->setCBRAlphaValue (0.0);
    metricSupervisor->setSimulationTimeValue (0.02);
    metricSupervisor->startCheckCBR ();

    for (const Time& transmissionTime : transmissionTimes)
      {
        Simulator::Schedule (transmissionTime,
                             &VehicularWifiCbrOracleTestCase::SendPacket,
                             this,
                             senderDevices.Get (0));
      }
    Simulator::Stop (MilliSeconds (15));
    Simulator::Run ();

    NS_TEST_ASSERT_MSG_EQ (m_txAttempts,
                           transmissionTimes.size (),
                           scenario << " attempted an unexpected number of transmissions");
    NS_TEST_ASSERT_MSG_EQ (m_txSucceededPackets,
                           transmissionTimes.size (),
                           scenario << " send failed");
    NS_TEST_ASSERT_MSG_EQ (m_rxPackets,
                           transmissionTimes.size (),
                           scenario << " did not receive every broadcast");
    NS_TEST_ASSERT_MSG_EQ (m_oracleTraceContextError, false, scenario << " failed to parse CBR oracle trace context");

    for (uint32_t nodeIndex : comparedNodeIndexes)
      {
        const std::string nodeId = std::to_string (nodes.Get (nodeIndex)->GetId ());
        const auto oracleIt = m_oracleBusy.find (nodeId);
        if (oracleIt == m_oracleBusy.end ())
          {
            NS_TEST_ASSERT_MSG_EQ (false,
                                   true,
                                   scenario << " 802.11bd CBR oracle did not observe busy time for node "
                                       << nodeId);
            continue;
          }

        const double expectedCbr = oracleIt->second.GetDouble () / (m_cbrWindowMs * 1e6);
        const double observedCbr = metricSupervisor->getCBRPerItem (nodeId);
        NS_TEST_ASSERT_MSG_GT (expectedCbr,
                               0.0,
                               scenario << " expected non-zero busy time for node " << nodeId);
        NS_TEST_ASSERT_MSG_EQ_TOL (observedCbr,
                                   expectedCbr,
                                   1e-12,
                                   scenario << " CBR should match PHY busy-time oracle for node " << nodeId);
      }

    Simulator::Destroy ();
  }

  void DoRun () override
  {
    RunScenario ("802.11bd CBR in-window busy interval", {MilliSeconds (1)}, {0, 1});
    RunScenario ("802.11bd CBR window-clipped sender busy interval", {MicroSeconds (9950)}, {0});
  }

  const Time m_windowStart = Seconds (0);
  const Time m_windowEnd = MilliSeconds (10);
  const double m_cbrWindowMs = 10.0;
  bool m_oracleTraceContextError = false;
  std::size_t m_txAttempts = 0;
  std::size_t m_txSucceededPackets = 0;
  uint32_t m_rxPackets = 0;
  std::unordered_map<std::string, Time> m_oracleBusy;
};

class VehicularGeoNetGlobalCbrTestCase : public TestCase
{
public:
  VehicularGeoNetGlobalCbrTestCase ()
    : TestCase ("Validate GeoNet global CBR uses simulation-time expiry")
  {
  }

private:
  void DoRun () override
  {
    GeoNet::LocationTableExtension firstNeighbor;
    firstNeighbor.CBR_R0_Hop.push_back (std::make_tuple (0, 0.95));
    firstNeighbor.CBR_R0_Hop.push_back (std::make_tuple (600, 0.40));
    firstNeighbor.CBR_R0_Hop.push_back (std::make_tuple (1500, 0.70));
    firstNeighbor.CBR_R1_Hop.push_back (std::make_tuple (0, 0.90));
    firstNeighbor.CBR_R1_Hop.push_back (std::make_tuple (700, 0.30));
    firstNeighbor.CBR_R1_Hop.push_back (std::make_tuple (900, 0.45));

    GeoNet::LocationTableExtension secondNeighbor;
    secondNeighbor.CBR_R0_Hop.push_back (std::make_tuple (700, 0.80));

    std::vector<GeoNet::LocationTableExtension*> extensions = {&firstNeighbor, &secondNeighbor};

    GeoNet::GlobalCbrAggregation aggregation =
        GeoNet::ComputeGlobalCbrAggregation (extensions, 1600, 1000, 0.50, 0.25);

    NS_TEST_ASSERT_MSG_EQ (aggregation.r0SampleCount,
                           3u,
                           "GeoNet CBR_G should keep R0 samples within the simulation-time window");
    NS_TEST_ASSERT_MSG_EQ (aggregation.r1SampleCount,
                           2u,
                           "GeoNet CBR_G should keep R1 samples within the simulation-time window");
    NS_TEST_ASSERT_MSG_EQ (firstNeighbor.CBR_R0_Hop.size (),
                           2u,
                           "GeoNet CBR_G should prune expired R0 samples");
    NS_TEST_ASSERT_MSG_EQ (std::get<0> (firstNeighbor.CBR_R0_Hop.front ()),
                           600,
                           "GeoNet CBR_G should keep samples exactly on the expiry boundary");
    NS_TEST_ASSERT_MSG_EQ_TOL (aggregation.cbrL1Hop,
                               0.80,
                               1e-12,
                               "GeoNet CBR_G should select max R0 when mean R0 exceeds target");
    NS_TEST_ASSERT_MSG_EQ_TOL (aggregation.cbrL2Hop,
                               0.30,
                               1e-12,
                               "GeoNet CBR_G should select second max R1 when mean R1 does not exceed target");
    NS_TEST_ASSERT_MSG_EQ_TOL (aggregation.cbrG,
                               0.80,
                               1e-12,
                               "GeoNet CBR_G should aggregate L0 previous, L1, and L2");

    aggregation = GeoNet::ComputeGlobalCbrAggregation (extensions, 2700, 1000, 0.50, 0.25);

    NS_TEST_ASSERT_MSG_EQ (aggregation.r0SampleCount,
                           0u,
                           "GeoNet CBR_G should expire R0 samples using simulation time");
    NS_TEST_ASSERT_MSG_EQ (aggregation.r1SampleCount,
                           0u,
                           "GeoNet CBR_G should expire R1 samples using simulation time");
    NS_TEST_ASSERT_MSG_EQ_TOL (aggregation.cbrG,
                               0.25,
                               1e-12,
                               "GeoNet CBR_G should fall back to previous local CBR when remote samples expire");
  }
};

class VehicularWifiProfileTestSuite : public TestSuite
{
public:
  VehicularWifiProfileTestSuite ()
    : TestSuite ("vehicular-wifi-profile", UNIT)
  {
    AddTestCase (new VehicularNgvMcsTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiProfileTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiProfileInstallTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiFallbackControllerTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiRateControllerTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiCore11bdTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiActualTxVectorTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiInteroperabilityTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiNgvErrorRateTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiNonNgv10CombiningTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiMarginalSnrGainTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiDccAirtimeTestCase, TestCase::QUICK);
    AddTestCase (new VehicularWifiCbrOracleTestCase, TestCase::QUICK);
    AddTestCase (new VehicularGeoNetGlobalCbrTestCase, TestCase::QUICK);
  }
};

static VehicularWifiProfileTestSuite vehicularWifiProfileTestSuite;
