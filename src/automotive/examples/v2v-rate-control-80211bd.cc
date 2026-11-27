/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/abort.h"
#include "ns3/command-line.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/vector.h"
#include "ns3/vehicular-wifi-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/yans-wifi-helper.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

static constexpr uint16_t RATE_CONTROL_PROTOCOL = 0x88D7;

struct CaseConfig
{
  std::string label;
  VehicularWifiRateController::Policy policy;
};

struct CaseResult
{
  std::string label;
  VehicularWifiRateController::Decision decision;
  uint32_t txAttempts = 0;
  uint32_t txSucceeded = 0;
  uint32_t rxPackets = 0;
  uint32_t ngvPpdus = 0;
  uint32_t mcs0Ppdus = 0;
  uint32_t mcs3Ppdus = 0;
  uint32_t mcs7Ppdus = 0;
  uint32_t otherMcsPpdus = 0;
  uint32_t duplicatePpdus = 0;
  uint32_t nonNgv10Ppdus = 0;
};

NetDeviceContainer
InstallVehicularDevice (NodeContainer nodes,
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

VehicularWifiProfile
MakeReceiverProfile (const VehicularWifiRateController::Decision& decision,
                     bool legacyReceiverPresent)
{
  if (legacyReceiverPresent)
    {
      return VehicularWifiProfile::Ieee80211p ();
    }
  if (decision.fallbackMode == VehicularWifiFallbackMode::NGV_20 ||
      decision.fallbackMode == VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE)
    {
      return VehicularWifiProfile::Ieee80211bd20 ();
    }
  return VehicularWifiProfile::Ieee80211bdNonNgv10 ();
}

bool
ReceivePacket (CaseResult* result,
               Ptr<NetDevice> dev,
               Ptr<const Packet> pkt,
               uint16_t protocol,
               const Address& sender)
{
  if (protocol == RATE_CONTROL_PROTOCOL)
    {
      result->rxPackets++;
    }
  return true;
}

void
NotifyTxPsduBegin (CaseResult* result,
                   WifiConstPsduMap psduMap,
                   WifiTxVector txVector,
                   double txPowerW)
{
  if (txVector.IsNgv ())
    {
      result->ngvPpdus++;
      if (txVector.GetNgvMcs () == 0)
        {
          result->mcs0Ppdus++;
        }
      else if (txVector.GetNgvMcs () == 3)
        {
          result->mcs3Ppdus++;
        }
      else if (txVector.GetNgvMcs () == 7)
        {
          result->mcs7Ppdus++;
        }
      else
        {
          result->otherMcsPpdus++;
        }
    }
  else if (txVector.IsNonNgv20Duplicate ())
    {
      result->duplicatePpdus++;
    }
  else if (txVector.IsNonNgv10 ())
    {
      result->nonNgv10Ppdus++;
    }
}

void
SendPacket (CaseResult* result, Ptr<NetDevice> sender, uint32_t payloadBytes)
{
  result->txAttempts++;
  if (sender->Send (Create<Packet> (payloadBytes),
                    Mac48Address::GetBroadcast (),
                    RATE_CONTROL_PROTOCOL))
    {
      result->txSucceeded++;
    }
}

CaseResult
RunCase (const CaseConfig& config, uint32_t payloadBytes, double distanceMeters)
{
  CaseResult result;
  result.label = config.label;
  result.decision = VehicularWifiRateController::Select (config.policy);

  NodeContainer nodes;
  nodes.Create (2);

  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0));
  positions->Add (Vector (distanceMeters, 0.0, 0.0));
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
      InstallVehicularDevice (senderNode,
                              channel,
                              VehicularWifiRateController::MakeProfile (result.decision));
  NetDeviceContainer receiverDevices =
      InstallVehicularDevice (receiverNode,
                              channel,
                              MakeReceiverProfile (result.decision,
                                                   config.policy.linkQuality.legacyReceiverPresent));

  receiverDevices.Get (0)->SetReceiveCallback (MakeBoundCallback (&ReceivePacket, &result));

  Ptr<WifiNetDevice> senderWifi = DynamicCast<WifiNetDevice> (senderDevices.Get (0));
  NS_ABORT_MSG_IF (senderWifi == nullptr, "Expected an installed sender Wi-Fi device");
  const bool traceConnected = senderWifi->GetPhy ()->TraceConnectWithoutContext (
      "PhyTxPsduBegin",
      MakeBoundCallback (&NotifyTxPsduBegin, &result));
  NS_ABORT_MSG_IF (!traceConnected, "Unable to connect PhyTxPsduBegin trace");

  Simulator::Schedule (MilliSeconds (100), &SendPacket, &result, senderDevices.Get (0), payloadBytes);
  Simulator::Stop (MilliSeconds (500));
  Simulator::Run ();
  Simulator::Destroy ();

  return result;
}

CaseConfig
MakeCase (const std::string& label, bool legacyReceiverPresent, double estimatedSnrDb)
{
  CaseConfig config;
  config.label = label;
  config.policy.standard = VehicularWifiProfile::Standard::IEEE_80211BD;
  config.policy.linkQuality.legacyReceiverPresent = legacyReceiverPresent;
  config.policy.linkQuality.estimatedSnrDb = estimatedSnrDb;
  return config;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 256;
  double distanceMeters = 5.0;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used by each rate-control case", payloadBytes);
  cmd.AddValue ("distance-m", "Distance between sender and receiver", distanceMeters);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (payloadBytes == 0, "payload-bytes must be greater than zero");

  const std::vector<CaseConfig> cases = {
      MakeCase ("High no-legacy", false, 35.0),
      MakeCase ("Medium no-legacy", false, 22.0),
      MakeCase ("Low no-legacy", false, 14.0),
      MakeCase ("Sub-NGV no-legacy", false, 8.0),
      MakeCase ("High legacy", true, 35.0),
      MakeCase ("Very low no-legacy", false, 2.0),
  };

  uint32_t totalTxAttempts = 0;
  uint32_t totalTxSucceeded = 0;
  uint32_t totalRxPackets = 0;
  uint32_t totalNgvPpdus = 0;
  uint32_t totalMcs0Ppdus = 0;
  uint32_t totalMcs3Ppdus = 0;
  uint32_t totalMcs7Ppdus = 0;
  uint32_t totalOtherMcsPpdus = 0;
  uint32_t totalDuplicatePpdus = 0;
  uint32_t totalNonNgv10Ppdus = 0;

  for (const auto& testCase : cases)
    {
      const CaseResult result = RunCase (testCase, payloadBytes, distanceMeters);
      totalTxAttempts += result.txAttempts;
      totalTxSucceeded += result.txSucceeded;
      totalRxPackets += result.rxPackets;
      totalNgvPpdus += result.ngvPpdus;
      totalMcs0Ppdus += result.mcs0Ppdus;
      totalMcs3Ppdus += result.mcs3Ppdus;
      totalMcs7Ppdus += result.mcs7Ppdus;
      totalOtherMcsPpdus += result.otherMcsPpdus;
      totalDuplicatePpdus += result.duplicatePpdus;
      totalNonNgv10Ppdus += result.nonNgv10Ppdus;

      std::cout << result.label << " selected decision: "
                << VehicularWifiRateController::DecisionName (result.decision) << std::endl;
      std::cout << result.label << " RX packets: " << result.rxPackets << std::endl;
    }

  std::cout << "Rate-control 802.11bd cases: " << cases.size () << std::endl;
  std::cout << "Rate-control 802.11bd TX attempts: " << totalTxAttempts << std::endl;
  std::cout << "Rate-control 802.11bd TX succeeded: " << totalTxSucceeded << std::endl;
  std::cout << "Rate-control 802.11bd RX packets: " << totalRxPackets << std::endl;
  std::cout << "Rate-control 802.11bd NGV PPDU count: " << totalNgvPpdus << std::endl;
  std::cout << "Rate-control 802.11bd NGV-MCS0 PPDU count: " << totalMcs0Ppdus
            << std::endl;
  std::cout << "Rate-control 802.11bd NGV-MCS3 PPDU count: " << totalMcs3Ppdus
            << std::endl;
  std::cout << "Rate-control 802.11bd NGV-MCS7 PPDU count: " << totalMcs7Ppdus
            << std::endl;
  std::cout << "Rate-control 802.11bd other NGV-MCS PPDU count: " << totalOtherMcsPpdus
            << std::endl;
  std::cout << "Rate-control 802.11bd NON_NGV_20_DUPLICATE PPDU count: "
            << totalDuplicatePpdus << std::endl;
  std::cout << "Rate-control 802.11bd NON_NGV_10 PPDU count: " << totalNonNgv10Ppdus
            << std::endl;

  return 0;
}
