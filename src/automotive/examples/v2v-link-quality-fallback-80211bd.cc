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

static constexpr uint16_t POLICY_PROTOCOL = 0x88D4;

struct CaseConfig
{
  std::string label;
  VehicularWifiFallbackController::LinkQualityPolicy policy;
};

struct CaseResult
{
  std::string label;
  VehicularWifiFallbackMode mode = VehicularWifiFallbackMode::NGV_20;
  uint32_t txAttempts = 0;
  uint32_t txSucceeded = 0;
  uint32_t rxPackets = 0;
  uint32_t ngvPpdus = 0;
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
MakeReceiverProfile (VehicularWifiFallbackMode mode, bool legacyReceiverPresent)
{
  if (legacyReceiverPresent)
    {
      return VehicularWifiProfile::Ieee80211p ();
    }
  if (mode == VehicularWifiFallbackMode::NGV_20 ||
      mode == VehicularWifiFallbackMode::NON_NGV_20_DUPLICATE)
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
  if (protocol == POLICY_PROTOCOL)
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
  if (sender->Send (Create<Packet> (payloadBytes), Mac48Address::GetBroadcast (), POLICY_PROTOCOL))
    {
      result->txSucceeded++;
    }
}

CaseResult
RunCase (const CaseConfig& config, uint32_t payloadBytes, double distanceMeters)
{
  CaseResult result;
  result.label = config.label;
  result.mode = VehicularWifiFallbackController::SelectForLinkQuality (config.policy);

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
                              VehicularWifiFallbackController::MakeProfile (result.mode));
  NetDeviceContainer receiverDevices =
      InstallVehicularDevice (receiverNode,
                              channel,
                              MakeReceiverProfile (result.mode, config.policy.legacyReceiverPresent));

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
  config.policy.legacyReceiverPresent = legacyReceiverPresent;
  config.policy.estimatedSnrDb = estimatedSnrDb;
  return config;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 256;
  double distanceMeters = 5.0;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used by each link-quality policy case", payloadBytes);
  cmd.AddValue ("distance-m", "Distance between sender and receiver", distanceMeters);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (payloadBytes == 0, "payload-bytes must be greater than zero");

  const std::vector<CaseConfig> cases = {
      MakeCase ("High no-legacy", false, 25.0),
      MakeCase ("Mid no-legacy", false, 12.0),
      MakeCase ("Low no-legacy", false, 2.0),
      MakeCase ("High legacy", true, 25.0),
      MakeCase ("Low legacy", true, 2.0),
  };

  uint32_t totalRxPackets = 0;
  uint32_t totalNgvPpdus = 0;
  uint32_t totalDuplicatePpdus = 0;
  uint32_t totalNonNgv10Ppdus = 0;

  for (const auto& testCase : cases)
    {
      const CaseResult result = RunCase (testCase, payloadBytes, distanceMeters);
      totalRxPackets += result.rxPackets;
      totalNgvPpdus += result.ngvPpdus;
      totalDuplicatePpdus += result.duplicatePpdus;
      totalNonNgv10Ppdus += result.nonNgv10Ppdus;

      std::cout << result.label << " selected mode: "
                << VehicularWifiFallbackModeName (result.mode) << std::endl;
      std::cout << result.label << " RX packets: " << result.rxPackets << std::endl;
    }

  std::cout << "Link-quality policy cases: " << cases.size () << std::endl;
  std::cout << "Link-quality policy RX packets: " << totalRxPackets << std::endl;
  std::cout << "Link-quality policy NGV PPDU count: " << totalNgvPpdus << std::endl;
  std::cout << "Link-quality policy NON_NGV_20_DUPLICATE PPDU count: " << totalDuplicatePpdus
            << std::endl;
  std::cout << "Link-quality policy NON_NGV_10 PPDU count: " << totalNonNgv10Ppdus
            << std::endl;

  return 0;
}
