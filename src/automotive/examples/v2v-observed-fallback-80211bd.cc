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

static constexpr uint16_t OBSERVED_POLICY_PROTOCOL = 0x88D5;

struct ObservationSample
{
  std::string label;
  double observedSnrDb = 30.0;
  bool deliverySucceeded = true;
  bool legacyReceiverPresent = false;
};

struct CaseResult
{
  std::string label;
  VehicularWifiFallbackMode mode = VehicularWifiFallbackMode::NGV_20;
  double ewmaSnrDb = 0.0;
  uint32_t sampleCount = 0;
  uint32_t consecutiveFailures = 0;
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
  if (protocol == OBSERVED_POLICY_PROTOCOL)
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
  if (sender->Send (Create<Packet> (payloadBytes),
                    Mac48Address::GetBroadcast (),
                    OBSERVED_POLICY_PROTOCOL))
    {
      result->txSucceeded++;
    }
}

CaseResult
RunCase (const ObservationSample& sample,
         VehicularWifiFallbackController::ObservationState& state,
         uint32_t payloadBytes,
         double distanceMeters)
{
  CaseResult result;
  result.label = sample.label;
  result.mode = VehicularWifiFallbackController::UpdateFromObservation (
      state,
      sample.observedSnrDb,
      sample.deliverySucceeded,
      sample.legacyReceiverPresent);
  result.ewmaSnrDb = state.ewmaSnrDb;
  result.sampleCount = state.sampleCount;
  result.consecutiveFailures = state.consecutiveFailures;

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
                              MakeReceiverProfile (result.mode, sample.legacyReceiverPresent));

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

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 256;
  double distanceMeters = 5.0;
  double ewmaAlpha = 0.5;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used by each observed-policy case", payloadBytes);
  cmd.AddValue ("distance-m", "Distance between sender and receiver", distanceMeters);
  cmd.AddValue ("ewma-alpha", "EWMA alpha applied to explicit SNR observations", ewmaAlpha);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (payloadBytes == 0, "payload-bytes must be greater than zero");

  VehicularWifiFallbackController::ObservationState state;
  state.ewmaAlpha = ewmaAlpha;
  state.policy.minNgv20SnrDb = 18.0;
  state.policy.minDuplicate20SnrDb = 6.0;
  state.failureThreshold = 2;

  const std::vector<ObservationSample> samples = {
      {"Observed high", 25.0, true, false},
      {"Observed mid", 9.0, true, false},
      {"Observed low-fail-1", 2.0, false, false},
      {"Observed low-fail-2", 2.0, false, false},
      {"Observed legacy-high", 25.0, true, true},
  };

  uint32_t totalRxPackets = 0;
  uint32_t totalNgvPpdus = 0;
  uint32_t totalDuplicatePpdus = 0;
  uint32_t totalNonNgv10Ppdus = 0;

  for (const auto& sample : samples)
    {
      const CaseResult result = RunCase (sample, state, payloadBytes, distanceMeters);
      totalRxPackets += result.rxPackets;
      totalNgvPpdus += result.ngvPpdus;
      totalDuplicatePpdus += result.duplicatePpdus;
      totalNonNgv10Ppdus += result.nonNgv10Ppdus;

      std::cout << result.label << " selected mode: "
                << VehicularWifiFallbackModeName (result.mode) << std::endl;
      std::cout << result.label << " EWMA SNR dB: " << result.ewmaSnrDb << std::endl;
      std::cout << result.label << " consecutive failures: " << result.consecutiveFailures
                << std::endl;
      std::cout << result.label << " RX packets: " << result.rxPackets << std::endl;
    }

  std::cout << "Observed fallback policy cases: " << samples.size () << std::endl;
  std::cout << "Observed fallback policy RX packets: " << totalRxPackets << std::endl;
  std::cout << "Observed fallback policy NGV PPDU count: " << totalNgvPpdus << std::endl;
  std::cout << "Observed fallback policy NON_NGV_20_DUPLICATE PPDU count: "
            << totalDuplicatePpdus << std::endl;
  std::cout << "Observed fallback policy NON_NGV_10 PPDU count: " << totalNonNgv10Ppdus
            << std::endl;
  std::cout << "Observed fallback policy final EWMA SNR dB: " << state.ewmaSnrDb << std::endl;
  std::cout << "Observed fallback policy final consecutive failures: " << state.consecutiveFailures
            << std::endl;

  return 0;
}
