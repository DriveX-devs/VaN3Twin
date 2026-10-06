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
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/yans-wifi-helper.h"

#include <cstdint>
#include <iomanip>
#include <iostream>

using namespace ns3;

namespace
{

static constexpr uint16_t TRACE_INITIAL_PROTOCOL = 0x88D6;
static constexpr uint16_t TRACE_ADAPTED_PROTOCOL = 0x88D7;

struct ReceiverStats
{
  uint32_t initialPackets = 0;
  uint32_t adaptedPackets = 0;
};

struct TxStats
{
  uint32_t ngvPpdus = 0;
  uint32_t duplicatePpdus = 0;
  uint32_t nonNgv10Ppdus = 0;
};

struct SendStats
{
  uint32_t attempts = 0;
  uint32_t succeeded = 0;
};

struct TracePolicyState
{
  VehicularWifiFallbackController::ObservationState observationState;
  VehicularWifiFallbackMode currentMode = VehicularWifiFallbackMode::NGV_20;
  bool legacyNeighborPresent = true;
  uint32_t traceSamples = 0;
  uint32_t policyUpdates = 0;
  double lastSnrDb = 0.0;
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

bool
ReceivePacket (ReceiverStats* stats,
               Ptr<NetDevice> dev,
               Ptr<const Packet> pkt,
               uint16_t protocol,
               const Address& sender)
{
  if (protocol == TRACE_INITIAL_PROTOCOL)
    {
      stats->initialPackets++;
    }
  else if (protocol == TRACE_ADAPTED_PROTOCOL)
    {
      stats->adaptedPackets++;
    }
  return true;
}

void
NotifyTxPsduBegin (TxStats* stats,
                   WifiConstPsduMap psduMap,
                   WifiTxVector txVector,
                   double txPowerW)
{
  if (txVector.IsNgv ())
    {
      stats->ngvPpdus++;
    }
  else if (txVector.IsNonNgv20Duplicate ())
    {
      stats->duplicatePpdus++;
    }
  else if (txVector.IsNonNgv10 ())
    {
      stats->nonNgv10Ppdus++;
    }
}

void
NotifyMonitorSniffRx (TracePolicyState* traceState,
                      Ptr<WifiRemoteStationManager> senderManager,
                      Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise,
                      uint16_t staId)
{
  traceState->traceSamples++;
  traceState->lastSnrDb =
      VehicularWifiFallbackController::EstimateSnrDb (signalNoise.signal, signalNoise.noise);
  traceState->currentMode = VehicularWifiFallbackController::UpdateFromSignalNoise (
      traceState->observationState,
      signalNoise.signal,
      signalNoise.noise,
      true,
      traceState->legacyNeighborPresent);
  traceState->policyUpdates++;
  VehicularWifiFallbackController::ConfigureRemoteStationManager (senderManager,
                                                                  traceState->currentMode);
}

void
SendPacket (SendStats* sendStats, Ptr<NetDevice> sender, uint16_t protocol, uint32_t payloadBytes)
{
  sendStats->attempts++;
  if (sender->Send (Create<Packet> (payloadBytes), Mac48Address::GetBroadcast (), protocol))
    {
      sendStats->succeeded++;
    }
}

uint32_t
TotalRx (const ReceiverStats& stats)
{
  return stats.initialPackets + stats.adaptedPackets;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 256;
  double distanceMeters = 5.0;
  bool legacyNeighborPresent = true;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used for each trace-observed fallback phase", payloadBytes);
  cmd.AddValue ("distance-m", "Distance between sender and receivers", distanceMeters);
  cmd.AddValue ("legacy-neighbor-present",
                "Whether trace observations should trigger legacy-compatible fallback",
                legacyNeighborPresent);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (payloadBytes == 0, "payload-bytes must be greater than zero");

  ReceiverStats receiver11bd;
  ReceiverStats receiver11p;
  TxStats txStats;
  SendStats sendStats;
  TracePolicyState traceState;
  traceState.legacyNeighborPresent = legacyNeighborPresent;
  traceState.observationState.policy.minNgv20SnrDb = 18.0;
  traceState.observationState.policy.minDuplicate20SnrDb = 6.0;

  NodeContainer nodes;
  nodes.Create (3);

  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0));
  positions->Add (Vector (distanceMeters, 0.0, 0.0));
  positions->Add (Vector (distanceMeters + 1.0, 0.0, 0.0));
  MobilityHelper mobility;
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = channelHelper.Create ();

  NodeContainer senderNode;
  senderNode.Add (nodes.Get (0));
  NodeContainer receiver11bdNode;
  receiver11bdNode.Add (nodes.Get (1));
  NodeContainer receiver11pNode;
  receiver11pNode.Add (nodes.Get (2));

  NetDeviceContainer senderDevices =
      InstallVehicularDevice (senderNode, channel, VehicularWifiProfile::Ieee80211bd20 ());
  NetDeviceContainer receiver11bdDevices =
      InstallVehicularDevice (receiver11bdNode, channel, VehicularWifiProfile::Ieee80211bd20 ());
  NetDeviceContainer receiver11pDevices =
      InstallVehicularDevice (receiver11pNode, channel, VehicularWifiProfile::Ieee80211p ());

  receiver11bdDevices.Get (0)->SetReceiveCallback (
      MakeBoundCallback (&ReceivePacket, &receiver11bd));
  receiver11pDevices.Get (0)->SetReceiveCallback (
      MakeBoundCallback (&ReceivePacket, &receiver11p));

  Ptr<WifiNetDevice> senderWifi = DynamicCast<WifiNetDevice> (senderDevices.Get (0));
  Ptr<WifiNetDevice> receiver11bdWifi = DynamicCast<WifiNetDevice> (receiver11bdDevices.Get (0));
  NS_ABORT_MSG_IF (senderWifi == nullptr, "Expected an installed sender Wi-Fi device");
  NS_ABORT_MSG_IF (receiver11bdWifi == nullptr, "Expected an installed 802.11bd receiver device");

  Ptr<WifiRemoteStationManager> senderManager = senderWifi->GetRemoteStationManager ();
  NS_ABORT_MSG_IF (senderManager == nullptr, "Expected an installed sender remote station manager");
  VehicularWifiFallbackController::ConfigureRemoteStationManager (senderManager,
                                                                  traceState.currentMode);

  const bool txTraceConnected = senderWifi->GetPhy ()->TraceConnectWithoutContext (
      "PhyTxPsduBegin",
      MakeBoundCallback (&NotifyTxPsduBegin, &txStats));
  NS_ABORT_MSG_IF (!txTraceConnected, "Unable to connect sender PhyTxPsduBegin trace");

  const bool rxTraceConnected = receiver11bdWifi->GetPhy ()->TraceConnectWithoutContext (
      "MonitorSnifferRx",
      MakeBoundCallback (&NotifyMonitorSniffRx, &traceState, senderManager));
  NS_ABORT_MSG_IF (!rxTraceConnected, "Unable to connect receiver MonitorSnifferRx trace");

  Simulator::Schedule (MilliSeconds (100),
                       &SendPacket,
                       &sendStats,
                       senderDevices.Get (0),
                       TRACE_INITIAL_PROTOCOL,
                       payloadBytes);
  Simulator::Schedule (MilliSeconds (300),
                       &SendPacket,
                       &sendStats,
                       senderDevices.Get (0),
                       TRACE_ADAPTED_PROTOCOL,
                       payloadBytes);
  Simulator::Stop (MilliSeconds (700));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << std::fixed << std::setprecision (6);
  std::cout << "Trace-observed fallback legacy neighbor present: " << legacyNeighborPresent
            << std::endl;
  std::cout << "Trace-observed fallback initial mode: NGV_20" << std::endl;
  std::cout << "Trace-observed fallback final mode: "
            << VehicularWifiFallbackModeName (traceState.currentMode) << std::endl;
  std::cout << "Trace-observed fallback trace samples: " << traceState.traceSamples
            << std::endl;
  std::cout << "Trace-observed fallback policy updates: " << traceState.policyUpdates
            << std::endl;
  std::cout << "Trace-observed fallback last SNR dB: " << traceState.lastSnrDb << std::endl;
  std::cout << "Trace-observed fallback send attempts: " << sendStats.attempts << std::endl;
  std::cout << "Trace-observed fallback send succeeded: " << sendStats.succeeded << std::endl;
  std::cout << "Trace-observed fallback 802.11bd RX packets: " << TotalRx (receiver11bd)
            << std::endl;
  std::cout << "Trace-observed fallback 802.11p RX packets: " << TotalRx (receiver11p)
            << std::endl;
  std::cout << "Trace-observed fallback 802.11p initial RX packets: "
            << receiver11p.initialPackets << std::endl;
  std::cout << "Trace-observed fallback 802.11p adapted RX packets: "
            << receiver11p.adaptedPackets << std::endl;
  std::cout << "Trace-observed fallback NGV PPDU count: " << txStats.ngvPpdus << std::endl;
  std::cout << "Trace-observed fallback NON_NGV_20_DUPLICATE PPDU count: "
            << txStats.duplicatePpdus << std::endl;
  std::cout << "Trace-observed fallback NON_NGV_10 PPDU count: " << txStats.nonNgv10Ppdus
            << std::endl;

  return 0;
}
