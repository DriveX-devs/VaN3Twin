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
#include "ns3/wifi-mac-header.h"
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

static constexpr uint16_t NGV_PROTOCOL = 0x88D1;
static constexpr uint16_t FALLBACK_PROTOCOL = 0x88D2;

struct ReceiverStats
{
  uint32_t ngvPackets = 0;
  uint32_t fallbackPackets = 0;
};

struct TxStats
{
  uint32_t ngvPpdus = 0;
  uint32_t duplicatePpdus = 0;
  Time ngvAirtime;
  Time duplicateAirtime;
};

struct SendStats
{
  uint32_t ngvAttempts = 0;
  uint32_t ngvSucceeded = 0;
  uint32_t fallbackAttempts = 0;
  uint32_t fallbackSucceeded = 0;
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
  if (protocol == NGV_PROTOCOL)
    {
      stats->ngvPackets++;
    }
  else if (protocol == FALLBACK_PROTOCOL)
    {
      stats->fallbackPackets++;
    }
  return true;
}

void
NotifyTxPsduBegin (TxStats* stats,
                   Ptr<WifiPhy> phy,
                   WifiConstPsduMap psduMap,
                   WifiTxVector txVector,
                   double txPowerW)
{
  const Time duration = WifiPhy::CalculateTxDuration (psduMap, txVector, phy->GetPhyBand ());
  if (txVector.IsNgv ())
    {
      stats->ngvPpdus++;
      stats->ngvAirtime += duration;
    }
  else if (txVector.IsNonNgv20Duplicate ())
    {
      stats->duplicatePpdus++;
      stats->duplicateAirtime += duration;
    }
}

void
ConfigureRuntimeFormat (Ptr<WifiRemoteStationManager> manager, bool fallback)
{
  const VehicularWifiFallbackMode mode =
      VehicularWifiFallbackController::Select (fallback, true, false);
  VehicularWifiFallbackController::ConfigureRemoteStationManager (manager, mode);
}

void
SendRuntimePhase (SendStats* stats,
                  Ptr<NetDevice> sender,
                  Ptr<WifiRemoteStationManager> manager,
                  bool fallback,
                  uint32_t payloadBytes)
{
  ConfigureRuntimeFormat (manager, fallback);
  if (fallback)
    {
      stats->fallbackAttempts++;
      if (sender->Send (Create<Packet> (payloadBytes),
                        Mac48Address::GetBroadcast (),
                        FALLBACK_PROTOCOL))
        {
          stats->fallbackSucceeded++;
        }
    }
  else
    {
      stats->ngvAttempts++;
      if (sender->Send (Create<Packet> (payloadBytes), Mac48Address::GetBroadcast (), NGV_PROTOCOL))
        {
          stats->ngvSucceeded++;
        }
    }
}

double
TimeToMicroseconds (Time value)
{
  return static_cast<double> (value.GetNanoSeconds ()) / 1000.0;
}

double
AverageDurationUs (Time airtime, uint32_t count)
{
  if (count == 0)
    {
      return 0.0;
    }
  return TimeToMicroseconds (airtime) / count;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 256;
  double distanceMeters = 5.0;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used for each runtime fallback phase", payloadBytes);
  cmd.AddValue ("distance-m", "Distance between sender and receivers", distanceMeters);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (payloadBytes == 0, "payload-bytes must be greater than zero");

  ReceiverStats receiver11bd;
  ReceiverStats receiver11p;
  TxStats txStats;
  SendStats sendStats;

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
  NS_ABORT_MSG_IF (senderWifi == nullptr, "Expected an installed sender Wi-Fi device");
  Ptr<WifiPhy> senderPhy = senderWifi->GetPhy ();
  Ptr<WifiRemoteStationManager> senderManager = senderWifi->GetRemoteStationManager ();
  NS_ABORT_MSG_IF (senderManager == nullptr, "Expected an installed sender remote station manager");

  const bool traceConnected = senderPhy->TraceConnectWithoutContext (
      "PhyTxPsduBegin",
      MakeBoundCallback (&NotifyTxPsduBegin, &txStats, senderPhy));
  NS_ABORT_MSG_IF (!traceConnected, "Unable to connect PhyTxPsduBegin trace");

  Simulator::Schedule (MilliSeconds (100),
                       &SendRuntimePhase,
                       &sendStats,
                       senderDevices.Get (0),
                       senderManager,
                       false,
                       payloadBytes);
  Simulator::Schedule (MilliSeconds (200),
                       &SendRuntimePhase,
                       &sendStats,
                       senderDevices.Get (0),
                       senderManager,
                       true,
                       payloadBytes);
  Simulator::Stop (MilliSeconds (500));
  Simulator::Run ();
  Simulator::Destroy ();

  const double ngvDurationUs = AverageDurationUs (txStats.ngvAirtime, txStats.ngvPpdus);
  const double duplicateDurationUs =
      AverageDurationUs (txStats.duplicateAirtime, txStats.duplicatePpdus);
  const double durationRatio = ngvDurationUs > 0.0 ? duplicateDurationUs / ngvDurationUs : 0.0;

  std::cout << std::fixed << std::setprecision (6);
  std::cout << "TX NGV attempts: " << sendStats.ngvAttempts << std::endl;
  std::cout << "TX NGV succeeded: " << sendStats.ngvSucceeded << std::endl;
  std::cout << "TX fallback attempts: " << sendStats.fallbackAttempts << std::endl;
  std::cout << "TX fallback succeeded: " << sendStats.fallbackSucceeded << std::endl;
  std::cout << "TX NGV PPDU count: " << txStats.ngvPpdus << std::endl;
  std::cout << "TX NON_NGV_20_DUPLICATE PPDU count: " << txStats.duplicatePpdus << std::endl;
  std::cout << "TX NGV average duration us: " << ngvDurationUs << std::endl;
  std::cout << "TX NON_NGV_20_DUPLICATE average duration us: " << duplicateDurationUs << std::endl;
  std::cout << "NON_NGV_20_DUPLICATE/NGV duration ratio: " << durationRatio << std::endl;
  std::cout << "802.11bd receiver NGV RX packets: " << receiver11bd.ngvPackets << std::endl;
  std::cout << "802.11bd receiver fallback RX packets: " << receiver11bd.fallbackPackets << std::endl;
  std::cout << "802.11p receiver NGV RX packets: " << receiver11p.ngvPackets << std::endl;
  std::cout << "802.11p receiver fallback RX packets: " << receiver11p.fallbackPackets << std::endl;

  return 0;
}
