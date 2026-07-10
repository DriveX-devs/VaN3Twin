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
#include <string>
#include <vector>

using namespace ns3;

namespace
{

struct ScenarioResult
{
  std::string label;
  uint8_t nss = 1;
  uint8_t ngvMcs = 3;
  uint16_t channelWidthMHz = 10;
  uint32_t payloadBytes = 0;
  uint32_t txAttempts = 0;
  uint32_t txSucceeded = 0;
  uint32_t rxPackets = 0;
  Time nominalTxDuration;
  Time observedTxAirtime;
  std::vector<Time> observedTxDurations;
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

WifiMacHeader
MakeBroadcastDataHeader ()
{
  WifiMacHeader header;
  header.SetType (WIFI_MAC_DATA);
  header.SetAddr1 (Mac48Address::GetBroadcast ());
  header.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
  header.SetAddr3 (Mac48Address::GetBroadcast ());
  return header;
}

VehicularWifiProfile
MakeNgvProfile (uint16_t channelWidthMHz, uint8_t ngvMcs, uint8_t nss)
{
  if (channelWidthMHz == 20)
    {
      return VehicularWifiProfile::Ieee80211bd20 ("OfdmRate24Mbps",
                                                  23.0,
                                                  -92.0,
                                                  4.0,
                                                  ngvMcs,
                                                  nss);
    }

  NS_ABORT_MSG_IF (channelWidthMHz != 10, "Only 10 MHz and 20 MHz 802.11bd profiles are supported");
  return VehicularWifiProfile::Ieee80211bd ("OfdmRate12MbpsBW10MHz",
                                            23.0,
                                            -95.0,
                                            4.0,
                                            ngvMcs,
                                            nss,
                                            channelWidthMHz);
}

bool
ReceivePacket (ScenarioResult* result,
               Ptr<NetDevice> dev,
               Ptr<const Packet> pkt,
               uint16_t protocol,
               const Address& sender)
{
  result->rxPackets++;
  return true;
}

void
SendPacket (ScenarioResult* result, Ptr<NetDevice> device, uint32_t payloadBytes)
{
  static constexpr uint16_t wsmpProtocol = 0x88DC;
  result->txAttempts++;
  if (device->Send (Create<Packet> (payloadBytes), Mac48Address::GetBroadcast (), wsmpProtocol))
    {
      result->txSucceeded++;
    }
}

void
NotifyTxPsduBegin (ScenarioResult* result,
                   Ptr<WifiPhy> phy,
                   WifiConstPsduMap psduMap,
                   WifiTxVector txVector,
                   double txPowerW)
{
  const Time duration = WifiPhy::CalculateTxDuration (psduMap, txVector, phy->GetPhyBand ());
  result->observedTxDurations.push_back (duration);
  result->observedTxAirtime += duration;
}

double
TimeToMicroseconds (Time value)
{
  return static_cast<double> (value.GetNanoSeconds ()) / 1000.0;
}

double
TimeToMilliseconds (Time value)
{
  return static_cast<double> (value.GetNanoSeconds ()) / 1000000.0;
}

double
AverageObservedDurationUs (const ScenarioResult& result)
{
  if (result.observedTxDurations.empty ())
    {
      return 0.0;
    }

  return TimeToMicroseconds (result.observedTxAirtime) / result.observedTxDurations.size ();
}

double
AirtimeGoodputMbps (const ScenarioResult& result)
{
  const double airtimeSeconds = result.observedTxAirtime.GetSeconds ();
  if (airtimeSeconds <= 0.0)
    {
      return 0.0;
    }
  return (result.rxPackets * result.payloadBytes * 8.0) / airtimeSeconds / 1e6;
}

ScenarioResult
RunScenario (const std::string& label,
             const VehicularWifiProfile& profile,
             uint32_t payloadBytes,
             uint32_t packetCount,
             uint32_t packetIntervalUs,
             double distanceMeters)
{
  ScenarioResult result;
  result.label = label;
  result.nss = profile.GetNgvSpatialStreams ();
  result.ngvMcs = profile.GetNgvMcsIndex ();
  result.channelWidthMHz = profile.GetChannelWidthMHz ();
  result.payloadBytes = payloadBytes;

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

  NetDeviceContainer senderDevices = InstallVehicularDevice (senderNode, channel, profile);
  NetDeviceContainer receiverDevices = InstallVehicularDevice (receiverNode, channel, profile);
  receiverDevices.Get (0)->SetReceiveCallback (MakeBoundCallback (&ReceivePacket, &result));

  Ptr<WifiNetDevice> senderWifi = DynamicCast<WifiNetDevice> (senderDevices.Get (0));
  NS_ABORT_MSG_IF (senderWifi == nullptr, "Expected an installed sender Wi-Fi device");
  Ptr<WifiPhy> senderPhy = senderWifi->GetPhy ();

  const WifiTxVector dataTxVector =
      senderWifi->GetRemoteStationManager ()->GetDataTxVector (MakeBroadcastDataHeader ());
  result.nominalTxDuration =
      WifiPhy::CalculateTxDuration (payloadBytes, dataTxVector, senderPhy->GetPhyBand ());

  const bool traceConnected = senderPhy->TraceConnectWithoutContext (
      "PhyTxPsduBegin",
      MakeBoundCallback (&NotifyTxPsduBegin, &result, senderPhy));
  NS_ABORT_MSG_IF (!traceConnected, "Unable to connect PhyTxPsduBegin trace");

  const Time firstTransmission = MilliSeconds (100);
  for (uint32_t i = 0; i < packetCount; ++i)
    {
      Simulator::Schedule (firstTransmission + MicroSeconds (static_cast<uint64_t> (i) * packetIntervalUs),
                           &SendPacket,
                           &result,
                           senderDevices.Get (0),
                           payloadBytes);
    }

  Simulator::Stop (firstTransmission + MicroSeconds (static_cast<uint64_t> (packetCount) * packetIntervalUs)
                   + MilliSeconds (100));
  Simulator::Run ();
  Simulator::Destroy ();

  return result;
}

void
PrintScenarioResult (const ScenarioResult& result)
{
  const std::string prefix = result.label;
  std::cout << prefix << " NGV-MCS: " << static_cast<uint32_t> (result.ngvMcs) << std::endl;
  std::cout << prefix << " channel width MHz: " << result.channelWidthMHz << std::endl;
  std::cout << prefix << " spatial streams: " << static_cast<uint32_t> (result.nss) << std::endl;
  std::cout << prefix << " nominal TX duration us: " << TimeToMicroseconds (result.nominalTxDuration)
            << std::endl;
  std::cout << prefix << " observed average TX duration us: " << AverageObservedDurationUs (result)
            << std::endl;
  std::cout << prefix << " total TX airtime ms: " << TimeToMilliseconds (result.observedTxAirtime)
            << std::endl;
  std::cout << prefix << " TX attempts: " << result.txAttempts << std::endl;
  std::cout << prefix << " TX succeeded: " << result.txSucceeded << std::endl;
  std::cout << prefix << " RX packets: " << result.rxPackets << std::endl;
  std::cout << prefix << " airtime goodput Mbps: " << AirtimeGoodputMbps (result) << std::endl;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 1024;
  uint32_t packetCount = 20;
  uint32_t packetIntervalUs = 2000;
  uint32_t ngvMcs = 3;
  uint32_t channelWidthMHz = 10;
  double distanceMeters = 5.0;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used for each broadcast packet", payloadBytes);
  cmd.AddValue ("packet-count", "Number of packets sent in each NSS scenario", packetCount);
  cmd.AddValue ("packet-interval-us", "Inter-packet interval in microseconds", packetIntervalUs);
  cmd.AddValue ("ngv-mcs", "NGV-MCS index used for both NSS=1 and NSS=2", ngvMcs);
  cmd.AddValue ("channel-width-mhz", "802.11bd channel width, 10 or 20 MHz", channelWidthMHz);
  cmd.AddValue ("distance-m", "Distance between sender and receiver", distanceMeters);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (packetCount == 0, "packet-count must be greater than zero");
  NS_ABORT_MSG_IF (packetIntervalUs == 0, "packet-interval-us must be greater than zero");
  NS_ABORT_MSG_IF (ngvMcs > 255, "ngv-mcs must fit in uint8_t");
  NS_ABORT_MSG_IF (channelWidthMHz != 10 && channelWidthMHz != 20,
                   "channel-width-mhz must be 10 or 20");

  std::cout << std::fixed << std::setprecision (6);

  const ScenarioResult nss1 = RunScenario ("NSS1",
                                           MakeNgvProfile (channelWidthMHz,
                                                           static_cast<uint8_t> (ngvMcs),
                                                           1),
                                           payloadBytes,
                                           packetCount,
                                           packetIntervalUs,
                                           distanceMeters);
  const ScenarioResult nss2 = RunScenario ("NSS2",
                                           MakeNgvProfile (channelWidthMHz,
                                                           static_cast<uint8_t> (ngvMcs),
                                                           2),
                                           payloadBytes,
                                           packetCount,
                                           packetIntervalUs,
                                           distanceMeters);

  PrintScenarioResult (nss1);
  PrintScenarioResult (nss2);

  const double durationRatio = AverageObservedDurationUs (nss2) / AverageObservedDurationUs (nss1);
  const double airtimeRatio =
      TimeToMicroseconds (nss2.observedTxAirtime) / TimeToMicroseconds (nss1.observedTxAirtime);
  const double goodputRatio = AirtimeGoodputMbps (nss2) / AirtimeGoodputMbps (nss1);

  std::cout << "NSS2/NSS1 average TX duration ratio: " << durationRatio << std::endl;
  std::cout << "NSS2/NSS1 total TX airtime ratio: " << airtimeRatio << std::endl;
  std::cout << "NSS2/NSS1 airtime goodput ratio: " << goodputRatio << std::endl;

  return 0;
}
