/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/abort.h"
#include "ns3/command-line.h"
#include "ns3/error-rate-model.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/ofdm-phy.h"
#include "ns3/packet.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"
#include "ns3/vehicular-wifi-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/yans-wifi-helper.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

using namespace ns3;

namespace
{

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

struct DeliveryStats
{
  uint32_t rxPackets = 0;
  uint32_t txAttempts = 0;
  uint32_t txSucceeded = 0;
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
  WifiMacHeader broadcastData;
  broadcastData.SetType (WIFI_MAC_DATA);
  broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
  broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
  broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());
  return broadcastData;
}

bool
ReceivePacket (DeliveryStats* stats, Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t protocol, const Address& sender)
{
  stats->rxPackets++;
  return true;
}

void
SendPacket (DeliveryStats* stats, Ptr<NetDevice> device, uint32_t bytes)
{
  static constexpr uint16_t wsmpProtocol = 0x88DC;
  stats->txAttempts++;
  if (device->Send (Create<Packet> (bytes), Mac48Address::GetBroadcast (), wsmpProtocol))
    {
      stats->txSucceeded++;
    }
}

WifiTxVector
GetDataTxVector (const VehicularWifiProfile& profile, uint16_t midamblePeriodicity = 0)
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

  if (profile.GetPpduFormat () == VehicularWifiPpduFormat::NGV && midamblePeriodicity != 0)
    {
      device->GetRemoteStationManager ()->SetAttribute ("NgvMidamblePeriodicity",
                                                        UintegerValue (midamblePeriodicity));
    }

  WifiTxVector txVector =
      device->GetRemoteStationManager ()->GetDataTxVector (MakeBroadcastDataHeader ());
  Simulator::Destroy ();
  return txVector;
}

double
GetPer (Ptr<ErrorRateModel> errorModel,
        const WifiTxVector& txVector,
        double snrDb,
        uint32_t bytes,
        WifiPpduField field = WIFI_PPDU_FIELD_DATA)
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

DeliveryStats
RunDeliverySmoke (const VehicularWifiProfile& senderProfile,
                  const VehicularWifiProfile& receiverProfile,
                  uint16_t senderMidamblePeriodicity,
                  uint32_t bytes)
{
  DeliveryStats stats;

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

  Ptr<WifiNetDevice> senderWifi = DynamicCast<WifiNetDevice> (senderDevices.Get (0));
  NS_ABORT_MSG_IF (senderWifi == nullptr, "Expected an installed sender Wi-Fi device");
  if (senderProfile.GetPpduFormat () == VehicularWifiPpduFormat::NGV && senderMidamblePeriodicity != 0)
    {
      senderWifi->GetRemoteStationManager ()->SetAttribute ("NgvMidamblePeriodicity",
                                                            UintegerValue (senderMidamblePeriodicity));
    }

  receiverDevices.Get (0)->SetReceiveCallback (MakeBoundCallback (&ReceivePacket, &stats));

  Simulator::Schedule (MilliSeconds (100), &SendPacket, &stats, senderDevices.Get (0), bytes);
  Simulator::Stop (MilliSeconds (500));
  Simulator::Run ();
  Simulator::Destroy ();

  return stats;
}

} // namespace

int
main (int argc, char* argv[])
{
  uint32_t payloadBytes = 1024;
  double ngvSnrDb = 20.0;
  double nonNgvSnrDb = -6.9897;
  uint16_t ngvMidamblePeriodicity = 4;
  uint8_t nonNgvRepetitions = 2;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payload-bytes", "Payload size used by the smoke scenario", payloadBytes);
  cmd.AddValue ("ngv-snr-db", "SNR used for native NGV PER comparison", ngvSnrDb);
  cmd.AddValue ("non-ngv-snr-db", "SNR used for NON_NGV_10 PER comparison", nonNgvSnrDb);
  cmd.AddValue ("ngv-midamble-periodicity", "Native NGV midamble periodicity to exercise", ngvMidamblePeriodicity);
  cmd.AddValue ("non-ngv-repetitions", "NON_NGV_10 N_PPDU_REP value to exercise", nonNgvRepetitions);
  cmd.Parse (argc, argv);

  Ptr<ErrorRateModel> yansErrorModel = CreateObject<YansErrorRateModel> ();
  Ptr<ErrorRateModel> linearErrorModel = CreateObject<LinearSnrErrorRateModel> ();

  const WifiTxVector ngvNoMidamble = GetDataTxVector (VehicularWifiProfile::Ieee80211bd ());
  const WifiTxVector ngvWithMidamble =
      GetDataTxVector (VehicularWifiProfile::Ieee80211bd (), ngvMidamblePeriodicity);
  const double ngvNoMidamblePer = GetPer (yansErrorModel, ngvNoMidamble, ngvSnrDb, payloadBytes);
  const double ngvMidamblePer = GetPer (yansErrorModel, ngvWithMidamble, ngvSnrDb, payloadBytes);

  const WifiTxVector nonNgvNoRepeat =
      GetDataTxVector (VehicularWifiProfile::Ieee80211bdNonNgv10 ());
  const WifiTxVector nonNgvRepeated =
      GetDataTxVector (VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz",
                                                                  23.0,
                                                                  -93.0,
                                                                  4.0,
                                                                  nonNgvRepetitions));
  const double nonNgvNoRepeatPer = GetPer (linearErrorModel, nonNgvNoRepeat, nonNgvSnrDb, payloadBytes);
  const double nonNgvRepeatedPer = GetPer (linearErrorModel, nonNgvRepeated, nonNgvSnrDb, payloadBytes);
  const double nonNgvNoRepeatHeaderPer =
      GetPer (linearErrorModel, nonNgvNoRepeat, nonNgvSnrDb, payloadBytes, WIFI_PPDU_FIELD_NON_HT_HEADER);
  const double nonNgvRepeatedHeaderPer =
      GetPer (linearErrorModel, nonNgvRepeated, nonNgvSnrDb, payloadBytes, WIFI_PPDU_FIELD_NON_HT_HEADER);

  DeliveryStats ngvDelivery = RunDeliverySmoke (VehicularWifiProfile::Ieee80211bd (),
                                                VehicularWifiProfile::Ieee80211bd (),
                                                ngvMidamblePeriodicity,
                                                128);
  DeliveryStats nonNgvDelivery =
      RunDeliverySmoke (VehicularWifiProfile::Ieee80211bdNonNgv10 ("OfdmRate6MbpsBW10MHz",
                                                                   23.0,
                                                                   -93.0,
                                                                   4.0,
                                                                   nonNgvRepetitions),
                        VehicularWifiProfile::Ieee80211bdNonNgv10 (),
                        0,
                        128);

  std::cout << "NGV no-midamble PER: " << ngvNoMidamblePer << std::endl;
  std::cout << "NGV midamble PER: " << ngvMidamblePer << std::endl;
  std::cout << "NON_NGV_10 no-repeat PER: " << nonNgvNoRepeatPer << std::endl;
  std::cout << "NON_NGV_10 repeated PER: " << nonNgvRepeatedPer << std::endl;
  std::cout << "NON_NGV_10 no-repeat header PER: " << nonNgvNoRepeatHeaderPer << std::endl;
  std::cout << "NON_NGV_10 repeated header PER: " << nonNgvRepeatedHeaderPer << std::endl;
  std::cout << "NGV smoke RX packets: " << ngvDelivery.rxPackets << std::endl;
  std::cout << "NGV smoke TX succeeded: " << ngvDelivery.txSucceeded << std::endl;
  std::cout << "NON_NGV_10 smoke RX packets: " << nonNgvDelivery.rxPackets << std::endl;
  std::cout << "NON_NGV_10 smoke TX succeeded: " << nonNgvDelivery.txSucceeded << std::endl;

  NS_ABORT_MSG_IF (ngvMidamblePer >= ngvNoMidamblePer,
                   "Expected native NGV midambles to improve marginal-SNR PER");
  NS_ABORT_MSG_IF (nonNgvRepeatedPer >= nonNgvNoRepeatPer,
                   "Expected repeated NON_NGV_10 to improve marginal-SNR PER");
  NS_ABORT_MSG_IF (std::abs (nonNgvRepeatedHeaderPer - nonNgvNoRepeatHeaderPer) > 1e-12,
                   "Expected NON_NGV_10 combining to leave header PER unchanged");
  NS_ABORT_MSG_IF (ngvDelivery.rxPackets == 0 || ngvDelivery.txSucceeded == 0,
                   "Expected native NGV smoke delivery to receive packets");
  NS_ABORT_MSG_IF (nonNgvDelivery.rxPackets == 0 || nonNgvDelivery.txSucceeded == 0,
                   "Expected repeated NON_NGV_10 smoke delivery to receive packets");

  return 0;
}
