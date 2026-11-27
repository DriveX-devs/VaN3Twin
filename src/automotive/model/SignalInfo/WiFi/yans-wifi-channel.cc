/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage, <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/wifi-net-device.h"
#include "ns3/node.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/mobility-model.h"
#include "ns3/ofdm-ppdu.h"
#include "yans-wifi-channel.h"
#include "yans-wifi-phy.h"
#include "wifi-utils.h"
#include "wifi-ppdu.h"
#include "wifi-psdu.h"

#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("YansWifiChannel");

namespace
{

bool
FrequencyRangesOverlap (uint16_t centerA, uint16_t widthA, uint16_t centerB, uint16_t widthB)
{
  const int minA = static_cast<int> (centerA) - static_cast<int> (widthA) / 2;
  const int maxA = static_cast<int> (centerA) + static_cast<int> (widthA) / 2;
  const int minB = static_cast<int> (centerB) - static_cast<int> (widthB) / 2;
  const int maxB = static_cast<int> (centerB) + static_cast<int> (widthB) / 2;

  return std::max (minA, minB) < std::min (maxA, maxB);
}

bool
ChannelsOverlap (Ptr<const YansWifiPhy> sender, Ptr<const YansWifiPhy> receiver)
{
  if (sender->GetChannelNumber () == receiver->GetChannelNumber ())
    {
      return true;
    }
  return FrequencyRangesOverlap (sender->GetFrequency (),
                                 sender->GetChannelWidth (),
                                 receiver->GetFrequency (),
                                 receiver->GetChannelWidth ());
}

Ptr<WifiPpdu>
GetReceiverPpdu (Ptr<const WifiPpdu> ppdu, Ptr<const YansWifiPhy> sender)
{
  WifiTxVector txVector = ppdu->GetTxVector ();
  if (!txVector.IsNonNgv20Duplicate ())
    {
      return ppdu->Copy ();
    }

  txVector.SetNgvPpduFormat (WifiNgvPpduFormat::NON_NGV_10);
  txVector.SetChannelWidth (10);
  txVector.SetNss (1);
  txVector.SetLdpc (false);

  Ptr<WifiPpdu> copy =
      Create<OfdmPpdu> (ppdu->GetPsdu (), txVector, sender->GetPhyBand (), ppdu->GetUid ());
  if (ppdu->IsTruncatedTx ())
    {
      copy->SetTruncatedTx ();
    }
  return copy;
}

} // namespace

NS_OBJECT_ENSURE_REGISTERED (YansWifiChannel);

TypeId
YansWifiChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::YansWifiChannel")
                          .SetParent<Channel> ()
                          .SetGroupName ("Wifi")
                          .AddConstructor<YansWifiChannel> ()
                          .AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                                         PointerValue (),
                                         MakePointerAccessor (&YansWifiChannel::m_loss),
                                         MakePointerChecker<PropagationLossModel> ())
                          .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                                         PointerValue (),
                                         MakePointerAccessor (&YansWifiChannel::m_delay),
                                         MakePointerChecker<PropagationDelayModel> ())
      ;
  return tid;
}

YansWifiChannel::YansWifiChannel ()
{
  NS_LOG_FUNCTION (this);
}

YansWifiChannel::~YansWifiChannel ()
{
  NS_LOG_FUNCTION (this);
  m_phyList.clear ();
}

void
YansWifiChannel::SetPropagationLossModel (const Ptr<PropagationLossModel> loss)
{
  NS_LOG_FUNCTION (this << loss);
  m_loss = loss;
}

void
YansWifiChannel::SetPropagationDelayModel (const Ptr<PropagationDelayModel> delay)
{
  NS_LOG_FUNCTION (this << delay);
  m_delay = delay;
}

void
YansWifiChannel::Send (Ptr<YansWifiPhy> sender, Ptr<const WifiPpdu> ppdu, double txPowerDbm) const
{
  NS_LOG_FUNCTION (this << sender << ppdu << txPowerDbm);
  Ptr<MobilityModel> senderMobility = sender->GetMobility ();
  NS_ASSERT (senderMobility != 0);
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++)
    {
      if (sender != (*i))
        {
          if (!ChannelsOverlap (sender, *i))
            {
              continue;
            }

          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
          double rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
          NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
                        "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);
          Ptr<WifiPpdu> copy = GetReceiverPpdu (ppdu, sender);
          Ptr<NetDevice> dstNetDevice = (*i)->GetDevice ();
          uint32_t dstNode;
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetNode ()->GetId ();
            }

          Simulator::ScheduleWithContext (dstNode,
                                          delay, &YansWifiChannel::Receive,
                                          (*i), copy, rxPowerDbm);
        }
    }
}

void
YansWifiChannel::Receive (Ptr<YansWifiPhy> phy, Ptr<WifiPpdu> ppdu, double rxPowerDbm)
{
  NS_LOG_FUNCTION (phy << ppdu << rxPowerDbm);
  // Do no further processing if signal is too weak
  // Current implementation assumes constant RX power over the PPDU duration
  if ((rxPowerDbm + phy->GetRxGain ()) < phy->GetRxSensitivity ())
    {
      NS_LOG_INFO ("Received signal too weak to process: " << rxPowerDbm << " dBm");
      return;
    }
  RxPowerWattPerChannelBand rxPowerW;
  rxPowerW.insert ({std::make_pair (0, 0), (DbmToW (rxPowerDbm + phy->GetRxGain ()))}); //dummy band for YANS
  phy->StartReceivePreamble (ppdu, rxPowerW, ppdu->GetTxDuration ());
}

std::size_t
YansWifiChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}

Ptr<NetDevice>
YansWifiChannel::GetDevice (std::size_t i) const
{
  return m_phyList[i]->GetDevice ();
}

void
YansWifiChannel::Add (Ptr<YansWifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  m_phyList.push_back (phy);
}

int64_t
YansWifiChannel::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  int64_t currentStream = stream;
  currentStream += m_loss->AssignStreams (stream);
  return (currentStream - stream);
}

} //namespace ns3
