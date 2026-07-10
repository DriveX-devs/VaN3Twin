/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

* Created by:
    *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#include "DCC.h"

#include "ns3/mac48-address.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-tx-vector.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("DCC");

TypeId
DCC::GetTypeId ()
{
  static TypeId tid = TypeId("ns3::DCC")
                          .SetParent <Object>()
                          .AddConstructor <DCC>();
  return tid;
}

DCC::DCC ()
{

};

DCC::~DCC()
= default;

void DCC::setCBRG(double cbr_g)
{
  // Set the new value for CBR_G and save the previous one
  m_CBR_G[1] = m_CBR_G[0];
  // For the first time, set the second CBR_G to 0
  if (m_CBR_G[1] == -1) m_CBR_G[1] = 0.0;
  m_CBR_G[0] = cbr_g;
};

void DCC::setNewCBRL0Hop(double cbr)
{
  m_CBR_L0_Hop[1] = m_CBR_L0_Hop[0];
  m_CBR_L0_Hop[0] = cbr;
}

std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> DCC::getConfiguration(double Ton, double currentCBR)
{
    std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> map;
    if (Ton <= 0.5)
      {
        map = m_reactive_parameters_Ton_500_us;
      }
    else if (Ton > 0.5 && Ton <= 1)
      {
        map = m_reactive_parameters_Ton_1ms;
      }
    else
      {
        // Default
        map = m_reactive_parameters_Ton_1ms;
      }
    ReactiveState old_state = m_current_state;
    if (currentCBR >= map[m_current_state].cbr_threshold && m_current_state != ReactiveState::Restrictive)
    {
      m_current_state = static_cast<ReactiveState>(m_current_state + 1);
    }
    else
      {
        if (m_current_state != ReactiveState::Relaxed)
          {
            ReactiveState prev_state = static_cast<ReactiveState> (m_current_state - 1);
            if (currentCBR <= map[prev_state].cbr_threshold)
              {
                m_current_state = static_cast<ReactiveState> (m_current_state - 1);
              }
          }
      }
    return map;
}

void DCC::SetupDCC(std::string item_id, Ptr<MetricSupervisor> met_sup, Ptr<Node> node, std::string modality, uint32_t dcc_interval, float cbr_target, int queue_length, int max_lifetime, std::string log_file, std::string profile_DCC)
{
  NS_ASSERT_MSG (modality == "adaptive" || modality == "reactive", "DCC modality can be only adaptive or reactive");
  NS_ASSERT_MSG (dcc_interval > 0, "DCC interval must be greater than 0");
  NS_ASSERT_MSG (met_sup != nullptr, "MetricSupervisor is null");
  NS_ASSERT_MSG (node != nullptr, "Node is null");
  m_item_id = item_id;
  m_node = node;
  m_modality = modality;
  m_dcc_interval = dcc_interval;
  m_metric_supervisor = met_sup;
  m_CBR_target = cbr_target;
  m_queue_length = queue_length;
  m_lifetime = max_lifetime;
  m_log_file = log_file;
  m_profile = profile_DCC;
}

Ptr<WifiPhy> DCC::GetPhy()
{
  Ptr<NetDevice> netDevice = m_node->GetDevice (0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (netDevice);
  if (wifiDevice == nullptr)
    {
      NS_FATAL_ERROR("WiFi Device object not found.");
    }
  // Get the PHY layer
  return wifiDevice->GetPhy ();
}

void DCC::StartDCC()
{
  if (m_modality == "adaptive")
  {
    Simulator::Schedule(MilliSeconds(m_T_CBR), &DCC::adaptiveDCCcheckCBR, this);
    Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::adaptiveDCC, this);
    if (m_log_file != "")
      {
        std::ofstream file;
        file.open(m_log_file, std::ios::out);
        file << "timestamp_unix_s,currentCBR,CBRITS,new_delta,int_pkt_time,#_dropped_lifetime,#_dropped_full_queue,#_pkt_to_send,#_pkt_received,average_aoi_dp0,average_aoi_dp1,average_aoi_dp2,average_aoi_dp3\n";
        file.close();
      }
  }
  else
  {
    Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::reactiveDCC, this);
    if (m_log_file != "")
      {
        std::ofstream file;
        file.open(m_log_file, std::ios::out);
        file << "timestamp_unix_s,CBR,state,tx_pwr,int_pkt_time,#_dropped_lifetime,#_dropped_full_queue,#_pkt_to_send,#_pkt_received,average_aoi_dp0,average_aoi_dp1,average_aoi_dp2,average_aoi_dp3\n";
        file.close();
      }
  }
  Simulator::Schedule (MilliSeconds (m_T_DCC_NET_Trig), &DCC::DCCcheckCBRG, this);
}

void DCC::DCCcheckCBRG()
{
  // Start check CBR from sharing process through GeoNet
  m_cbr_g_callback();
  Simulator::Schedule (MilliSeconds (m_T_DCC_NET_Trig), &DCC::DCCcheckCBRG, this);
}

void DCC::reactiveDCC()
{
  NS_LOG_INFO("Starting DCC check");
  NS_ASSERT_MSG (m_metric_supervisor != nullptr, "Metric Supervisor not set");
  NS_ASSERT_MSG (m_dcc_interval != -1, "DCC interval not set");

  double currentCBR;
  m_current_cbr =  m_metric_supervisor->getCBRPerItem(m_item_id);
  setNewCBRL0Hop (m_current_cbr);
  if (m_profile == "etsi")
    {
      // ETSI strategy (default): last sensed CBR
      currentCBR = m_CBR_L0_Hop[0];
    }
  else if (m_profile == "c2c")
    {
      // Car2Car strategy: average of the last two sensed CBRs
      currentCBR = 0.5 * (m_CBR_L0_Hop[0] + m_CBR_L0_Hop[1]);
    }
  else if (m_profile == "cbrg")
    {
      // DriveX strategy
      if (m_CBR_G[0] == -1 && m_CBR_G[1] == -1)
        {
          // If the CBR_G is not available, adopt the C2C strategy
          currentCBR = 0.5 * (m_CBR_L0_Hop[0] + m_CBR_L0_Hop[1]);
        }
      // average of the last two calculated CBR_Gs
      else currentCBR = 0.5 * (m_CBR_G[0] + m_CBR_G[1]);
    }

  if (currentCBR == -1)
    {
      Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::reactiveDCC, this);
      return;
    }
  // Get the NetDevice
  Ptr<NetDevice> netDevice = m_node->GetDevice (0);
  Ptr<WifiNetDevice> wifiDevice;
  Ptr<WifiPhy> phy80211p = nullptr;
  // Get the WifiNetDevice
  wifiDevice = DynamicCast<WifiNetDevice> (netDevice);
  if (wifiDevice == nullptr)
  {
    NS_FATAL_ERROR("WiFi Device object not found.");
  }
  // Get the PHY layer
  phy80211p = wifiDevice->GetPhy ();
  std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(m_Ton_pp, currentCBR);
  long tx_power = map[m_current_state].tx_power;
  long int_pkt_time = map[m_current_state].tx_inter_packet_time;
  float sensitivity = map[m_current_state].sensitivity;
  phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
  phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
  phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
  updateTgoAfterStateCheck(map[m_current_state].tx_inter_packet_time);

  if(m_log_file != "")
    {
      std::ofstream file;
      file.open(m_log_file, std::ios::app);

      float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
      int dropped_lifetime = m_dropped_lifetime;
      int dropped_full_queue = m_dropped_full_queue;
      int pkt_to_send = m_pkt_to_send;
      int pkt_received = m_pkt_received;

      double average_aoi_dp0, average_aoi_dp1, average_aoi_dp2, average_aoi_dp3;

      if (m_packet_sent_dp0 == 0) average_aoi_dp0 = -1;
      else average_aoi_dp0 = m_cumulative_time_dp0 / m_packet_sent_dp0;
      m_cumulative_time_dp0 = 0;
      m_packet_sent_dp0 = 0;

      if (m_packet_sent_dp1 == 0) average_aoi_dp1 = -1;
      else average_aoi_dp1 = m_cumulative_time_dp1 / m_packet_sent_dp1;
      m_cumulative_time_dp1 = 0;
      m_packet_sent_dp1 = 0;

      if (m_packet_sent_dp2 == 0) average_aoi_dp2 = -1;
      else average_aoi_dp2 = m_cumulative_time_dp2 / m_packet_sent_dp2;
      m_cumulative_time_dp2 = 0;
      m_packet_sent_dp2 = 0;

      if (m_packet_sent_dp3 == 0) average_aoi_dp3 = -1;
      else average_aoi_dp3 = m_cumulative_time_dp3 / m_packet_sent_dp3;
      m_cumulative_time_dp3 = 0;
      m_packet_sent_dp3 = 0;

      file << std::fixed << now << "," << currentCBR << "," << m_current_state << "," << tx_power << "," << int_pkt_time << "," << dropped_lifetime << "," << dropped_full_queue << "," << pkt_to_send << "," << pkt_received << "," << average_aoi_dp0 << "," << average_aoi_dp1 << "," << average_aoi_dp2 << "," << average_aoi_dp3 << "\n";
      file.close();
    }

  Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::reactiveDCC, this);
}

void DCC::adaptiveDCCcheckCBR()
{
  NS_ASSERT_MSG (m_metric_supervisor != nullptr, "Metric Supervisor not set");
  double previous_cbr = m_metric_supervisor->getCBRPerItem(m_item_id);
  if (previous_cbr == -1) previous_cbr = 0;
  setNewCBRL0Hop (previous_cbr);
  Simulator::Schedule(MilliSeconds(m_T_CBR), &DCC::adaptiveDCCcheckCBR, this);
}

void DCC::adaptiveDCC()
{
  NS_LOG_INFO ("Starting DCC check");
  NS_ASSERT_MSG (m_metric_supervisor != nullptr, "Metric Supervisor not set");
  NS_ASSERT_MSG (m_dcc_interval != -1.0, "DCC interval not set");

  double currentCBR = m_metric_supervisor->getCBRPerItem(m_item_id);
  if (currentCBR == -1)
    {
      Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::adaptiveDCC, this);
      return;
    }
  m_current_cbr = currentCBR;
  setNewCBRL0Hop (currentCBR);
  Time now = Simulator::Now ();
  double time = now.GetSeconds ();
  double delta_offset;
  // Step 1
  if (m_CBR_its != -1)
    {
      if (m_CBR_G[0] == -1 && m_CBR_G[1] == -1)
        {
          m_CBR_its = 0.5 * m_CBR_its + 0.5 * ((m_CBR_L0_Hop[0] + m_CBR_L0_Hop[1]) / 2);
        }
      else
        {
          m_CBR_its = 0.5 * m_CBR_its + 0.5 * ((m_CBR_G[0] + m_CBR_G[1]) / 2);
        }
    }
  else
    {
      if (m_CBR_G[0] == -1 && m_CBR_G[1] == -1)
        {
          m_CBR_its = (m_CBR_L0_Hop[0] + m_CBR_L0_Hop[1]) / 2;
        }
      else
        {
          m_CBR_its = (m_CBR_G[0] + m_CBR_G[1]) / 2;
        }
    }
  // Step 2
  double factor1 = m_beta * (m_CBR_target - m_CBR_its);
  if ((m_CBR_target - m_CBR_its) > 0)
    {
      delta_offset = factor1 < m_Gmax ? factor1 : m_Gmax;
    }
  else
    {
      delta_offset = factor1 > m_Gmin ? factor1 : m_Gmin;
    }

  // Step 3
  float old_delta = m_delta;
  if (old_delta == 0)
    {
      // First case
      old_delta = m_Ton_pp / 100.0 > 1e-3 ? m_Ton_pp / 100.0 : 1e-3;
    }
  float new_delta = (1 - m_alpha) * old_delta + delta_offset;

  // Step 4
  bool flag = false;
  if (new_delta > m_delta_max)
    {
      m_delta = m_delta_max;
      flag = true;
    }

  // Step 5
  if (new_delta < m_delta_min)
    {
      m_delta = m_delta_min;
      flag = true;
    }

  if (!flag)
    {
      m_delta = new_delta;
    }

  updateTgoAfterDeltaUpdate();

  if(m_log_file != "")
    {
      std::ofstream file;
      file.open(m_log_file, std::ios::app);

      float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
      int dropped_full_queue = m_dropped_full_queue;
      int dropped_lifetime = m_dropped_lifetime;
      int pkt_to_send = m_pkt_to_send;
      int pkt_received = m_pkt_received;

      double average_aoi_dp0, average_aoi_dp1, average_aoi_dp2, average_aoi_dp3;

      if (m_packet_sent_dp0 == 0) average_aoi_dp0 = -1;
      else average_aoi_dp0 = m_cumulative_time_dp0 / m_packet_sent_dp0;
      m_cumulative_time_dp0 = 0;
      m_packet_sent_dp0 = 0;

      if (m_packet_sent_dp1 == 0) average_aoi_dp1 = -1;
      else average_aoi_dp1 = m_cumulative_time_dp1 / m_packet_sent_dp1;
      m_cumulative_time_dp1 = 0;
      m_packet_sent_dp1 = 0;

      if (m_packet_sent_dp2 == 0) average_aoi_dp2 = -1;
      else average_aoi_dp2 = m_cumulative_time_dp2 / m_packet_sent_dp2;
      m_cumulative_time_dp2 = 0;
      m_packet_sent_dp2 = 0;

      if (m_packet_sent_dp3 == 0) average_aoi_dp3 = -1;
      else average_aoi_dp3 = m_cumulative_time_dp3 / m_packet_sent_dp3;
      m_cumulative_time_dp3 = 0;
      m_packet_sent_dp3 = 0;

      file << std::fixed << now << "," << currentCBR << "," << m_CBR_its << "," << new_delta << "," << m_Toff_ms <<"," << dropped_lifetime << "," << dropped_full_queue << "," << pkt_to_send << "," << pkt_received << "," << average_aoi_dp0 << "," << average_aoi_dp1 << "," << average_aoi_dp2 << "," << average_aoi_dp3 << "\n";
      file.close();
    }

  Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::adaptiveDCC, this);
}

void DCC::updateTgoAfterTransmission()
{
  float aux;
  if (m_Ton_pp / m_delta > 25)
    {
      aux = m_Ton_pp / m_delta;
    }
  else
    {
      aux = 25;
    }

  if (aux > 1000)
    {
      aux = 1000;
    }

  m_Tpg_ms = static_cast<float>(Simulator::Now().GetMilliSeconds());
  // Compute next time gate will be open
  m_Tgo_ms = m_Tpg_ms + aux;
  m_Toff_ms = aux;
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  float elapsed = now - m_last_tx;
  if (m_queue_length > 0)
    {
      if (elapsed >= m_Toff_ms) checkQueue();
      else Simulator::Schedule(MilliSeconds(m_Toff_ms - elapsed), &DCC::checkQueue, this);
    }
}

void DCC::updateTgoAfterDeltaUpdate()
{
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  if (checkGateOpen(now))
    {
      // Update just if the gate is currently closed, otherwise return
      return;
    }
  float aux = m_Ton_pp / m_delta;
  aux = aux * ((m_Tgo_ms - now) / (m_Tgo_ms - m_Tpg_ms));
  aux = aux + (now - m_Tpg_ms);
  if (aux < 25)
    {
      aux = 25;
    }
  if (aux > 1000)
    {
      aux = 1000;
    }
  m_Tgo_ms = m_Tpg_ms + aux;
  m_Toff_ms = aux;
  double elapsed = now - m_last_tx;
  if (m_queue_length > 0)
    {
      if (elapsed >= m_Toff_ms) checkQueue();
      else Simulator::Schedule(MilliSeconds(m_Toff_ms - elapsed), &DCC::checkQueue, this);
    }
}

bool DCC::checkGateOpen(float now)
{
  // Return true if the gate is open now
  bool ret = now - m_last_tx >= m_Toff_ms;
  return ret;
}

void DCC::updateTonpp(ssize_t pktSize)
{
  if (m_node != nullptr && m_node->GetNDevices () > 0)
    {
      Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (m_node->GetDevice (0));
      if (wifiDevice != nullptr && wifiDevice->GetPhy () != nullptr &&
          wifiDevice->GetRemoteStationManager () != nullptr)
        {
          WifiMacHeader broadcastData;
          broadcastData.SetType (WIFI_MAC_DATA);
          broadcastData.SetAddr1 (Mac48Address::GetBroadcast ());
          broadcastData.SetAddr2 (Mac48Address ("00:00:00:00:00:01"));
          broadcastData.SetAddr3 (Mac48Address::GetBroadcast ());

          WifiTxVector txVector = wifiDevice->GetRemoteStationManager ()->GetDataTxVector (broadcastData);
          Time txDuration = WifiPhy::CalculateTxDuration (static_cast<uint32_t> (pktSize),
                                                          txVector,
                                                          wifiDevice->GetPhy ()->GetPhyBand ());
          Time totalDuration = txDuration;
          if (txVector.IsNonNgv10 () && txVector.GetNgvPpduRepetitions () > 0)
            {
              const uint8_t repetitions = txVector.GetNgvPpduRepetitions ();
              totalDuration = txDuration * (1 + repetitions) + wifiDevice->GetPhy ()->GetSifs () * repetitions;
            }
          m_Ton_pp = static_cast<float> (totalDuration.GetSeconds () * 1000.0);
          return;
        }
    }

  double bits = pktSize * 8;
  double tx_duration_s = static_cast<double>(bits) / m_bitrate_bps;
  double total_duration_s = tx_duration_s + (68e-6); // 68 µs extra
  m_Ton_pp = total_duration_s * 1000.0;
}

void DCC::updateTgoAfterStateCheck(uint32_t Toff)
{
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  m_Tgo_ms = now + Toff;
  m_Toff_ms = Toff;
  float elapsed = now - m_last_tx;
  if (m_queue_length > 0)
    {
      if (elapsed >= m_Toff_ms) checkQueue();
      else Simulator::Schedule(MilliSeconds(m_Toff_ms - elapsed), &DCC::checkQueue, this);
    }
}

void
DCC::cleanQueues(float now)
{
  for(auto it = m_dcc_queue_dp0.begin(); it != m_dcc_queue_dp0.end();)
    {
      if (now > it->time + m_lifetime)
        {
          m_dropped_lifetime++;
          it = m_dcc_queue_dp0.erase(it);
        }
      else
        {
          ++it;
        }
    }

  for(auto it = m_dcc_queue_dp1.begin(); it != m_dcc_queue_dp1.end();)
    {
      if (now > it->time + m_lifetime)
        {
          m_dropped_lifetime++;
          it = m_dcc_queue_dp1.erase(it);
        }
      else
        {
          ++it;
        }
    }

  for(auto it = m_dcc_queue_dp2.begin(); it != m_dcc_queue_dp2.end();)
    {
      if (now > it->time + m_lifetime)
        {
          m_dropped_lifetime++;
          it = m_dcc_queue_dp2.erase(it);
        }
      else
        {
          ++it;
        }
    }

  for(auto it = m_dcc_queue_dp3.begin(); it != m_dcc_queue_dp3.end();)
    {
      if (now > it->time + m_lifetime)
        {
          m_dropped_lifetime++;
          it = m_dcc_queue_dp3.erase(it);
        }
      else
        {
          ++it;
        }
    }
}

void
DCC::enqueue(int priority, QueuePacket p)
{
  if (m_queue_length == 0)
    {
      // There is no queue, the pkt is directly discarded
      m_dropped_full_queue ++;
      return;
    }
  bool inserted = false;
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  cleanQueues(now);
  switch(priority)
    {
    case 0:
      if (m_dcc_queue_dp0.size() < m_queue_length)
        {
          // The queue is not full, we can accept a new packet
          p.priority = 0;
          m_dcc_queue_dp0.push_back(p);
          inserted = true;
        }
      break;
    case 1:
      if (m_dcc_queue_dp1.size()< m_queue_length)
        {
          p.priority = 1;
          m_dcc_queue_dp1.push_back(p);
          inserted = true;
        }
      break;
    case 2:
      if (m_dcc_queue_dp2.size() < m_queue_length)
        {
          p.priority = 2;
          m_dcc_queue_dp2.push_back(p);
          inserted = true;
        }
      break;
    case 3:
      if (m_dcc_queue_dp3.size() < m_queue_length)
        {
          p.priority = 3;
          m_dcc_queue_dp3.push_back(p);
          inserted = true;
        }
      break;
    }
  if (inserted && m_last_tx > 0)
    {
      auto now = static_cast<float>(Simulator::Now().GetMilliSeconds());
      float elapsed = now - m_last_tx;
      if (elapsed >= m_Toff_ms) checkQueue();
      else Simulator::Schedule(MilliSeconds(m_Toff_ms - elapsed), &DCC::checkQueue, this);
    }
}

std::tuple<bool, QueuePacket>
DCC::dequeue(int priority)
{
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  cleanQueues(now);
  QueuePacket pkt;
  bool found = false;

  if (m_dcc_queue_dp0.size() > 0 && priority >= 0)
    {
      pkt = std::move(m_dcc_queue_dp0.front());
      m_dcc_queue_dp0.erase(m_dcc_queue_dp0.begin());
      found = true;
    } else if (m_dcc_queue_dp1.size() > 0 && priority >= 1)
    {
      pkt = std::move(m_dcc_queue_dp1.front());
      m_dcc_queue_dp1.erase(m_dcc_queue_dp1.begin());
      found = true;
    } else if (m_dcc_queue_dp2.size() > 0 && priority >= 2)
    {
      pkt = std::move(m_dcc_queue_dp2.front());
      m_dcc_queue_dp2.erase(m_dcc_queue_dp2.begin());
      found = true;
    } else if (m_dcc_queue_dp3.size() > 0 && priority >= 3)
    {
      pkt = std::move(m_dcc_queue_dp3.front());
      m_dcc_queue_dp3.erase(m_dcc_queue_dp3.begin());
      found = true;
    }

  return {found, pkt};
}

void DCC::checkQueue ()
{
  float now = static_cast<float>(Simulator::Now().GetMilliSeconds());
  float elapsed = now - m_last_tx;

  if (elapsed >= m_Toff_ms)
    {
      // Gate is opened
      bool queues_have_packets = !(m_dcc_queue_dp0.empty() && m_dcc_queue_dp1.empty() && m_dcc_queue_dp2.empty() && m_dcc_queue_dp3.empty());
      if (queues_have_packets)
        {
          // Set priority = 4 (maximum is 3) so that the dequeue will check all the priority queues
          std::tuple<bool, QueuePacket> value = this->dequeue(4);
          bool status = std::get<0>(value);
          if (status)
            {
              QueuePacket pkt_to_send = std::get<1>(value);
              m_send_callback(pkt_to_send);
            }
        }
    }
  else
  {
    if (m_last_tx > 0)
      {
        float wait_time = m_Toff_ms - elapsed;
        if (wait_time <= 0) wait_time = 5;
        Simulator::Schedule(MilliSeconds(wait_time), &DCC::checkQueue, this);
      }
  }
}

void DCC::setSendCallback(std::function<void(const QueuePacket&)> cb)
{
  m_send_callback = std::move(cb);
}

void DCC::setCBRGCallback (std::function<void()> cb)
{
  m_cbr_g_callback = std::move(cb);
}

void DCC::updateAoI (int priority, double time)
{
  switch(priority)
    {
    case 0:
      m_packet_sent_dp0 ++;
      m_cumulative_time_dp0 = m_cumulative_time_dp0 + time;
      break;
    case 1:
      m_packet_sent_dp1 ++;
      m_cumulative_time_dp1 = m_cumulative_time_dp1 + time;
      break;
    case 2:
      m_packet_sent_dp2 ++;
      m_cumulative_time_dp2 = m_cumulative_time_dp2 + time;
      break;
    case 3:
      m_packet_sent_dp3 ++;
      m_cumulative_time_dp3 = m_cumulative_time_dp3 + time;
      break;
    }
}

}
