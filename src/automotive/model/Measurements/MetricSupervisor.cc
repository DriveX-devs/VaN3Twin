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
    *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
    *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
    *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
    *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include "MetricSupervisor.h"
#include "ns3/csv-utils.h"
#include <sstream>
#include <cfloat>

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

        namespace {
  double MetricSupervisor_haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
    // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
    return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  }
}

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("MetricSupervisor");

std::unordered_map<std::string, Time> currentBusyCBR;
std::unordered_map<std::string, std::pair<Time, WifiPhyState>> nodeLastState80211p;
std::unordered_map<std::string, std::vector<std::pair<Time, Time>>> nodeBusyIntervals80211p;
std::unordered_map<std::string, Time> nodeDurationStateNr;
Time lastCBRCheck = Time(-1.0);

TypeId
MetricSupervisor::GetTypeId ()
{
  static TypeId tid = TypeId("ns3::MetricSupervisor")
                          .SetParent <Object>()
                          .AddConstructor <MetricSupervisor>();
  return tid;
}

MetricSupervisor::~MetricSupervisor ()
{
  NS_LOG_FUNCTION(this);

  std::list<EventId>::iterator eventit = eventList.begin();
  while(eventit != eventList.end())
    {
      if(eventit->IsExpired() == false)
        {
          Simulator::Cancel(*eventit);
        }
      eventit = eventList.erase(eventit);
    }
}

std::string
MetricSupervisor::bufToString(uint8_t *buf, uint32_t bufsize)
{
  std::stringstream bufss;

  bufss << std::hex << std::setfill('0');

  for(size_t i=0;i<bufsize;++i)
    {
      bufss << std::setw(2) << static_cast<unsigned>(buf[i]);
    }

  return bufss.str();
}

void
MetricSupervisor::signalSentPacket(std::string buf, double lat, double lon, uint64_t nodeID, messageType_e messagetype)
{
  EventId computePRR_id;

  if(m_traci_ptr != nullptr)
    {

      // If the packet is sent by an excluded vehicle due to a problem in the configuration of the simulation, ignore it
      if (m_excluded_vehID_enabled == true &&
          (m_excluded_vehID_list.find (nodeID) != m_excluded_vehID_list.end ()))
        {
          return;
        }

      //std::vector<std::string> ids = m_traci_ptr->TraCIAPI::vehicle.getIDList ();
      std::map<std::string, std::pair<StationType_t, Ptr<Node>>> node_map =
          m_traci_ptr->get_NodeMap ();

      for (std::map<std::string, std::pair<StationType_t, Ptr<Node>>>::iterator it =
               node_map.begin ();
           it != node_map.end (); ++it)
        {
          StationType_t station_type = it->second.first;
          uint64_t stationID;
          try
            {
              if (station_type == StationType_roadSideUnit)
                {
                  // Handle both "rsu_<N>" and "rsu<N>" ID formats
                  std::string id_str;
                  std::size_t underscore_pos = it->first.find ("_");
                  if (underscore_pos != std::string::npos)
                    {
                      id_str = it->first.substr (underscore_pos + 1);
                    }
                  else if (it->first.size () > 3 && it->first.substr (0, 3) == "rsu")
                    {
                      id_str = it->first.substr (3);
                    }
                  else
                    {
                      id_str = it->first;
                    }
                  stationID = m_stationId_baseline + std::stoi (id_str);
                }
              else
                {
                  stationID = std::stol (it->first.substr (3));
                }
            }
          catch (const std::exception &e)
            {
              NS_LOG_WARN ("MetricSupervisor: failed to parse station ID '"
                           << it->first << "': " << e.what () << ", skipping");
              continue;
            }

          libsumo::TraCIPosition pos;
          if (station_type == StationType_pedestrian)
            {
              pos = m_traci_ptr->TraCIAPI::person.getPosition (it->first);
            }
          else if (station_type == StationType_roadSideUnit)
            {
              // Try POI first, fall back to vehicle
              try
                {
                  pos = m_traci_ptr->TraCIAPI::poi.getPosition (it->first);
                }
              catch (const libsumo::TraCIException &)
                {
                  pos = m_traci_ptr->TraCIAPI::vehicle.getPosition (it->first);
                }
            }
          else
            {
              pos = m_traci_ptr->TraCIAPI::vehicle.getPosition (it->first);
            }
          pos = m_traci_ptr->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

          if (stationID == nodeID)
            m_stationtype_map[buf] = station_type;

          if (m_excluded_vehID_enabled == false ||
              (m_excluded_vehID_list.find (stationID) == m_excluded_vehID_list.end ()))
            {
              if (MetricSupervisor_haversineDist (lat, lon, pos.y, pos.x) <= m_baseline_m)
                {
                  m_packetbuff_map[buf].nodeList.push_back (stationID);
                }
            }
        }
    }
  else if(m_carla_ptr != nullptr)
    {

      // If the packet is sent by an excluded vehicle due to a problem in the configuration of the simulation, ignore it
      if (m_excluded_vehID_enabled == true &&
          (m_excluded_vehID_list.find (nodeID) != m_excluded_vehID_list.end ()))
        {
          return;
        }


      std::map<std::basic_string<char>, std::basic_string<char>> ids = m_carla_ptr->getManagedConnectedNodes();
      for (auto it = ids.begin(); it != ids.end(); ++it)
        {
          std::string stationID = it->first;

          carla::Vehicle vehicle = m_carla_ptr->GetManagedActorById(std::stoi(stationID));

          if (std::stoi(stationID) == nodeID)
            m_stationtype_map[buf] = StationType_passengerCar;

          if(m_excluded_vehID_enabled==false || (m_excluded_vehID_list.find(std::stoi(stationID))==m_excluded_vehID_list.end())) {
              if(MetricSupervisor_haversineDist(lat,lon,vehicle.latitude (),vehicle.longitude ())<=m_baseline_m)
                {
                  m_packetbuff_map[buf].nodeList.push_back(std::stoi(stationID));
                }
            }
        }
    }
  else
    {
      // TODO handle the PRR for gps-tc module
      if (!m_v_gpstc.empty()) return;
      NS_FATAL_ERROR("Fatal error: mobility client not set in PRR Supervisor.");
    }

  computePRR_id = Simulator::Schedule(MilliSeconds (m_pprcomp_timeout*1000.0), &MetricSupervisor::computePRR, this, buf);
  eventList.push_back (computePRR_id);

  m_latency_map[buf] = Simulator::Now ().GetNanoSeconds ();

  m_id_map[buf] = nodeID;

  m_messagetype_map[buf] = messagetype;

  if(m_stationtype_map[buf]==StationType_pedestrian) {
      m_ntx_per_ped[nodeID]++;
  } else if(m_stationtype_map[buf]==StationType_roadSideUnit) {
      m_ntx_per_rsu[nodeID]++;
  } else {
      m_ntx_per_veh[nodeID]++;
  }

  m_total_tx++;
  m_ntx_per_messagetype[messagetype]++;
}

void
MetricSupervisor::signalReceivedPacket(std::string buf, uint64_t nodeID)
{
  baselineVehicleData_t currBaselineData;
  double curr_latency_ms = DBL_MAX;

  if(m_traci_ptr == nullptr && m_carla_ptr == nullptr)
    {
      // TODO handle the PRR for gps-tc module
      if (!m_v_gpstc.empty()) return;
      NS_FATAL_ERROR("Fatal error: mobility client not set in PRR Supervisor.");
    }

  // If the packet was received by an excluded vehicle due to a problem in the configuration of the simulation, ignore it
  if(m_excluded_vehID_enabled==true && (m_excluded_vehID_list.find(nodeID)!=m_excluded_vehID_list.end()))
    {
      return;
    }

  // If the packet was sent by an excluded vehicle due to a problem in the configuration of the simulation, it will be automatically
  // ignored as it will not be in the m_packetbuff_map and m_packetbuff_map.count(buf)==0
  if(m_packetbuff_map.count(buf)>0)
    {
      currBaselineData = m_packetbuff_map[buf];

      if(std::find(currBaselineData.nodeList.begin(), currBaselineData.nodeList.end(), nodeID) != currBaselineData.nodeList.end())
        {
          (m_packetbuff_map[buf].x)++;
        }
    }

  messageType_e messagetype = m_messagetype_map[buf];
  StationType_t station_type = m_stationtype_map[buf];
  uint64_t senderID = m_id_map[buf];

  // Compute latency in ms
  if(m_latency_map.count(buf)>0)
    {
      curr_latency_ms = static_cast<double>(Simulator::Now ().GetNanoSeconds () - m_latency_map[buf])/1000000.0;
      m_count_latency++;

      m_avg_latency_ms += (curr_latency_ms-m_avg_latency_ms)/m_count_latency;

      if(station_type == StationType_pedestrian){

          if(m_count_latency_per_ped.count(senderID)<=0) {
              m_count_latency_per_ped[senderID]=0;
            }

          if(m_avg_latency_ms_per_ped.count(senderID)<=0) {
              m_avg_latency_ms_per_ped[senderID]=0;
            }

          m_count_latency_per_ped[senderID]++;
          m_avg_latency_ms_per_ped[senderID] += (curr_latency_ms - m_avg_latency_ms_per_ped[senderID])/m_count_latency_per_ped[senderID];

        }
      else if(station_type == StationType_roadSideUnit)
        {

          if (m_count_latency_per_rsu.count (senderID) <= 0)
            {
              m_count_latency_per_rsu[senderID] = 0;
            }

          if (m_avg_latency_ms_per_rsu.count (senderID) <= 0)
            {
              m_avg_latency_ms_per_rsu[senderID] = 0;
            }

          m_count_latency_per_rsu[senderID]++;
          m_avg_latency_ms_per_rsu[senderID] +=
              (curr_latency_ms - m_avg_latency_ms_per_rsu[senderID]) /
              m_count_latency_per_rsu[senderID];
        }
      else{
          if(m_count_latency_per_veh.count(senderID)<=0) {
              m_count_latency_per_veh[senderID]=0;
            }

          if(m_avg_latency_ms_per_veh.count(senderID)<=0) {
              m_avg_latency_ms_per_veh[senderID]=0;
            }

          m_count_latency_per_veh[senderID]++;
          m_avg_latency_ms_per_veh[senderID] += (curr_latency_ms - m_avg_latency_ms_per_veh[senderID])/m_count_latency_per_veh[senderID];
        }

      if(m_count_latency_per_messagetype.count(messagetype)<=0) {
          m_count_latency_per_messagetype[messagetype]=0;
        }

      if(m_avg_latency_ms_per_messagetype.count(messagetype)<=0) {
          m_avg_latency_ms_per_messagetype[messagetype]=0;
        }

      m_count_latency_per_messagetype[messagetype]++;
      m_avg_latency_ms_per_messagetype[messagetype] += (curr_latency_ms - m_avg_latency_ms_per_messagetype[messagetype])/m_count_latency_per_messagetype[messagetype];

      if(m_prr_verbose_stdout == true) {
          std::cout << "|Latency| ID: " << nodeID << " Current: " << curr_latency_ms << " - Average: " << m_avg_latency_ms << std::endl;
        }
    }

  m_total_rx++;
  m_nrx_per_messagetype[m_messagetype_map[buf]]++;

  if(m_stationtype_map[buf]==StationType_pedestrian) {
    m_nrx_per_ped[nodeID]++;
  } else if(m_stationtype_map[buf]==StationType_roadSideUnit) {
    m_nrx_per_rsu[nodeID]++;
  } else {
    m_nrx_per_veh[nodeID]++;
  }
}

void
MetricSupervisor::computePRR(std::string buf)
{
  double PRR = 0.0;

  if(m_packetbuff_map[buf].nodeList.size()>1)
    {
      uint64_t senderID = m_id_map[buf];
      messageType_e messagetype = m_messagetype_map[buf];
      StationType_t station_type = m_stationtype_map[buf];

      // Number of vehicles/other road users in the baseline ("Y" in the PRR formula)
      double nvehbsln = (double) (m_packetbuff_map[buf].nodeList.size())-1.0;

      if (station_type == StationType_pedestrian){
        if(m_count_nvehbsln_per_ped.count(senderID)<=0) {
            m_count_nvehbsln_per_ped[senderID]=0;
        }

        m_count_nvehbsln_per_ped[senderID]++;
        m_avg_nvehbsln_per_ped[senderID] += (nvehbsln-m_avg_nvehbsln_per_ped[senderID]) / static_cast<double>(m_count_nvehbsln_per_ped[senderID]);
      } else if(station_type == StationType_roadSideUnit) {
        if(m_count_nvehbsln_per_rsu.count(senderID)<=0) {
            m_count_nvehbsln_per_rsu[senderID]=0;
        }

        m_count_nvehbsln_per_rsu[senderID]++;
        m_avg_nvehbsln_per_rsu[senderID] += (nvehbsln-m_avg_nvehbsln_per_rsu[senderID]) / static_cast<double>(m_count_nvehbsln_per_rsu[senderID]);
      } else {
        if(m_count_nvehbsln_per_veh.count(senderID)<=0) {
            m_count_nvehbsln_per_veh[senderID]=0;
        }

        m_count_nvehbsln_per_veh[senderID]++;
        m_avg_nvehbsln_per_veh[senderID] += (nvehbsln-m_avg_nvehbsln_per_veh[senderID]) / static_cast<double>(m_count_nvehbsln_per_veh[senderID]);

        if(m_prr_verbose_stdout == true) {
          std::cout << "|Number of vehicles in the baseline| Vehicle ID: " << senderID << " Current: " << nvehbsln << " - Average: " << m_avg_nvehbsln_per_veh[senderID] << std::endl;
        }
      }

      if(m_avg_nvehbsln_per_messagetype.count(messagetype)<=0) {
          m_avg_nvehbsln_per_messagetype[messagetype]=0;
      }

      m_count_nvehbsln_per_messagetype[messagetype]++;
      m_avg_nvehbsln_per_messagetype[messagetype] += (nvehbsln-m_avg_nvehbsln_per_messagetype[messagetype]) / static_cast<double>(m_count_nvehbsln_per_messagetype[messagetype]);

      PRR = (double) m_packetbuff_map[buf].x/nvehbsln;

      if(PRR>1) {
          std::cerr << "Value of X: " << (double) m_packetbuff_map[buf].x << " - value of Y: " << (double) (m_packetbuff_map[buf].nodeList.size()-1.0) << std::endl;
          NS_FATAL_ERROR ("Error. Computed a PRR greater than 1. This is not possible. Please check how you configured your simulation and the MetricSupervisor.");
        }

      m_count++;
      m_avg_PRR += (PRR-m_avg_PRR)/m_count;

      if(m_prr_verbose_stdout) {
          std::cout << "|PRR| Current: " << PRR << " - Average: " << m_avg_PRR << std::endl;
        }

      if (station_type == StationType_pedestrian){
          if(m_count_per_ped.count(senderID)<=0) {
              m_count_per_ped[senderID]=0;
            }

          if(m_avg_PRR_per_ped.count(senderID)<=0) {
              m_avg_PRR_per_ped[senderID]=0;
            }

          m_count_per_ped[senderID]++;
          m_avg_PRR_per_ped[senderID] += (PRR-m_avg_PRR_per_ped[senderID])/m_count_per_ped[senderID];
        }
      else if(station_type == StationType_roadSideUnit)
        {
          if (m_count_per_rsu.count (senderID) <= 0)
            {
              m_count_per_rsu[senderID] = 0;
            }

          if (m_avg_PRR_per_rsu.count (senderID) <= 0)
            {
              m_avg_PRR_per_rsu[senderID] = 0;
            }

          m_count_per_rsu[senderID]++;
          m_avg_PRR_per_rsu[senderID] +=
              (PRR - m_avg_PRR_per_rsu[senderID]) / m_count_per_rsu[senderID];
        }
      else
        {
          if(m_count_per_veh.count(senderID)<=0) {
              m_count_per_veh[senderID]=0;
            }

          if(m_avg_PRR_per_veh.count(senderID)<=0) {
              m_avg_PRR_per_veh[senderID]=0;
            }

          m_count_per_veh[senderID]++;
          m_avg_PRR_per_veh[senderID] += (PRR-m_avg_PRR_per_veh[senderID])/m_count_per_veh[senderID];
        }

      if(m_count_per_messagetype.count(messagetype)<=0) {
          m_count_per_messagetype[messagetype]=0;
        }

      if(m_avg_PRR_per_messagetype.count(messagetype)<=0) {
          m_avg_PRR_per_messagetype[messagetype]=0;
        }

      m_count_per_messagetype[messagetype]++;
      m_avg_PRR_per_messagetype[messagetype] += (PRR-m_avg_PRR_per_messagetype[messagetype])/m_count_per_messagetype[messagetype];

      m_packetbuff_map.erase(buf);
      m_id_map.erase(buf);
      m_messagetype_map.erase(buf);
    }

  // Some time has passed -> remove the packet from the latency map too
  m_latency_map.erase(buf);
}

bool IsChannelBusy(WifiPhyState state) {
  return state != WifiPhyState::SLEEP && state != WifiPhyState::IDLE;
}

bool IsWaveCbrTechnology(const std::string& technology)
{
  return technology == "80211p" || technology == "80211bd";
}

void
RefreshWaveBusyCbrForCurrentWindow ()
{
  Time windowStart = lastCBRCheck.IsNegative () ? Seconds (0) : lastCBRCheck;
  Time windowEnd = Simulator::Now ();
  currentBusyCBR.clear ();

  for (auto it = nodeBusyIntervals80211p.begin (); it != nodeBusyIntervals80211p.end ();)
    {
      Time busyCbr = Seconds (0);
      std::vector<std::pair<Time, Time>> remainingIntervals;

      for (const auto& interval : it->second)
        {
          Time intervalStart = interval.first;
          Time intervalEnd = interval.second;
          if (intervalEnd > windowStart && intervalStart < windowEnd)
            {
              Time clippedStart = intervalStart < windowStart ? windowStart : intervalStart;
              Time clippedEnd = intervalEnd > windowEnd ? windowEnd : intervalEnd;
              if (clippedEnd > clippedStart)
                {
                  busyCbr += clippedEnd - clippedStart;
                }
            }

          if (intervalEnd > windowEnd)
            {
              remainingIntervals.push_back (interval);
            }
        }

      currentBusyCBR[it->first] = busyCbr;
      if (remainingIntervals.empty ())
        {
          it = nodeBusyIntervals80211p.erase (it);
        }
      else
        {
          it->second = remainingIntervals;
          ++it;
        }
    }
}

void
storeCBR80211p (std::string context, Time start, Time duration, WifiPhyState state)
{
  // End and start are expressed in ns
  // In this case Duration is the time the channel was in the specific state (referred to the past)
  std::size_t first = context.find ("/NodeList/") + 10; // 10 is the length of "/NodeList/"
  std::size_t last = context.find ("/", first);
  std::string node = context.substr (first, last - first);

  if (IsChannelBusy (state))
    {
      Time end = start + duration;
      if (end > start)
        {
          nodeBusyIntervals80211p[node].push_back (std::make_pair (start, end));
        }
      // Storing the current state and the current time is essential
      // Situations where the next CBR check happens before the end of the current busy state must be handled with this method
      nodeLastState80211p[node].first = Simulator::Now();
      nodeLastState80211p[node].second = WifiPhyState::CCA_BUSY;
    }
  else
    {
      if (currentBusyCBR.find(node) == currentBusyCBR.end())
        {
          currentBusyCBR[node] = Time(0);
        }
      // Skip the part for the updating of CBR, because doing += Time(0) would be useless
      nodeLastState80211p[node].first = Simulator::Now();
      nodeLastState80211p[node].second = WifiPhyState::IDLE;
    }
}

void
storeCBRNr(std::string context, Time duration)
{
  // In this case Duration is the time the channel will be in a busy state (referred to the future)
  std::size_t first = context.find ("/NodeList/") + 10; // 10 is the length of "/NodeList/"
  std::size_t last = context.find ("/", first);
  std::string node = context.substr (first, last - first);

  // How long the state will last for the other nodes?
  // This management will be useful when the CheckCBR function will start (see below)
  for (auto it = nodeDurationStateNr.begin(); it != nodeDurationStateNr.end(); ++it)
    {
      if (it->first != node) nodeDurationStateNr[it->first] = Simulator::Now() + duration;
    }

  for (auto it = currentBusyCBR.begin(); it != currentBusyCBR.end(); ++it)
    {
      if (it->first != node) it->second += duration;
    }

}

void
MetricSupervisor::checkCBR ()
{

  std::unordered_map<std::string, Time> nextTimeToAddNr;
  if (IsWaveCbrTechnology (m_channel_technology))
    {
      RefreshWaveBusyCbrForCurrentWindow ();
      for (uint32_t i = 0; i < m_node_container.GetN (); ++i)
        {
          const std::string node_id = std::to_string (m_node_container.Get (i)->GetId ());
          if (currentBusyCBR.find (node_id) == currentBusyCBR.end ())
            {
              currentBusyCBR[node_id] = Seconds (0);
            }
        }
    }
  if(m_traci_ptr != nullptr)
    {
      std::map<std::basic_string<char>, std::pair<StationType_t, Ptr<Node>>> nodes = m_traci_ptr->get_NodeMap ();

      for (auto it = nodes.begin (); it != nodes.end (); ++it)
        {
          std::string item = it->first;

          std::basic_string<char> node_id = std::to_string (it->second.second->GetId ());

          if (currentBusyCBR.find (node_id) == currentBusyCBR.end ())
            {
              continue;
            }

          Time busyCbr = currentBusyCBR[node_id];

          // We are in the middle of a busy state
          /*
          if (m_channel_technology == "80211p" &&
              nodeLastState80211p[node_id].second == WifiPhyState::CCA_BUSY)
            {
              // 80211p duration refers to the past time the channel was busy
              // We need to add the time from the last check if the state is still busy
              busyCbr += Simulator::Now () - nodeLastState80211p[node_id].first;
            }
            */

          if (m_channel_technology == "Nr")
            {
              // NR duration refers to the future time the channel will be busy due to resource allocation for a certain node
              // We need to subtract the time that the channel will be busy for this node after this current check
              // This time will be added for the next check (see below)
              if (nodeDurationStateNr[node_id] > Simulator::Now ())
                {
                  Time nextToAdd = nodeDurationStateNr[node_id] - Simulator::Now ();
                  busyCbr -= nextToAdd;
                  nextTimeToAddNr[node_id] = nextToAdd;
                }
            }

          double currentCbr = busyCbr.GetDouble () / (m_cbr_window * 1e6);
          if (false) std::cout << Simulator::Now().GetSeconds() << "s - Node " << node_id << " - CBR: " << 100 * currentCbr << std::endl;

          if (m_average_cbr.find (item) != m_average_cbr.end ())
            {
              // Exponential moving average
              double new_cbr = m_cbr_alpha * m_average_cbr[item].back () + (1 - m_cbr_alpha) * currentCbr;
              m_average_cbr[item].push_back (new_cbr);
            }
          else
            {
              m_average_cbr[item].push_back (currentCbr);
            }
        }
    }
  else if (m_carla_ptr != nullptr)
    {
      std::map<std::string,std::string> obj_node_map = m_carla_ptr->getManagedConnectedNodes();

      for (const auto& pair : obj_node_map)
        {
          std::string node_id = pair.second;
          std::string obj_id = pair.first;
          // std::string node_id = std::to_string (item);

          if (currentBusyCBR.find (node_id) == currentBusyCBR.end ())
            {
              continue;
            }

          Time busyCbr = currentBusyCBR[node_id];

          // We are in the middle of a busy state
          /*
          if (m_channel_technology == "80211p" &&
              nodeLastState80211p[node_id].second == WifiPhyState::CCA_BUSY)
            {
              // 80211p duration refers to the past time the channel was busy
              // We need to add the time from the last check if the state is still busy
              busyCbr += Simulator::Now () - nodeLastState80211p[node_id].first;
            }
            */

          if (m_channel_technology == "Nr")
            {
              // NR duration refers to the future time the channel will be busy due to resource allocation for a certain node
              // We need to subtract the time that the channel will be busy for this node after this current check
              // This time will be added for the next check (see below)
              if (nodeDurationStateNr[node_id] > Simulator::Now ())
                {
                  Time nextToAdd = nodeDurationStateNr[node_id] - Simulator::Now ();
                  busyCbr -= nextToAdd;
                  nextTimeToAddNr[node_id] = nextToAdd;
                }
            }

          double currentCbr = busyCbr.GetDouble () / (m_cbr_window * 1e6);

          if (m_average_cbr.find (obj_id) != m_average_cbr.end ())
            {
              // Exponential moving average
              double new_cbr =
                  m_cbr_alpha * m_average_cbr[obj_id].back () + (1 - m_cbr_alpha) * currentCbr;
              m_average_cbr[obj_id].push_back (new_cbr);
            }
          else
            {
              m_average_cbr[obj_id].push_back (currentCbr);
            }
        }
    }
  else if (!m_v_gpstc.empty())
    {
      for (auto gpstc_ptr = m_v_gpstc.begin(); gpstc_ptr != m_v_gpstc.end(); ++gpstc_ptr)
        {
          if (currentBusyCBR.find ((*gpstc_ptr)->getNodeID()) != currentBusyCBR.end ())
            {
              Time busyCbr = currentBusyCBR[(*gpstc_ptr)->getNodeID()];

              if (m_channel_technology == "Nr")
                {
                  // NR duration refers to the future time the channel will be busy due to resource allocation for a certain node
                  // We need to subtract the time that the channel will be busy for this node after this current check
                  // This time will be added for the next check (see below)
                  if (nodeDurationStateNr[(*gpstc_ptr)->getNodeID()] > Simulator::Now ())
                    {
                      Time nextToAdd = nodeDurationStateNr[(*gpstc_ptr)->getNodeID()] - Simulator::Now ();
                      busyCbr -= nextToAdd;
                      nextTimeToAddNr[(*gpstc_ptr)->getNodeID()] = nextToAdd;
                    }
                }

              double currentCbr = busyCbr.GetDouble () / (m_cbr_window * 1e6);

              if (m_average_cbr.find ((*gpstc_ptr)->getID()) != m_average_cbr.end ())
                {
                  // Exponential moving average
                  double new_cbr =
                      m_cbr_alpha * m_average_cbr[(*gpstc_ptr)->getID()].back () + (1 - m_cbr_alpha) * currentCbr;
                  m_average_cbr[(*gpstc_ptr)->getID()].push_back (new_cbr);
                }
              else
                {
                  m_average_cbr[(*gpstc_ptr)->getID()].push_back (currentCbr);
                }
            }
        }
    }
  else
    {
      for (uint32_t i = 0; i < m_node_container.GetN (); ++i)
        {
          std::string node_id = std::to_string (m_node_container.Get (i)->GetId ());

          if (currentBusyCBR.find (node_id) == currentBusyCBR.end ())
            {
              continue;
            }

          Time busyCbr = currentBusyCBR[node_id];

          if (m_channel_technology == "Nr")
            {
              if (nodeDurationStateNr[node_id] > Simulator::Now ())
                {
                  Time nextToAdd = nodeDurationStateNr[node_id] - Simulator::Now ();
                  busyCbr -= nextToAdd;
                  nextTimeToAddNr[node_id] = nextToAdd;
                }
            }

          double currentCbr = busyCbr.GetDouble () / (m_cbr_window * 1e6);

          if (m_average_cbr.find (node_id) != m_average_cbr.end ())
            {
              double new_cbr =
                  m_cbr_alpha * m_average_cbr[node_id].back () + (1 - m_cbr_alpha) * currentCbr;
              m_average_cbr[node_id].push_back (new_cbr);
            }
          else
            {
              m_average_cbr[node_id].push_back (currentCbr);
            }
        }
    }
  currentBusyCBR.clear();
  if(IsWaveCbrTechnology(m_channel_technology)) nodeLastState80211p.clear();
  if(m_channel_technology == "Nr")
    {
      nodeDurationStateNr.clear();
      for(auto it : nextTimeToAddNr)
        {
          currentBusyCBR[it.first] = it.second;
        }
    }

  lastCBRCheck = Simulator::Now();

  // If enabled, log each CBR value to a CSV file
  if (m_cbr_csv_log_file!="") {
      for (auto it = m_average_cbr.begin(); it != m_average_cbr.end(); ++it) {
          if (!it->second.empty()) {
              writeDataToCSV(m_cbr_csv_log_file, "timestamp_ms,vehicle_id,CBR,CBR_time_window",
                static_cast<double>(Simulator::Now().GetNanoSeconds())/1000000.0,it->first,it->second.back(),m_cbr_window);
          }
      }
  }

  Simulator::Schedule (MilliSeconds (m_cbr_window), &MetricSupervisor::checkCBR, this);
}

void
MetricSupervisor::logLastCBRs ()
{
  if (m_cbr_verbose_stdout)
    {
      std::ofstream file;
      std::cout << "CBR last values for each node:" << std::endl;
      if (m_cbr_write_to_file)
        {
          file.open ("cbr_values.txt", std::ios_base::out);
          file << "CBR last values for each node:" << std::endl;
        }
      for (auto it = m_average_cbr.begin (); it !=m_average_cbr.end (); ++it)
        {
          std::string node = it->first;
          if (it->second.empty ())
            {
              continue;
            }
          double cbr = it->second.back();
          std::cout << "Node " << node << ": " << std::fixed << std::setprecision(2) << cbr * 100 << "%" << std::endl;
          if (m_cbr_write_to_file)
            {
              file << "Node " << node << ": " << std::fixed << std::setprecision(2) << cbr * 100 << "%" << std::endl;
            }
        }
      if (m_cbr_write_to_file)
        {
          file.close ();
        }
    }
}

void
MetricSupervisor::startCheckCBR (int num_nodes)
{
  // Assert that the parameters for the CBR are set
  NS_ASSERT_MSG(m_cbr_window > 0, "CBR window must be greater than 0");
  NS_ASSERT_MSG(m_cbr_alpha >= 0 && m_cbr_alpha <= 1, "CBR alpha must be between 0 and 1");
  NS_ASSERT_MSG (m_channel_technology != "", "Channel technology must be set, choose between 80211p, 80211bd, and Nr");
  NS_ASSERT_MSG (m_simulation_time > 0, "Simulation time must be greater than 0");

  NS_ASSERT_MSG (m_node_container.GetN() != 0, "The Node container must be filled before the CBR checking.");
  currentBusyCBR.clear ();
  nodeLastState80211p.clear ();
  nodeBusyIntervals80211p.clear ();
  nodeDurationStateNr.clear ();
  lastCBRCheck = Time (-1.0);

  uint8_t i = 0;
  uint8_t nodes_to_check = num_nodes == -1 ? m_node_container.GetN() : num_nodes;
  for(; i < nodes_to_check; i++)
    {
      Ptr<Node> node = m_node_container.Get (i);
      std::basic_ostringstream<char> oss;
      if (IsWaveCbrTechnology (m_channel_technology))
        {
          oss << "/NodeList/" << node->GetId() << "/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State";
          std::string var = oss.str();
          Config::Connect(var, MakeCallback(&storeCBR80211p));
        }
      else if (m_channel_technology == "Nr")
        {
          oss << "/NodeList/" << node->GetId() << "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/NrSpectrumPhyList/*/ChannelOccupied";
          std::string var = oss.str();
          Config::Connect(var, MakeCallback(&storeCBRNr));
          currentBusyCBR[std::to_string (node->GetId())] = Time(0);
          nodeDurationStateNr[std::to_string (node->GetId())] = Time(0);
        }
    }

  Simulator::Schedule (MilliSeconds(m_cbr_window), &MetricSupervisor::checkCBR, this);
  Simulator::Schedule (Seconds (m_simulation_time), &MetricSupervisor::logLastCBRs, this);

}

double
MetricSupervisor::getCBRPerItem (std::string itemID)
{
  if (m_average_cbr.find (itemID) != m_average_cbr.end ())
    {
      return m_average_cbr[itemID].back();
    } else {
      return -1.0;
    }
}

float MetricSupervisor::getAverageCBROverall ()
{
  float sum = 0;
  int count = 0;
  for (auto it = m_average_cbr.begin (); it != m_average_cbr.end (); ++it)
    {
      if (it->second.empty ())
        {
          continue;
        }
      sum += it->second.back ();
      count++;
    }
  return sum / count;
}


std::unordered_map<std::string, std::vector<double>> MetricSupervisor::getCBRValues()
{
  std::unordered_map<std::string, std::vector<double>> cbr_values = m_average_cbr;
  return cbr_values;
}


void
MetricSupervisor::channelOccupationBytesPerSecondsPerSquareMeter ()
{
  if(m_traci_ptr != nullptr)
    {
      if (m_total_area == 0)
        {
          std::vector<std::basic_string<char>> lanes = m_traci_ptr->TraCIAPI::lane.getIDList ();
          double totalArea = 0;
          for (auto it : lanes)
            {
              double length = m_traci_ptr->TraCIAPI::lane.getLength (it);
              double weight = m_traci_ptr->TraCIAPI::lane.getWidth (it);
              totalArea += length * weight;
            }

          m_total_area = totalArea;
        }
    }
  else if (m_carla_ptr != nullptr)
    {
      if (m_total_area == 0)
        {
          std::map<std::basic_string<char>, std::basic_string<char>> ids = m_carla_ptr->getManagedConnectedNodes ();
          double totalArea = 0;
          for (auto it = ids.begin(); it != ids.end(); ++it)
            {
              std::string stationID = it->first;
              carla::Vehicle vehicle = m_carla_ptr->GetManagedActorById(std::stoi (stationID));
              totalArea += vehicle.length () * vehicle.width ();
            }

          m_total_area = totalArea;
        }
    }
  else
    {
      NS_FATAL_ERROR("Fatal error: mobility client not set in Metric Supervisor.");
    }
  uint32_t bytes = m_total_bytes;
  uint64_t receivedBytesWithinTimeWindow;
  if (m_received_bytes_checkpoint != 0)
    {
      receivedBytesWithinTimeWindow = bytes - m_received_bytes_checkpoint;
    }
  else
    {
      receivedBytesWithinTimeWindow = bytes;
    }
  m_received_bytes_checkpoint = bytes;

  double result = (double) receivedBytesWithinTimeWindow / (m_total_area * m_channel_window);

  m_bytes_per_second_per_square_meter.push_back(result);

  double cbr = getAverageCBROverall();

  Simulator::Schedule(Seconds(m_channel_window), &MetricSupervisor::channelOccupationBytesPerSecondsPerSquareMeter, this);

  if (cbr > 0.0)
    {
      m_cbr_bytes_per_second_per_square_meter.push_back(std::make_tuple (cbr, result));
    }

}


} // namespace ns3
