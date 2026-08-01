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
 *
 * Created by:
 *   Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
 */

#include "txTracker.h" // Include the header file defining the TxTracker class and related methods

namespace ns3 {

// Define the logging component for this module
NS_LOG_COMPONENT_DEFINE ("TxTracker");

// Helper function to convert dBm to Watts
double DbmToW (double dbm) {
  return std::pow(10, (dbm - 30) / 10);
}

// Helper function to convert Watts to dBm
double WToDbm (double w) {
  return 10 * std::log10(w) + 30;
}

uint8_t calculateNumRb (double lteBandwidthHz, double rbOh, uint8_t numerology) {
  double realBw = lteBandwidthHz * (1 - rbOh);
  uint32_t subcarrierSpacing = 15000 * static_cast<uint32_t> (std::pow (2, numerology));
  uint32_t rbWidth = subcarrierSpacing * NrSpectrumValueHelper::SUBCARRIERS_PER_RB;
  return static_cast<uint32_t> (realBw / rbWidth);
}

// Insert 802.11p nodes into the tracker
void
TxTracker::Insert11pNodes (std::vector<std::tuple<std::string, uint8_t, Ptr<WifiNetDevice>>> nodes) {
  for (auto n : nodes) {
      // Extract vehicle ID, node ID, and network device from the tuple
      std::string vehID = std::get<0>(n);
      uint8_t nodeID = std::get<1>(n);
      Ptr<WifiNetDevice> netDevice = std::get<2>(n);

      // Ensure the network device is valid
      NS_ASSERT_MSG (netDevice != nullptr, "Node is nullptr");

      // Get the physical layer (PHY) of the Wi-Fi device
      Ptr<WifiPhy> wifiPhy = netDevice->GetPhy();

      // Store the node's transmission parameters in the tracker
      m_txMap11p[vehID] = txParameters11p {
          nodeID,
          netDevice,
          std::pow(10, (wifiPhy->GetTxPowerStart() - 30) / 10), // Convert transmission power from dBm to Watts
      };
    }
}

// Insert NR (New Radio) nodes into the tracker
void
TxTracker::InsertNrNodes (std::vector<std::tuple<std::string, uint8_t, Ptr<NrUeNetDevice>>> nodes) {
  for (auto n : nodes) {
      // Extract vehicle ID, node ID, and network device from the tuple
      std::string vehID = std::get<0>(n);
      uint8_t nodeID = std::get<1>(n);
      Ptr<NrUeNetDevice> netDevice = std::get<2>(n);

      // Ensure the network device is valid
      NS_ASSERT_MSG (netDevice != nullptr, "Node is nullptr");

      // Get the physical layer (PHY) of the NR device and calculate RB (Resource Block) bandwidth
      Ptr<NrUePhy> uePhy = netDevice->GetPhy (0);
      double rbBand = m_bandWidthNr / uePhy->GetRbNum();

      // Store the node's transmission parameters in the tracker
      m_txMapNr[vehID] = txParametersNR {
          nodeID,
          netDevice,
          rbBand
      };
    }
}

// Insert LTE nodes into the tracker
/* void
TxTracker::InsertLteNodes (std::vector<std::tuple<std::string, uint8_t, Ptr<cv2x_LteUeNetDevice>>> nodes, double rbOh, uint32_t numerology)
{
  for (auto n : nodes)
    {
      // Extract vehicle ID, node ID, and network device from the tuple
      std::string vehID = std::get<0>(n);
      uint8_t nodeID = std::get<1>(n);
      Ptr<cv2x_LteUeNetDevice> netDevice = std::get<2>(n);

      // Ensure the network device is valid
      NS_ASSERT_MSG (netDevice != nullptr, "Node is nullptr");

      // Get the physical layer (PHY) of the NR device and calculate RB (Resource Block) bandwidth
      uint8_t numRb = calculateNumRb(m_bandWidthLte, rbOh, numerology);
      double rbBand = m_bandWidthLte / numRb;

      // Store the node's transmission parameters in the tracker
      m_txMapLte[vehID] = txParametersLTE {
          nodeID,
          netDevice,
          rbBand,
      };
    }
} */

// Add interference from C-V2X (NR-V2X) signals to DSRC signals
void
TxTracker::AddInterferenceFromCV2X (Ptr<NetDevice> netDevice, Ptr<SpectrumValue> signal, Ptr<PropagationLossModel> propagationLoss, Time duration) {
  // We need to determine the technology type of the transmitting node
  bool found = false;
  std::string technologyType;
  for (auto it = m_txMapNr.begin(); it != m_txMapNr.end(); ++it) {
      if (it->second.netDevice == DynamicCast<NrUeNetDevice>(netDevice)) {
          found = true;
          technologyType = "Nr";
          break;
        }
    }
  /* if (!found)
    {
      for (auto it = m_txMapLte.begin(); it != m_txMapLte.end(); ++it)
        {
          if (it->second.netDevice == DynamicCast<cv2x_LteUeNetDevice>(netDevice))
            {
              found = true;
              technologyType = "Lte";
              break;
            }
        }
    } */

  // NS_ASSERT_MSG (found, "NetDevice not found in TxTracker.");

  if (!m_txMap11p.empty()) {
      double wifiCentralFreq = m_centralFrequency11p;
      double wifiBandwidth = m_bandWidth11p;
      // double cCentralFrequency = technologyType == "Nr" ? m_centralFrequencyNr : m_centralFrequencyLte;
      double cCentralFrequency = m_centralFrequencyNr;
      // double cBandwidth = technologyType == "Nr" ? m_bandWidthNr : m_bandWidthLte;
      double cBandwidth = m_bandWidthNr;

      double cLowerFreq = cCentralFrequency - cBandwidth / 2;
      double cUpperFreq = cCentralFrequency + cBandwidth / 2;
      double wifiLowerFreq = wifiCentralFreq - wifiBandwidth / 2;
      double wifiUpperFreq = wifiCentralFreq + wifiBandwidth / 2;
      bool overlap = std::max(cLowerFreq, wifiLowerFreq) <= std::min(cUpperFreq, wifiUpperFreq);

      if (overlap) {
          // double freqPerRb = technologyType == "Nr" ? m_bandWidthNr / signal->GetValuesN() : m_bandWidthLte / signal->GetValuesN();
          double freqPerRb = m_bandWidthNr / signal->GetValuesN();
          Ptr<MobilityModel> cMobility = netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();

          for (auto it = m_txMap11p.begin(); it != m_txMap11p.end(); ++it) {
              Ptr<YansWifiPhy> wifiPhy = DynamicCast<YansWifiPhy>(it->second.netDevice->GetPhy());
              Ptr<MobilityModel> wifiMobility = it->second.netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
              uint8_t j = 0;
              std::vector<uint8_t> indexesRB;
              double powerW = 0.0;
              for (auto it2 = signal->ValuesBegin(); it2 != signal->ValuesEnd(); ++it2) {
                  if ((*it2) > 0) {
                      double subBandFreq = cLowerFreq + (j + 0.5) * freqPerRb;
                      if (subBandFreq >= wifiLowerFreq && subBandFreq <= wifiUpperFreq) {
                          powerW += (*it2) * freqPerRb;
                        }
                    }
		  j++;
                }

              // Calculate interference delay and received power
              if (powerW == 0.0) {
                  continue;
                }

              double powerDbm = WToDbm(powerW);
              double pathLoss = propagationLoss->CalcRxPower(0, cMobility, wifiMobility);
              double finalInterferencePowerDbm = powerDbm - std::abs(pathLoss);
              double finalInterferencePowerW = DbmToW(finalInterferencePowerDbm);

              if ((finalInterferencePowerDbm + wifiPhy->GetRxGain ()) < wifiPhy->GetRxSensitivity ()) {
                  continue;
                }
              else {
                  RxPowerWattPerChannelBand rxInterference = RxPowerWattPerChannelBand ();
                  rxInterference.insert ({std::make_pair (0, 0), finalInterferencePowerW});
                  wifiPhy->GetInterferenceHelper()->AddForeignSignal(duration, rxInterference);
                }
            }
        }
    }

  // The interference comes from a NR signal
  /* if (technologyType == "Nr" && !m_txMapLte.empty())
    {
      // The transmission is from NR, so we need to add interference from LTE signals
      double nrLowerFreq = m_centralFrequencyNr - m_bandWidthNr / 2;
      double nrUpperFreq = m_centralFrequencyNr + m_bandWidthNr / 2;
      double lteLowerFreq = m_centralFrequencyLte - m_bandWidthLte / 2;
      double lteUpperFreq = m_centralFrequencyLte + m_bandWidthLte / 2;
      bool overlap = std::max(nrLowerFreq, lteLowerFreq) <= std::min(nrUpperFreq, lteUpperFreq);

      if (overlap)
        {
          Ptr<MobilityModel> c2Mobility = netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
          double freqPerRb = m_bandWidthNr / signal->GetValuesN();
          for (auto it = m_txMapLte.begin(); it != m_txMapLte.end(); ++it)
            {
              Ptr<cv2x_LteSpectrumPhy> ltePhy = it->second.netDevice->GetPhy ()->GetSlSpectrumPhy ();

              // Create an interference signal compatible with the Lte Phy
              Ptr<SpectrumValue> interferenceSignal = Create<SpectrumValue> (ltePhy->GetRxSpectrumModel());

              Ptr<MobilityModel> c1Mobility = it->second.netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
              double pathLoss = propagationLoss->CalcRxPower (0, c1Mobility, c2Mobility);
              if (std::abs(pathLoss) > m_noisePowerThreshold)
                {
                  continue;
                }

              uint8_t j = 0;
              for (auto it2 = signal->ValuesBegin (); it2 != signal->ValuesEnd (); ++it2)
                {
                  double subBandFreq = nrLowerFreq + (j+1) * freqPerRb;
                  if ((*it2) > 0)
                    {
                      if (subBandFreq >= lteLowerFreq && subBandFreq <= lteUpperFreq)
                        {
                          (*interferenceSignal)[j] = (*it2);
                        }
                        j += 1;
                    }
                }

              double pathGainLinear = std::pow (10.0, pathLoss / 10.0);
              *(interferenceSignal) *= pathGainLinear;

              ltePhy->GetDataInterferencePointer()->AddSignal (interferenceSignal, duration);
              ltePhy->GetSlInterferencePointer()->AddSignal (interferenceSignal, duration);
              ltePhy->GetCtrlInterferencePointer()->AddSignal (interferenceSignal, duration);
            }
        }
        }

  // The interference comes from a LTE signal
  else if (technologyType == "Lte" && !m_txMapNr.empty())
    {
      // The transmission is from LTE, so we need to add interference from NR signals
      double lteLowerFreq = m_centralFrequencyLte - m_bandWidthLte / 2;
      double lteUpperFreq = m_centralFrequencyLte + m_bandWidthLte / 2;
      double nrLowerFreq = m_centralFrequencyNr - m_bandWidthNr / 2;
      double nrUpperFreq = m_centralFrequencyNr - m_bandWidthNr / 2;

      bool overlap = std::max(lteLowerFreq, nrLowerFreq) <= std::min(lteUpperFreq, nrUpperFreq);

      if (overlap)
        {
          Ptr<MobilityModel> c2Mobility = netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
          double freqPerRb = m_bandWidthLte / signal->GetValuesN();
          for (auto it = m_txMapNr.begin(); it != m_txMapNr.end(); ++it)
            {
              Ptr<NrSpectrumPhy> nrPhy = it->second.netDevice->GetPhy (0)->GetSpectrumPhy ();

              // Create an interference signal compatible with the Nr Phy
              Ptr<SpectrumValue> interferenceSignal = Create<SpectrumValue> (nrPhy->GetRxSpectrumModel());

              Ptr<MobilityModel> c1Mobility = it->second.netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
              double pathLoss = propagationLoss->CalcRxPower (-1, c1Mobility, c2Mobility);
              if (std::abs(pathLoss) > m_noisePowerThreshold)
                {
                  continue;
                }

              uint8_t j = 0;
              for (auto it2 = signal->ValuesBegin (); it2 != signal->ValuesEnd (); ++it2)
                {
                  double subBandFreq = lteLowerFreq + (j+1) * freqPerRb;
                  if ((*it2) > 0)
                    {
                      if (subBandFreq >= nrLowerFreq && subBandFreq <= nrUpperFreq)
                        {
                          (*interferenceSignal)[j] = (*it2);
                        }
                        j += 1;
                    }
                }

              double pathGainLinear = std::pow (10.0, pathLoss / 10.0);
              *(interferenceSignal) *= pathGainLinear;

              // nrPhy->GetNrInterference()->AddSignal (interferenceSignal, duration);
              nrPhy->GetDataInterferencePointer()->AddSignal (interferenceSignal, duration);
              nrPhy->GetSlInterferencePointer()->AddSignal (interferenceSignal, duration);
              nrPhy->GetCtrlInterferencePointer()->AddSignal (interferenceSignal, duration);
            }
        }
    } */
}

// Add interference from DSRC signals to C-V2X (NR-V2X) signals
void
TxTracker::AddInterferenceFrom11p (Ptr<YansWifiPhy> sender, Ptr<MobilityModel> receiverMobility, Ptr<PropagationLossModel> propagationLoss, Ptr<PropagationDelayModel> propagationDelay, Time duration) {
  if(!m_txMapNr.empty()) {
      double wifiLowerFreq = m_centralFrequency11p - m_bandWidth11p / 2;
      double wifiUpperFreq = m_centralFrequency11p + m_bandWidth11p / 2;
      double nrLowerFreq = m_centralFrequencyNr - m_bandWidthNr / 2;
      double nrUpperFreq = m_centralFrequencyNr + m_bandWidthNr / 2;
      bool overlap = std::max(wifiLowerFreq, nrLowerFreq) <= std::min(wifiUpperFreq, nrUpperFreq);

      if (overlap) {
          Ptr<MobilityModel> wifiMobility = sender->GetMobility();
          for (auto it = m_txMapNr.begin (); it != m_txMapNr.end (); ++it) {
              double interferencePower = 0.0;
              Ptr<NrSpectrumPhy> nrPhy = it->second.netDevice->GetPhy (0)->GetSpectrumPhy ();

              // Create an interference signal compatible with the Nr Phy
              Ptr<SpectrumValue> interferenceSignal = Create<SpectrumValue> (nrPhy->GetRxSpectrumModel());

              // Calculate interference for overlapping frequency bands
              Ptr<MobilityModel> c1Mobility = it->second.netDevice->GetNode()->GetObject<ConstantPositionMobilityModel>();
              double pathLoss = propagationLoss->CalcRxPower (0, wifiMobility, c1Mobility);

              if (std::abs(pathLoss) > m_noisePowerThreshold) {
                  continue;
                }

              Time interfDuration = duration /*+ propagationDelay->GetDelay (c1Mobility, wifiMobility)*/;

              uint8_t j = 0;
              double noisePowerDbm = sender->GetTxPowerStart() - std::abs(pathLoss);
              double noisePowerW = DbmToW (noisePowerDbm);
              double noisePowerPerHz = noisePowerW / m_bandWidth11p;
              double freqPerRb = m_bandWidthNr / interferenceSignal->GetValuesN();
              for (uint8_t i = 0; i < interferenceSignal->GetValuesN(); ++i) {
                  double subBandFreq = nrLowerFreq + (i + 0.5) * freqPerRb;
                  if (subBandFreq >= wifiLowerFreq && subBandFreq <= wifiUpperFreq) {
                      (*interferenceSignal)[j] = noisePowerPerHz;
                    }
                  j += 1;
                }

              // nrPhy->GetNrInterference()->AddSignal (interferenceSignal, interfDuration);
              nrPhy->GetDataInterferencePointer()->AddSignal (interferenceSignal, interfDuration);
              nrPhy->GetSlInterferencePointer()->AddSignal (interferenceSignal, interfDuration);
              nrPhy->GetCtrlInterferencePointer()->AddSignal (interferenceSignal, interfDuration);

            }
        }
    }

  /* if (!m_txMapLte.empty())
    {
      double wifiLowerFreq = m_centralFrequency11p - m_bandWidth11p / 2;
      double wifiUpperFreq = m_centralFrequency11p + m_bandWidth11p / 2;
      double lteLowerFreq = m_centralFrequencyLte - m_bandWidthLte / 2;
      double lteUpperFreq = m_centralFrequencyLte + m_bandWidthLte / 2;

      bool overlap =
          std::max (wifiLowerFreq, lteLowerFreq) <= std::min (wifiUpperFreq, lteUpperFreq);

      if (overlap)
        {
          Ptr<MobilityModel> wifiMobility = sender->GetMobility ();
          for (auto it = m_txMapLte.begin (); it != m_txMapLte.end (); ++it)
            {
              Ptr<cv2x_LteSpectrumPhy> ltePhy = it->second.netDevice->GetPhy ()->GetSlSpectrumPhy ();

              // Create an interference signal compatible with the Nr Phy
              Ptr<SpectrumValue> interferenceSignal =
                  Create<SpectrumValue> (ltePhy->GetRxSpectrumModel());

              // Calculate interference for overlapping frequency bands
              Ptr<MobilityModel> c1Mobility = it->second.netDevice->GetNode ()->GetObject<ConstantPositionMobilityModel> ();
              double pathLoss = propagationLoss->CalcRxPower (0, c1Mobility, wifiMobility);
              if (std::abs (pathLoss) > m_noisePowerThreshold)
                {
                  continue;
                }

              Time interfDuration = duration + propagationDelay->GetDelay (c1Mobility, wifiMobility);

              uint8_t j = 0;
              double noisePowerDbm = WToDbm (sender->GetTxPowerStart()) - std::abs(pathLoss);
              double noisePowerW = DbmToW (noisePowerDbm);
              double noisePowerPerHz = noisePowerW / m_bandWidthLte;
              double freqPerRb = m_bandWidthLte / interferenceSignal->GetValuesN();
              for (uint8_t i = 0; i < interferenceSignal->GetValuesN(); ++i)
                {
                  double subBandFreq = lteLowerFreq + (i + 1) * freqPerRb;
                  if (subBandFreq >= wifiLowerFreq && subBandFreq <= wifiUpperFreq)
                    {
                      (*interferenceSignal)[j] = noisePowerPerHz * freqPerRb;
                    }
                    j += 1;
                }

              ltePhy->GetDataInterferencePointer()->AddSignal (interferenceSignal, interfDuration);
              ltePhy->GetSlInterferencePointer()->AddSignal (interferenceSignal, interfDuration);
              ltePhy->GetCtrlInterferencePointer()->AddSignal (interferenceSignal, interfDuration);
            }
        }
    } */
}

} // namespace ns3
