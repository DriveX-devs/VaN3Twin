/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "error-rate-model.h"
#include "ns3/dsss-error-rate-model.h"
#include "ngv/ngv-calibration.h"
#include "ngv/ngv-phy.h"
#include "wifi-utils.h"
#include "wifi-tx-vector.h"

#include <algorithm>
#include <cmath>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (ErrorRateModel);

namespace {

const NgvCalibration::PerReference&
GetNgvPerReference (const WifiTxVector& txVector)
{
  for (const auto& reference : NgvCalibration::PER_REFERENCES)
    {
      if (reference.mcs == txVector.GetNgvMcs () &&
          reference.channelWidthMHz == txVector.GetChannelWidth () &&
          reference.nss == txVector.GetNss ())
        {
          return reference;
        }
    }
  NS_ABORT_MSG ("Invalid or reserved NGV-MCS index for " << txVector.GetChannelWidth ()
                                                         << " MHz, NSS=" << +txVector.GetNss ());
}

double
GetNgvMidambleDopplerGainDb (const WifiTxVector& txVector, uint64_t nbits)
{
  const uint16_t periodicity = txVector.GetNgvMidamblePeriodicity ();
  double maxGainDb = 0.0;
  for (const auto& gain : NgvCalibration::MIDAMBLE_DOPPLER_GAINS)
    {
      if (gain.periodicity == periodicity)
        {
          maxGainDb = gain.maxGainDb;
          break;
        }
    }
  if (maxGainDb == 0.0)
    {
      return 0.0;
    }

  const uint16_t dataBitsPerSymbol = NgvPhy::GetDataBitsPerSymbol (txVector);
  const double codedBits =
      static_cast<double> (NgvCalibration::SERVICE_BITS) + static_cast<double> (nbits) +
      NgvCalibration::TAIL_BITS;
  const uint32_t numSymbols = static_cast<uint32_t> (std::ceil (codedBits / dataBitsPerSymbol));
  if (numSymbols <= NgvCalibration::SHORT_PPDU_SYMBOLS)
    {
      return 0.0;
    }

  const double longPpduFactor =
      std::min (1.0,
                (numSymbols - NgvCalibration::SHORT_PPDU_SYMBOLS) /
                    NgvCalibration::LONG_PPDU_GAIN_NORMALIZATION_SYMBOLS);
  return maxGainDb * longPpduFactor;
}

double
GetNonNgv10RepetitionCombiningGainDb (const WifiTxVector& txVector, WifiPpduField field)
{
  if (!txVector.IsNonNgv10 () || txVector.GetNgvPpduRepetitions () == 0 ||
      field != WIFI_PPDU_FIELD_DATA)
    {
      return 0.0;
    }

  const double repeatedObservations = 1.0 + txVector.GetNgvPpduRepetitions ();
  return 10.0 * std::log10 (repeatedObservations);
}

double
CalculateNgvChunkSuccessRate (const WifiTxVector& txVector, double snr, uint64_t nbits)
{
  NS_ABORT_MSG_IF (!txVector.IsValid (), "Cannot calculate PER for an invalid NGV TXVECTOR");

  const NgvCalibration::PerReference& reference = GetNgvPerReference (txVector);
  const double snrDb = RatioToDb (snr) + GetNgvMidambleDopplerGainDb (txVector, nbits);
  const double snr50PercentPerDb =
      reference.snr10PercentPerDb - NgvCalibration::PER_TRANSITION_SLOPE_DB * std::log (9.0);
  const double logisticArgument =
      (snrDb - snr50PercentPerDb) / NgvCalibration::PER_TRANSITION_SLOPE_DB;

  double referencePer;
  if (logisticArgument > 60.0)
    {
      referencePer = 0.0;
    }
  else if (logisticArgument < -60.0)
    {
      referencePer = 1.0;
    }
  else
    {
      referencePer = 1.0 / (1.0 + std::exp (logisticArgument));
    }

  const double referenceBits = static_cast<double> (reference.referenceBytes) * 8.0;
  const double sizeRatio = std::max (1.0, static_cast<double> (nbits)) / referenceBits;
  double per = 1.0 - std::pow (1.0 - referencePer, sizeRatio);
  per = std::min (1.0, std::max (0.0, per));
  return 1.0 - per;
}

} // namespace

TypeId ErrorRateModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ErrorRateModel")
    .SetParent<Object> ()
    .SetGroupName ("Wifi")
  ;
  return tid;
}

double
ErrorRateModel::CalculateSnr (const WifiTxVector& txVector, double ber) const
{
  //This is a very simple binary search.
  double low, high, precision;
  low = 1e-25;
  high = 1e25;
  precision = 2e-12;
  while (high - low > precision)
    {
      NS_ASSERT (high >= low);
      double middle = low + (high - low) / 2;
      if ((1 - GetChunkSuccessRate (txVector.GetMode (), txVector, middle, 1)) > ber)
        {
          low = middle;
        }
      else
        {
          high = middle;
        }
    }
  return low;
}

double
ErrorRateModel::GetChunkSuccessRate (WifiMode mode, const WifiTxVector& txVector, double snr, uint64_t nbits, uint8_t numRxAntennas, WifiPpduField field, uint16_t staId) const
{
  if (txVector.IsNgv () && field == WIFI_PPDU_FIELD_DATA)
    {
      return CalculateNgvChunkSuccessRate (txVector, snr, nbits);
    }
  else if (mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS || mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS)
    {
      switch (mode.GetDataRate (22, 0, 1))
        {
          case 1000000:
            return DsssErrorRateModel::GetDsssDbpskSuccessRate (snr, nbits);
          case 2000000:
            return DsssErrorRateModel::GetDsssDqpskSuccessRate (snr, nbits);
          case 5500000:
            return DsssErrorRateModel::GetDsssDqpskCck5_5SuccessRate (snr, nbits);
          case 11000000:
            return DsssErrorRateModel::GetDsssDqpskCck11SuccessRate (snr, nbits);
          default:
            NS_ASSERT ("undefined DSSS/HR-DSSS datarate");
        }
    }
  else
    {
      snr *= DbToRatio (GetNonNgv10RepetitionCombiningGainDb (txVector, field));
      return DoGetChunkSuccessRate (mode, txVector, snr, nbits, numRxAntennas, field, staId);
    }
  return 0;
}

bool
ErrorRateModel::IsAwgn (void) const
{
  return true;
}

int64_t
ErrorRateModel::AssignStreams (int64_t stream)
{
  // Override this method if the error model uses random variables
  return 0;
}

} //namespace ns3
