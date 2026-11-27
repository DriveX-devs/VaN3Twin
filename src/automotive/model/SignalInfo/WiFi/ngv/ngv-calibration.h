/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Surrogate 802.11bd NGV PER parameters for VaN3Twin's current 10 MHz,
 * NSS=1 network-simulation model.
 *
 * These constants are modeling assumptions, not link-level calibrated
 * waveform results. Future work should replace them with calibrated
 * PER-vs-SNR and midamble/Doppler parameters.
 */

#ifndef NGV_CALIBRATION_H
#define NGV_CALIBRATION_H

#include <array>
#include <cstdint>

namespace ns3 {
namespace NgvCalibration {

struct PerReference
{
  uint8_t mcs;
  uint16_t channelWidthMHz;
  uint8_t nss;
  double snr10PercentPerDb;
  uint32_t referenceBytes;
};

struct MidambleDopplerGain
{
  uint16_t periodicity;
  double maxGainDb;
};

constexpr double PER_TRANSITION_SLOPE_DB = 1.5;
constexpr uint16_t SERVICE_BITS = 16;
constexpr uint16_t TAIL_BITS = 6;
constexpr uint32_t SHORT_PPDU_SYMBOLS = 16;
constexpr double LONG_PPDU_GAIN_NORMALIZATION_SYMBOLS = 128;

constexpr std::array<PerReference, 40> PER_REFERENCES = {{
    {0, 10, 1, 12, 4096},
    {1, 10, 1, 15, 4096},
    {2, 10, 1, 17, 4096},
    {3, 10, 1, 20, 4096},
    {4, 10, 1, 24, 4096},
    {5, 10, 1, 28, 4096},
    {6, 10, 1, 29, 4096},
    {7, 10, 1, 30, 4096},
    {8, 10, 1, 35, 4096},
    {15, 10, 1, 9, 2048},
    {0, 10, 2, 12, 4096},
    {1, 10, 2, 15, 4096},
    {2, 10, 2, 17, 4096},
    {3, 10, 2, 20, 4096},
    {4, 10, 2, 24, 4096},
    {5, 10, 2, 28, 4096},
    {6, 10, 2, 29, 4096},
    {7, 10, 2, 30, 4096},
    {8, 10, 2, 35, 4096},
    {0, 20, 1, 12, 4096},
    {1, 20, 1, 15, 4096},
    {2, 20, 1, 17, 4096},
    {3, 20, 1, 20, 4096},
    {4, 20, 1, 24, 4096},
    {5, 20, 1, 28, 4096},
    {6, 20, 1, 29, 4096},
    {7, 20, 1, 30, 4096},
    {8, 20, 1, 35, 4096},
    {9, 20, 1, 37, 4096},
    {15, 20, 1, 12, 2048},
    {0, 20, 2, 12, 4096},
    {1, 20, 2, 15, 4096},
    {2, 20, 2, 17, 4096},
    {3, 20, 2, 20, 4096},
    {4, 20, 2, 24, 4096},
    {5, 20, 2, 28, 4096},
    {6, 20, 2, 29, 4096},
    {7, 20, 2, 30, 4096},
    {8, 20, 2, 35, 4096},
    {9, 20, 2, 37, 4096}
}};

constexpr std::array<MidambleDopplerGain, 3> MIDAMBLE_DOPPLER_GAINS = {{
    {4, 2},
    {8, 1.4},
    {16, 0.8}
}};

} // namespace NgvCalibration
} // namespace ns3

#endif /* NGV_CALIBRATION_H */
