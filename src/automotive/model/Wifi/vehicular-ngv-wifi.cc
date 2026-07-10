#include "vehicular-ngv-wifi.h"

#include "ns3/fatal-error.h"

#include <array>

namespace ns3
{

namespace
{

constexpr std::array<VehicularNgvMcs, 10> g_ngvMcs10MHzNss1 = {{
    {0, "BPSK", 1, 2, 1, 52, 52, 26, 3.3, false},
    {1, "QPSK", 1, 2, 2, 52, 104, 52, 6.5, false},
    {2, "QPSK", 3, 4, 2, 52, 104, 78, 9.8, false},
    {3, "16-QAM", 1, 2, 4, 52, 208, 104, 13.0, false},
    {4, "16-QAM", 3, 4, 4, 52, 208, 156, 19.5, false},
    {5, "64-QAM", 2, 3, 6, 52, 312, 208, 26.0, false},
    {6, "64-QAM", 3, 4, 6, 52, 312, 234, 29.3, false},
    {7, "64-QAM", 5, 6, 6, 52, 312, 260, 32.5, false},
    {8, "256-QAM", 3, 4, 8, 52, 416, 312, 39.0, false},
    {15, "BPSK-DCM", 1, 2, 1, 26, 26, 13, 1.6, true},
}};

constexpr std::array<VehicularNgvMcs, 9> g_ngvMcs10MHzNss2 = {{
    {0, "BPSK", 1, 2, 1, 52, 104, 52, 6.5, false},
    {1, "QPSK", 1, 2, 2, 52, 208, 104, 13.0, false},
    {2, "QPSK", 3, 4, 2, 52, 208, 156, 19.5, false},
    {3, "16-QAM", 1, 2, 4, 52, 416, 208, 26.0, false},
    {4, "16-QAM", 3, 4, 4, 52, 416, 312, 39.0, false},
    {5, "64-QAM", 2, 3, 6, 52, 624, 416, 52.0, false},
    {6, "64-QAM", 3, 4, 6, 52, 624, 468, 58.5, false},
    {7, "64-QAM", 5, 6, 6, 52, 624, 520, 65.0, false},
    {8, "256-QAM", 3, 4, 8, 52, 832, 624, 78.0, false},
}};

constexpr std::array<VehicularNgvMcs, 11> g_ngvMcs20MHzNss1 = {{
    {0, "BPSK", 1, 2, 1, 108, 108, 54, 6.8, false},
    {1, "QPSK", 1, 2, 2, 108, 216, 108, 13.5, false},
    {2, "QPSK", 3, 4, 2, 108, 216, 162, 20.3, false},
    {3, "16-QAM", 1, 2, 4, 108, 432, 216, 27.0, false},
    {4, "16-QAM", 3, 4, 4, 108, 432, 324, 40.5, false},
    {5, "64-QAM", 2, 3, 6, 108, 648, 432, 54.0, false},
    {6, "64-QAM", 3, 4, 6, 108, 648, 486, 60.8, false},
    {7, "64-QAM", 5, 6, 6, 108, 648, 540, 67.5, false},
    {8, "256-QAM", 3, 4, 8, 108, 864, 648, 81.0, false},
    {9, "256-QAM", 5, 6, 8, 108, 864, 720, 90.0, false},
    {15, "BPSK-DCM", 1, 2, 1, 54, 54, 27, 3.4, true},
}};

constexpr std::array<VehicularNgvMcs, 10> g_ngvMcs20MHzNss2 = {{
    {0, "BPSK", 1, 2, 1, 108, 216, 108, 13.5, false},
    {1, "QPSK", 1, 2, 2, 108, 432, 216, 27.0, false},
    {2, "QPSK", 3, 4, 2, 108, 432, 324, 40.5, false},
    {3, "16-QAM", 1, 2, 4, 108, 864, 432, 54.0, false},
    {4, "16-QAM", 3, 4, 4, 108, 864, 648, 81.0, false},
    {5, "64-QAM", 2, 3, 6, 108, 1296, 864, 106.0, false},
    {6, "64-QAM", 3, 4, 6, 108, 1296, 972, 121.5, false},
    {7, "64-QAM", 5, 6, 6, 108, 1296, 1080, 135.0, false},
    {8, "256-QAM", 3, 4, 8, 108, 1728, 1296, 162.0, false},
    {9, "256-QAM", 5, 6, 8, 108, 1728, 1440, 180.0, false},
}};

template <std::size_t N>
const VehicularNgvMcs*
FindMcs (const std::array<VehicularNgvMcs, N>& table, uint8_t index)
{
  for (const auto& mcs : table)
    {
      if (mcs.index == index)
        {
          return &mcs;
        }
    }
  return nullptr;
}

const VehicularNgvMcs*
FindMcs (uint8_t index, uint16_t channelWidthMHz, uint8_t spatialStreams)
{
  if (channelWidthMHz == 10 && spatialStreams == 1)
    {
      return FindMcs (g_ngvMcs10MHzNss1, index);
    }
  if (channelWidthMHz == 10 && spatialStreams == 2)
    {
      return FindMcs (g_ngvMcs10MHzNss2, index);
    }
  if (channelWidthMHz == 20 && spatialStreams == 1)
    {
      return FindMcs (g_ngvMcs20MHzNss1, index);
    }
  if (channelWidthMHz == 20 && spatialStreams == 2)
    {
      return FindMcs (g_ngvMcs20MHzNss2, index);
    }
  return nullptr;
}

} // namespace

std::string
VehicularWifiPpduFormatName (VehicularWifiPpduFormat format)
{
  switch (format)
    {
    case VehicularWifiPpduFormat::NON_NGV_10:
      return "NON_NGV_10";
    case VehicularWifiPpduFormat::NON_NGV_20_DUPLICATE:
      return "NON_NGV_20_DUPLICATE";
    case VehicularWifiPpduFormat::NGV:
      return "NGV";
    }
  NS_FATAL_ERROR ("Unknown vehicular Wi-Fi PPDU format");
  return "";
}

bool
IsVehicularNgvMcsValid (uint8_t index, uint16_t channelWidthMHz, uint8_t spatialStreams)
{
  return FindMcs (index, channelWidthMHz, spatialStreams) != nullptr;
}

const VehicularNgvMcs&
GetVehicularNgvMcs (uint8_t index, uint16_t channelWidthMHz, uint8_t spatialStreams)
{
  if (const VehicularNgvMcs* mcs = FindMcs (index, channelWidthMHz, spatialStreams);
      mcs != nullptr)
    {
      return *mcs;
    }

  NS_FATAL_ERROR ("Invalid or reserved NGV-MCS index for " << channelWidthMHz
                                                           << " MHz, NSS=" << +spatialStreams);
  return g_ngvMcs10MHzNss1.front ();
}

} // namespace ns3
