#ifndef VEHICULAR_NGV_WIFI_H
#define VEHICULAR_NGV_WIFI_H

#include <cstdint>
#include <string>

namespace ns3
{

enum class VehicularWifiPpduFormat
{
  NON_NGV_10,
  NON_NGV_20_DUPLICATE,
  NGV
};

struct VehicularNgvMcs
{
  uint8_t index;
  const char* modulation;
  uint8_t codeRateNumerator;
  uint8_t codeRateDenominator;
  uint16_t bitsPerSubcarrier;
  uint16_t dataSubcarriers;
  uint16_t codedBitsPerSymbol;
  uint16_t dataBitsPerSymbol;
  double dataRateMbps;
  bool dcm;
};

std::string VehicularWifiPpduFormatName (VehicularWifiPpduFormat format);
bool IsVehicularNgvMcsValid (uint8_t index, uint16_t channelWidthMHz = 10, uint8_t spatialStreams = 1);
const VehicularNgvMcs& GetVehicularNgvMcs (uint8_t index,
                                           uint16_t channelWidthMHz = 10,
                                           uint8_t spatialStreams = 1);

} // namespace ns3

#endif // VEHICULAR_NGV_WIFI_H
