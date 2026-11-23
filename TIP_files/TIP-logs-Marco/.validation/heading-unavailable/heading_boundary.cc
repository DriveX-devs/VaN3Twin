#include "ns3/VRUBasicService.h"
#include "ns3/VRUdp.h"
#include "ns3/Encoding.hpp"
#include "ns3/Setter.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace ns3;

class FixedPedestrian : public VRUdp {
public:
  VRUdp_position_latlon_t getPedPosition() override { return {0, 0, 0}; }
  double getPedSpeedValue() override { return 0; }
  double getPedHeadingValue() override { return 0; }
  VRUDP_position_cartesian_t getXY(double lon, double lat) override { return {lon, lat, 0}; }
};

struct Geometry { const char* name; double x, y, speed; bool eligible; };

Ptr<Packet> encodeMessage(bool vehicle, const Geometry& g, long heading = 0) {
  std::string encoded;
  if (vehicle) {
    auto msg = asn1cpp::makeSeq(CAM);
    msg->header.protocolVersion = 2;
    msg->header.messageId = 2;
    msg->header.stationId = 12;
    auto& p = msg->cam.camParameters;
    p.basicContainer.stationType = StationType_passengerCar;
    p.basicContainer.referencePosition.longitude = std::lround(g.x * 10000000);
    p.basicContainer.referencePosition.latitude = std::lround(g.y * 10000000);
    p.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& hf = p.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    hf.heading.headingValue = heading;
    hf.heading.headingConfidence = 1;
    hf.speed.speedValue = std::lround(g.speed * 100);
    hf.speed.speedConfidence = 1;
    hf.vehicleLength.vehicleLengthValue = 1;
    hf.vehicleWidth = 1;
    hf.longitudinalAcceleration.confidence = 1;
    encoded = asn1cpp::uper::encode(msg);
  } else {
    auto msg = asn1cpp::makeSeq(VAM);
    msg->header.protocolVersion = 3;
    msg->header.messageId = 16;
    msg->header.stationId = 1006;
    auto& p = msg->vam.vamParameters;
    p.basicContainer.stationType = StationType_pedestrian;
    p.basicContainer.referencePosition.longitude = std::lround(g.x * 10000000);
    p.basicContainer.referencePosition.latitude = std::lround(g.y * 10000000);
    auto& hf = p.vruHighFrequencyContainer;
    hf.heading.value = heading;
    hf.heading.confidence = 1;
    hf.speed.speedValue = std::lround(g.speed * 100);
    hf.speed.speedConfidence = 1;
    hf.longitudinalAcceleration.longitudinalAccelerationConfidence = 1;
    encoded = asn1cpp::uper::encode(msg);
  }
  if (encoded.empty()) throw std::runtime_error("ASN.1 encoding failed");
  return Create<Packet>(reinterpret_cast<const uint8_t*>(encoded.data()), encoded.size());
}

#include <limits>
class TestPedestrian : public FixedPedestrian {
public:
  double heading = 0, speed = 0;
  double getPedHeadingValue() override { return heading; }
  double getPedSpeedValue() override { return speed; }
};
void configure(VRUBasicService& service, TestPedestrian& provider, const std::string& modality = "exp") {
  service.setVRUdp(&provider);
  service.setUseTIP();
  service.setTTCMax(4); service.setTTCMin(1); service.setSTCMin(5);
  service.setK(.27); service.setSigma(1); service.setModality(modality);
}
bool close(double a, double b) { return std::isfinite(a) && std::abs(a-b)<1e-6; }
struct HeadingCase { const char* name; double remote, ego, egoSpeed, x, y; bool valid; };
int main(int argc, char** argv) {
  if (argc!=2) return 2;
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  const std::vector<HeadingCase> cases={
    {"valid_north",0,0,0,0,-2,true},
    {"valid_east",90,0,0,-2,0,true},
    {"remote_scaled_unavailable",360.1,0,0,0,-2,true},
    {"remote_raw_unavailable",3601,0,0,0,-2,true},
    {"ego_scaled_unavailable",0,360.1,.5,0,-1,true},
    {"ego_raw_unavailable",0,3601,.5,0,-1,true},
    {"both_scaled_unavailable",360.1,360.1,0,0,-2,false},
    {"both_raw_unavailable",3601,3601,0,0,-2,false},
    {"mixed_unavailable",360.1,3601,0,0,-2,false},
    {"remote_nan",nan,0,0,0,-2,true},
    {"ego_nan",0,nan,.5,0,-1,true},
    {"remote_infinite",inf,0,0,0,-2,true},
    {"both_nan",nan,nan,0,0,-2,false},
    {"mixed_infinite_unavailable",inf,360.1,0,0,-2,false},
  };
  int failures=0;
  std::cout<<"path,modality,case,ttc_s,stc_m,tip,pass\n";
  for (const auto& c:cases) {
    TestPedestrian provider; provider.heading=c.ego; provider.speed=c.egoSpeed;
    VRUBasicService service; configure(service,provider);
    vehicleData_t remote={}; remote.heading=c.remote; remote.speed_ms=1; remote.lon=c.x; remote.lat=c.y;
    const auto [ttc,stc]=service.compute_ttc_stc(remote);
    bool pass=c.valid ? close(ttc,2)&&close(stc,0) : ttc==-1&&stc==-1;
    failures+=!pass;
    std::cout<<"compute,,"<<c.name<<','<<ttc<<','<<stc<<",,"<<(pass?"PASS":"FAIL")<<'\n';
  }
  for (bool vehicle:{true,false}) for (const std::string modality:{"exp","gaus"}) {
    for (const std::string name:{"valid_north","remote_unavailable","both_unavailable","ego_unavailable"}) {
      TestPedestrian provider;
      if (name=="both_unavailable" || name=="ego_unavailable") provider.heading=360.1;
      if (name=="ego_unavailable") provider.speed=.5;
      VRUBasicService service; configure(service,provider,modality);
      const std::string path=std::string(argv[1])+"/"+(vehicle?"CAM_":"VAM_")+modality+"_"+name+".csv";
      std::ofstream(path,std::ios::trunc).close(); service.setTIPLogFile(path);
      Geometry g={"receiver",0,name=="ego_unavailable"?-1.0:-2.0,1,true};
      BTPDataIndication_t indication={};
      long rawHeading=(name=="remote_unavailable" || name=="both_unavailable")?3601:0;
      indication.data=encodeMessage(vehicle,g,rawHeading);
      service.VRUreceivedNewMessageForTIP(indication,vehicle);
      std::ifstream log(path); std::string line,field; std::getline(log,line);
      std::istringstream parser(line); std::vector<std::string> fields;
      while(std::getline(parser,field,',')) fields.push_back(field);
      if (fields.size()!=7) throw std::runtime_error("Unexpected TIP log: "+line);
      const double ttc=std::stod(fields[4]),stc=std::stod(fields[5]),tip=std::stod(fields[6]);
      const double expected=modality=="exp"?std::exp(-.27):std::exp(-2.0);
      bool pass=name=="both_unavailable" ? ttc==-1&&stc==-1&&tip==-1 : close(ttc,2)&&close(stc,0)&&close(tip,expected);
      failures+=!pass;
      std::cout<<(vehicle?"CAM":"VAM")<<','<<modality<<','<<name<<','<<ttc<<','<<stc<<','<<tip<<','<<(pass?"PASS":"FAIL")<<'\n';
    }
  }
  Simulator::Destroy();
  return failures?1:0;
}
