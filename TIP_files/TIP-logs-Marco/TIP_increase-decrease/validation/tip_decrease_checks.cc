#define main headingRegressionMain
#include "/home/marco/Desktop/GitHub_projects/VaN3Twin/ns-3-dev/TIP_files/TIP-logs-Marco/.validation/heading-unavailable/heading_boundary.cc"
#undef main
#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/DCC.h"
#include "ns3/geonet.h"
#include <algorithm>
#include <functional>
#include <memory>

int checks=0,failures=0;
std::string scenario;
void check(bool pass,const std::string& label) {
  ++checks;failures+=!pass;
  std::cout<<scenario<<','<<label<<','<<(pass?"PASS":"FAIL")<<'\n';
}
class Provider:public TestPedestrian {
public:
  bool bad=false;
  VAM_mandatory_data_t getVAMMandatoryData() override {
    VAM_mandatory_data_t data{};
    data.speed=VRUdpValueConfidence<>(0,1);
    data.longitude=0;data.latitude=bad?1000000000:0;
    data.altitude=VRUdpValueConfidence<>(0,0);
    data.posConfidenceEllipse={1,1,0};
    data.longAcceleration=VRUdpValueConfidence<>(0,1);
    data.heading=VRUdpValueConfidence<>(0,1);
    return data;
  }
  libsumo::TraCIPosition getPedPositionValue() override {libsumo::TraCIPosition p;p.x=0;p.y=0;p.z=0;return p;}
  std::vector<distance_t> get_min_distance(Ptr<LDM>,uint64_t,double,double,double) override {return {};}
};
struct Fixture {
  Provider provider;
  Ptr<Node> node;
  Ptr<GeoNet> gn;
  Ptr<btp> transport;
  Ptr<Socket> socket;
  Ptr<DCC> dcc;
  std::unique_ptr<VRUBasicService> service;
  std::vector<std::vector<long>> captured;
  Fixture(const std::string& mode="") {
    NodeContainer nodes;nodes.Create(1);node=nodes.Get(0);
    YansWifiChannelHelper channel=YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;phy.SetChannel(channel.Create());
    auto mac=NqosWaveMacHelper::Default();auto wifi=Wifi80211pHelper::Default();
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue("OfdmRate12MbpsBW10MHz"),"ControlMode",StringValue("OfdmRate12MbpsBW10MHz"),"NonUnicastMode",StringValue("OfdmRate12MbpsBW10MHz"));
    auto devices=wifi.Install(phy,mac,nodes);
    MobilityHelper mobility;mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");mobility.Install(nodes);
    PacketSocketHelper packets;packets.Install(nodes);
    socket=GeoNet::createGNPacketSocket(node);
    gn=CreateObject<GeoNet>();gn->m_socket_tx=socket;
    gn->m_stationtype=StationType_pedestrian;gn->m_station_id=1003;
    gn->m_egoPV.POS_EPV.lat=0;gn->m_egoPV.POS_EPV.lon=0;gn->m_egoPV.S_EPV=0;gn->m_egoPV.H_EPV=0;gn->m_egoPV.TST_EPV=0;
    gn->m_GNAreaForwardingAlgorithm=0;
    transport=CreateObject<btp>();transport->setGeoNet(gn);
    service=std::make_unique<VRUBasicService>();configure(*service,provider);
    service->setBTP(transport);service->m_station_id=1003;service->m_stationtype=StationType_pedestrian;service->m_trigg_cond=TIP_TRIGGER;
    auto device=DynamicCast<WifiNetDevice>(devices.Get(0));
    device->GetMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&Fixture::capture,this));
    if(!mode.empty()) {
      dcc=CreateObject<DCC>();dcc->m_node=node;dcc->m_modality=mode;
      dcc->m_queue_length=4;dcc->m_lifetime=5000;dcc->m_Toff_ms=100;dcc->m_last_tx=0;
      dcc->m_delta=.01;dcc->m_bitrate_bps=12000000;
      gn->setDCC(dcc);
    }
  }
  void capture(Ptr<const Packet> packet) {
    std::vector<uint8_t> bytes(packet->GetSize());packet->CopyData(bytes.data(),bytes.size());
    const std::vector<uint8_t> llc={0xaa,0xaa,3,0,0,0,0x89,0x47};
    auto pos=std::search(bytes.begin(),bytes.end(),llc.begin(),llc.end());
    if(pos==bytes.end())return;
    size_t gn_offset=pos-bytes.begin()+8;
    if(bytes.size()<gn_offset+44||bytes[gn_offset+5]!=0x50)return;
    size_t length=(bytes[gn_offset+8]<<8)|bytes[gn_offset+9];
    if(length<4||bytes.size()<gn_offset+40+length)return;
    const auto* payload=bytes.data()+gn_offset+44;
    if(payload[0]!=3||payload[1]!=16)return;
    std::string encoded(reinterpret_cast<const char*>(payload),length-4);
    auto message=asn1cpp::uper::decodeASN(encoded,VAM);
    if(!message)throw std::runtime_error("Cannot decode captured VAM");
    std::vector<long> probabilities;
    auto* motion=message->vam.vamParameters.vruMotionPredictionContainer;
    if(motion&&motion->trajectoryInterceptionIndication) {
      const auto& list=motion->trajectoryInterceptionIndication->list;
      for(int i=0;i<list.count;++i)probabilities.push_back(list.array[i]->trajectoryInterceptionProbability);
    }
    captured.push_back(probabilities);
  }
  VRUBasicService_error_t generate(double probability) {
    service->addNewTIPToMap(12,Simulator::Now().GetMilliSeconds(),probability);
    return service->generateAndEncodeVam();
  }
  double reported() {return std::get<2>(service->m_tip_map.at(12));}
  void release() {
    dcc->m_last_tx=Simulator::Now().GetMilliSeconds()-dcc->m_Toff_ms;
    dcc->checkQueue();
  }
  void run(double until_ms) {Simulator::Stop(MilliSeconds(until_ms));Simulator::Run();}
};
BTPDataRequest_t request(bool vehicle,TransportType_t type=TSB) {
  BTPDataRequest_t r{};r.BTPType=BTP_B;r.destPort=vehicle?CA_PORT:VA_PORT;r.GNType=type;
  r.GNCommProfile=UNSPECIFIED;r.GNMaxLife=1;r.GNMaxHL=1;r.GNTraClass=2;
  r.GnAddress={0,0,50,0,0,CIRCULAR};
  Geometry g={"test",0,-2,1,true};r.data=encodeMessage(vehicle,g);r.lenght=r.data->GetSize();return r;
}
void finish() {Simulator::Destroy();}

#include <limits>
struct SelectionCase {const char* name;double previous,current;bool increase_only,with_decrease;};
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  std::cout<<"scenario,check,pass\n";
  const std::vector<SelectionCase> cases={
    {"increase_above",.4,.51,true,true}, {"increase_at",0,.1,true,true},
    {"increase_below",.4,.49,false,false}, {"decrease_above",.8,.6,false,true},
    {"decrease_at_decimal_boundary",.7,.6,false,true},
    {"decrease_just_below",.7,.600000001,false,false},
    {"decrease_below",.7,.61,false,false}, {"unchanged",.7,.7,false,false},
    {"decrease_to_zero",.2,0,false,true}, {"unavailable",.7,-1,false,false},
    {"NaN",.7,std::numeric_limits<double>::quiet_NaN(),false,false},
    {"positive_infinity",.7,std::numeric_limits<double>::infinity(),false,false},
    {"negative_infinity",.7,-std::numeric_limits<double>::infinity(),false,false}};
  for(int mode:{-1,0,1}) {
    for(const auto& c:cases) {
      TestPedestrian provider;VRUBasicService s;configure(s,provider);
      if(mode>=0)s.setTIPDecrease(mode==1);
      scenario=std::string(mode==-1?"default_":mode==0?"false_":"true_")+c.name;
      s.addNewTIPToMap(12,0,c.current);s.updatePreviousSentTIPOnMap(12,c.previous);s.searchFirstEightTIPs();
      const bool expected=mode==1?c.with_decrease:c.increase_only;
      check((!s.m_tip_array.empty())==expected,"eligible_as_expected");
      check(close(std::get<2>(s.m_tip_map.at(12)),c.previous),"selection_does_not_commit_history");
    }
  }
  {
    scenario="constructor_defaults";TestPedestrian p;
    VRUBasicService a,b(1003,StationType_pedestrian,&p,false);
    check(!a.m_tip_decrease&&!b.m_tip_decrease,"both_constructors_default_false");
  }
  {
    scenario="eight_entry_limit";TestPedestrian p;VRUBasicService s;configure(s,p);s.setTIPDecrease(true);
    for(int i=1;i<=10;++i){s.addNewTIPToMap(i,0,i*.02);s.updatePreviousSentTIPOnMap(i,.9);}
    s.searchFirstEightTIPs();check(s.m_tip_array.size()==8,"limit_is_preserved");
    bool sorted=true;for(size_t i=1;i<s.m_tip_array.size();++i)sorted&=std::get<1>(s.m_tip_array[i-1])>=std::get<1>(s.m_tip_array[i]);
    check(sorted&&close(std::get<1>(s.m_tip_array.front()),.2)&&close(std::get<1>(s.m_tip_array.back()),.06),"descending_probability_order_is_preserved");
  }
  for(bool enabled:{false,true}) {
    scenario=enabled?"true_actual_decrease_trigger":"false_no_decrease_trigger";Fixture f;
    f.service->setTIPDecrease(enabled);f.generate(.7);
    const auto initial=f.service->m_vam_sent;const auto initial_tips=f.service->m_tip_sent;
    Simulator::Schedule(MilliSeconds(100),[&]{f.service->addNewTIPToMap(12,100,.6);f.service->checkVamConditions();});
    f.run(101);
    check(f.service->m_vam_sent==initial+(enabled?1:0),"actual_generation_matches_option");
    check(f.service->m_tip_sent==initial_tips+(enabled?1:0),"TIP_trigger_counter_matches_option");
    check(close(f.reported(),enabled?.6:.7),"reported_reference_matches_transmitted_payload");
    check(f.captured.size()==(enabled?2u:1u)&&(!enabled||f.captured.back()==std::vector<long>{30}),"wire_payload_matches_option");finish();
  }
  {
    scenario="fresh_motion_triggered_decrease";Fixture f;f.service->setTIPDecrease(true);
    f.service->addNewTIPToMap(12,0,.8);f.service->updatePreviousSentTIPOnMap(12,.8);f.service->searchFirstEightTIPs();
    f.service->m_trigg_cond=POSITION_CHANGE;f.generate(.609);
    check(f.captured.size()==1&&f.captured[0]==std::vector<long>{30},"motion_VAM_contains_fresh_decreased_TIP");
    check(close(f.reported(),.6),"reference_is_quantized_decreased_TIP");
    check(close(std::get<1>(f.service->m_tip_map.at(12)),.609),"current_estimate_preserves_precision");finish();
  }
  for(const std::string mode:{"reactive","adaptive"}) {
    {
      scenario=mode+"_queued_decrease_snapshots";Fixture f(mode);f.service->setTIPDecrease(true);
      f.service->addNewTIPToMap(12,0,.8);f.service->updatePreviousSentTIPOnMap(12,.8);f.generate(.609);
      check(close(f.reported(),.8)&&f.service->m_vam_sent==0,"queueing_does_not_commit_decrease");
      Simulator::Schedule(MilliSeconds(10),[&]{f.generate(.409);});
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();check(close(f.reported(),.6),"first_decrease_commits_own_quantized_value");check(close(std::get<1>(f.service->m_tip_map.at(12)),.409),"newer_current_prediction_is_preserved");});
      Simulator::Schedule(MilliSeconds(40),[&]{f.release();check(close(f.reported(),.4),"second_decrease_commits_own_quantized_value");});
      f.run(41);
      check(f.captured.size()==2&&f.captured[0]==std::vector<long>{30}&&f.captured[1]==std::vector<long>{20},"queued_wire_values_match_history");
      check(f.service->m_vam_sent==2&&f.service->m_tip_sent==2,"queued_transmissions_counted_once");finish();
    }
    for(bool expire:{false,true}) {
      scenario=mode+(expire?"_expired_decrease":"_dropped_decrease");Fixture f(mode);f.service->setTIPDecrease(true);
      if(expire)f.dcc->m_lifetime=10;else f.dcc->m_queue_length=0;
      f.service->addNewTIPToMap(12,0,.8);f.service->updatePreviousSentTIPOnMap(12,.8);f.generate(.609);
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(close(f.reported(),.8)&&f.service->m_vam_sent==0,"unsent_decrease_does_not_commit");finish();
    }
  }
  std::cerr<<"CHECKS="<<checks<<" PASSED="<<checks-failures<<" FAILED="<<failures<<'\n';
  return failures?1:0;
}
