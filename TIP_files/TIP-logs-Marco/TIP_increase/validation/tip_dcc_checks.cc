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
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  std::cout<<"scenario,check,pass\n";
  {
    scenario="immediate_quantization";Fixture f;
    check(f.generate(.409)==VAM_NO_ERROR,"submission_succeeds");
    check(close(f.reported(),.4),"reference_matches_encoded_value");
    check(close(std::get<1>(f.service->m_tip_map.at(12)),.409),"estimate_preserves_full_precision");
    check(f.captured.size()==1&&f.captured[0]==std::vector<long>{20},"wire_probability_is_20");
    check(f.service->m_vam_sent==1&&f.service->m_tip_sent==1,"counted_once");finish();
  }
  {
    scenario="fresh_selection_on_motion_trigger";Fixture f;
    f.service->addNewTIPToMap(12,0,.7);f.service->searchFirstEightTIPs();
    f.service->m_trigg_cond=POSITION_CHANGE;f.generate(.86);
    check(f.captured.size()==1&&f.captured[0]==std::vector<long>{43},"payload_uses_current_selection");
    check(close(f.reported(),.86),"reference_uses_fresh_payload");
    check(f.service->m_pos_sent==1&&f.service->m_tip_sent==0,"correct_trigger_counter");finish();
  }
  {
    scenario="encoding_failure";Fixture f;f.provider.bad=true;
    f.service->addNewTIPToMap(12,0,.409);f.service->updatePreviousSentTIPOnMap(12,.1);
    check(f.service->generateAndEncodeVam()==VAM_ASN1_UPER_ENC_ERROR,"encoding_error_returned");
    check(close(f.reported(),.1)&&f.service->m_vam_sent==0,"history_and_count_unchanged");
    check(f.captured.empty(),"no_packet_submitted");finish();
  }
  {
    scenario="socket_failure";Fixture f;f.socket->Close();
    check(f.generate(.409)==VAM_CANNOT_SEND,"socket_error_returned");
    check(close(f.reported(),0)&&f.service->m_vam_sent==0,"unconfirmed_packet_not_reported");finish();
  }
  for(const std::string mode:{"reactive","adaptive"}) {
    {
      scenario=mode+"_queued_snapshots";Fixture f(mode);
      check(f.generate(.409)==VAM_NO_ERROR,"queued_generation_succeeds");
      check(close(f.reported(),0)&&f.service->m_vam_sent==0,"queueing_does_not_commit");
      Simulator::Schedule(MilliSeconds(10),[&]{f.generate(.809);check(close(f.reported(),0),"second_queueing_does_not_commit");});
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();check(close(f.reported(),.4),"first_packet_commits_own_value");check(close(std::get<1>(f.service->m_tip_map.at(12)),.809),"newer_estimate_is_preserved");});
      Simulator::Schedule(MilliSeconds(40),[&]{f.service->addNewTIPToMap(12,40,.99);f.release();check(close(f.reported(),.8),"second_packet_commits_own_value");check(close(std::get<1>(f.service->m_tip_map.at(12)),.99),"live_estimate_not_replaced_by_snapshot");});
      f.run(41);check(f.service->m_vam_sent==2&&f.service->m_tip_sent==2,"queued_packets_counted_once");
      check(f.captured.size()==2&&f.captured[0]==std::vector<long>{20}&&f.captured[1]==std::vector<long>{40},"queued_wire_values_match_history");finish();
    }
    {
      scenario=mode+"_queue_disabled";Fixture f(mode);f.dcc->m_queue_length=0;
      f.generate(.409);check(f.dcc->m_dropped_full_queue==1,"disabled_queue_drops_packet");
      check(close(f.reported(),0)&&f.service->m_vam_sent==0,"dropped_packet_not_reported");finish();
    }
    {
      scenario=mode+"_queue_overflow";Fixture f(mode);f.dcc->m_queue_length=1;
      f.generate(.409);f.generate(.809);
      check(f.dcc->m_dcc_queue_dp0.size()==1,"full_queue_retains_only_one_packet");
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(close(f.reported(),.4)&&f.service->m_vam_sent==1,"only_retained_packet_commits");finish();
    }
    {
      scenario=mode+"_queue_expiry";Fixture f(mode);f.dcc->m_lifetime=10;
      f.generate(.409);Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(f.dcc->m_dropped_lifetime==1,"expired_packet_removed");
      check(close(f.reported(),0)&&f.service->m_vam_sent==0,"expiry_does_not_commit");finish();
    }
    {
      scenario=mode+"_terminated_session";Fixture f(mode);f.generate(.409);
      f.service->terminateDissemination();Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(close(f.reported(),0)&&f.service->m_vam_sent==0,"terminated_callback_ignored");finish();
    }
    {
      scenario=mode+"_destroyed_service";Fixture f(mode);f.generate(.409);
      auto lifetime=std::weak_ptr<int>(f.service->m_tip_tx_lifetime);f.service.reset();
      check(lifetime.expired(),"service_lifetime_expires");
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(f.dcc->m_dcc_queue_dp0.empty(),"late_packet_callback_is_safe");finish();
    }
    {
      scenario=mode+"_missing_prediction_at_confirmation";Fixture f(mode);f.generate(.409);
      f.service->m_tip_map.erase(12);Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(f.service->m_tip_map.count(12)==1&&close(f.reported(),.4),"history_recreated_from_transmitted_packet");
      check(std::get<1>(f.service->m_tip_map.at(12))==-1,"missing_current_prediction_remains_unavailable");finish();
    }
    {
      scenario=mode+"_initial_queue_and_generation_pacing";Fixture f(mode);f.dcc->m_Toff_ms=500;
      f.service->addNewTIPToMap(12,0,.409);f.service->startVamDissemination();f.run(250);
      check(f.service->m_prev_heading!=-1&&f.service->lastVamGen>=0,"queued_initial_generation_initializes_pacing");
      check(f.service->m_vam_sent==0&&close(f.reported(),0),"pending_generation_is_not_transmission");finish();
    }
    {
      scenario=mode+"_substitution_and_gate_spacing";Fixture f(mode);std::vector<std::pair<char,double>> events;
      auto a=request(true);a.onTxSuccess=[&]{events.emplace_back('A',Simulator::Now().GetMilliSeconds());};
      f.transport->sendBTP(a,0,MessageId_cam);
      Simulator::Schedule(MilliSeconds(100),[&]{
        auto b=request(false);b.onTxSuccess=[&]{events.emplace_back('B',Simulator::Now().GetMilliSeconds());};
        auto status=f.transport->sendBTP(b,0,MessageId_vam);
        check(std::get<1>(status)==MessageId_cam,"return_identifies_dequeued_packet");
        check(events.size()==1&&events[0].first=='A',"enqueue_does_not_reenter_same_gate");
      });
      f.run(250);
      check(events.size()==2&&events[1].first=='B',"both_packets_confirmed_once");
      check(events.size()==2&&events[1].second-events[0].second>=24.999,"DCC_spacing_is_respected");finish();
    }
    {
      scenario=mode+"_queued_GBC_confirmation";Fixture f(mode);int sent=0;
      auto packet=request(false,GBC);packet.onTxSuccess=[&]{++sent;};
      f.transport->sendBTP(packet,0,MessageId_vam);
      check(sent==0,"GBC_queueing_does_not_confirm");
      Simulator::Schedule(MilliSeconds(20),[&]{f.release();});f.run(21);
      check(sent==1,"shared_GBC_sender_confirms_once");finish();
    }
  }
  std::cerr<<"CHECKS="<<checks<<" PASSED="<<checks-failures<<" FAILED="<<failures<<'\n';
  return failures?1:0;
}
