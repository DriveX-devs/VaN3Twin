#define main headingRegressionMain
#include "/home/marco/Desktop/GitHub_projects/VaN3Twin/ns-3-dev/TIP_files/TIP-logs-Marco/.validation/heading-unavailable/heading_boundary.cc"
#undef main

int failures=0,checks=0;
void check(bool condition,const std::string& name,const std::string& variant) {
  ++checks; failures+=!condition;
  std::cout<<variant<<','<<name<<','<<(condition?"PASS":"FAIL")<<'\n';
}
bool selected(VRUBasicService& service,uint64_t id) {
  for (const auto& entry:service.m_tip_array) if(std::get<0>(entry)==id) return true;
  return false;
}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  std::cout<<"variant,check,pass\n";
  for(bool vehicle:{true,false}) for(const std::string modality:{"exp","gaus"}) {
    for(const std::string reason:{"diverging","stationary","ttc_outside","stc_outside","stc_at_limit","headings_unavailable"}) {
      TestPedestrian provider;
      VRUBasicService service;configure(service,provider,modality);
      uint64_t id=vehicle?12:1006;
      std::string variant=std::string(vehicle?"CAM_":"VAM_")+modality+"_"+reason;
      std::string path=std::string(argv[1])+"/"+variant+".csv";
      std::ofstream(path,std::ios::trunc).close();service.setTIPLogFile(path);
      auto receive=[&](const Geometry& geometry,long rawHeading=0) {
        BTPDataIndication_t incoming={};incoming.data=encodeMessage(vehicle,geometry,rawHeading);
        service.VRUreceivedNewMessageForTIP(incoming,vehicle);
      };
      Geometry invalid={"invalid",0,2,1,false};long rawHeading=0;
      if(reason=="stationary") invalid={"invalid",0,-2,0,false};
      if(reason=="ttc_outside") invalid={"invalid",0,-5,1,false};
      if(reason=="stc_outside") invalid={"invalid",3.2,-2.25,5,false};
      if(reason=="stc_at_limit") invalid={"invalid",3,-2.25,5,false};
      if(reason=="headings_unavailable") {invalid={"invalid",0,-2,1,false};rawHeading=3601;provider.heading=360.1;}
      receive(invalid,rawHeading);
      check(service.m_tip_map.empty()&&service.m_tip_array.empty(),"invalid_does_not_create_entry",variant);
      provider.heading=0;
      Geometry valid={"valid",0,-2,1,true};receive(valid);
      service.searchFirstEightTIPs();
      check(service.m_tip_map.count(id)&&std::get<1>(service.m_tip_map.at(id))>=0&&selected(service,id),"valid_prediction_selected",variant);
      const double current=std::get<1>(service.m_tip_map.at(id));
      service.addNewTIPToMap(id,-100,current);service.updatePreviousSentTIPOnMap(id,.01);
      service.addNewTIPToMap(4242,0,.65);service.searchFirstEightTIPs();
      const auto other=service.m_tip_map.at(4242);
      if(reason=="headings_unavailable") provider.heading=360.1;
      receive(invalid,rawHeading);
      check(service.m_tip_map.count(id)==1,"history_entry_retained",variant);
      check(std::get<1>(service.m_tip_map.at(id))==-1,"current_prediction_invalidated",variant);
      check(close(std::get<2>(service.m_tip_map.at(id)),.01),"last_reported_tip_preserved",variant);
      check(std::get<0>(service.m_tip_map.at(id))==Simulator::Now().GetMilliSeconds(),"timestamp_refreshed",variant);
      check(!selected(service,id),"cached_selection_removed_immediately",variant);
      check(service.m_tip_map.at(4242)==other&&selected(service,4242),"other_station_preserved",variant);
      std::ifstream log(path);std::string line,last,field;while(std::getline(log,line)) last=line;
      std::istringstream parser(last);std::vector<std::string> fields;
      while(std::getline(parser,field,',')) fields.push_back(field);
      check(fields.size()==7&&std::stod(fields[6])==-1,"invalid_evaluation_logged",variant);
      service.searchFirstEightTIPs();
      check(!selected(service,id)&&selected(service,4242),"invalid_prediction_excluded_from_selection",variant);
      provider.heading=0;receive(valid);
      check(close(std::get<1>(service.m_tip_map.at(id)),current)&&close(std::get<2>(service.m_tip_map.at(id)),.01),"valid_prediction_restored_with_history",variant);
      service.searchFirstEightTIPs();
      check(selected(service,id),"renewed_risk_selected",variant);
    }
  }
  Simulator::Destroy();
  return failures?1:0;
}
