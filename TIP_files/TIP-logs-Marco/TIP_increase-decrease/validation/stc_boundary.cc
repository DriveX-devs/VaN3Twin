#define main headingRegressionMain
#include "/home/marco/Desktop/GitHub_projects/VaN3Twin/ns-3-dev/TIP_files/TIP-logs-Marco/.validation/heading-unavailable/heading_boundary.cc"
#undef main
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::vector<Geometry> cases={
    {"zero",0,-2,1,true},{"tiny_positive",.0000001,-2,1,true},
    {"positive",.1,-2,1,true},{"receding",0,2,1,false},
    {"stationary",0,-2,0,false},{"colocated",0,0,1,false},
    {"ttc_at_limit",0,-4,1,false},{"stc_at_limit",3,-2.25,5,false},
    {"stc_above_limit",3.2,-2.25,5,false}};
  int failures=0;
  std::cout<<"protocol,modality,case,tip,pass\n";
  for(bool vehicle:{true,false})for(const std::string modality:{"exp","gaus"})for(const auto& g:cases){
    TestPedestrian provider;VRUBasicService service;configure(service,provider,modality);
    const std::string path=std::string(argv[1])+"/"+(vehicle?"CAM_":"VAM_")+modality+"_"+g.name+".csv";
    std::ofstream(path,std::ios::trunc).close();service.setTIPLogFile(path);
    BTPDataIndication_t indication{};indication.data=encodeMessage(vehicle,g);
    service.VRUreceivedNewMessageForTIP(indication,vehicle);
    std::ifstream log(path);std::string line,field;std::getline(log,line);
    std::istringstream parser(line);std::vector<std::string> fields;
    while(std::getline(parser,field,','))fields.push_back(field);
    if(fields.size()!=7)throw std::runtime_error("Bad TIP log");
    const double tip=std::stod(fields[6]);
    vehicleData_t remote{};remote.heading=0;remote.speed_ms=g.speed;remote.lon=g.x;remote.lat=g.y;
    const auto [ttc,stc]=service.compute_ttc_stc(remote);
    const double expected=modality=="exp"?std::min(1.,std::exp(-.27*(ttc-1))):std::exp(-ttc*ttc/2);
    const bool pass=g.eligible?tip>=0&&close(tip,expected):tip==-1;
    failures+=!pass;
    std::cout<<(vehicle?"CAM":"VAM")<<','<<modality<<','<<g.name<<','<<tip<<','<<(pass?"PASS":"FAIL")<<'\n';
  }
  Simulator::Destroy();return failures?1:0;
}
