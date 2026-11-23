#define main headingRegressionMain
#include "/home/marco/Desktop/GitHub_projects/VaN3Twin/ns-3-dev/TIP_files/TIP-logs-Marco/.validation/heading-unavailable/heading_boundary.cc"
#undef main

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  std::ifstream input(argv[1]);std::string line;std::getline(input,line);
  std::cout<<"dataset\tcase\tcapture_time\tdecoded\tstation\tgenerationDeltaTime\tsubjects\tprobabilities\tconfidences\n";
  while(std::getline(input,line)) {
    std::istringstream parser(line);std::vector<std::string> fields;std::string field;
    while(std::getline(parser,field,'\t'))fields.push_back(field);
    if(fields.size()!=4)throw std::runtime_error("Invalid input");
    std::string payload;
    for(size_t i=0;i<fields[3].size();i+=2)payload.push_back(static_cast<char>(std::stoul(fields[3].substr(i,2),nullptr,16)));
    auto message=asn1cpp::uper::decodeASN(payload,VAM);
    std::cout<<fields[0]<<'\t'<<fields[1]<<'\t'<<fields[2]<<'\t';
    if(!message){std::cout<<"false\t\t\t\t\t\n";continue;}
    std::string subjects,probabilities,confidences;
    auto* motion=message->vam.vamParameters.vruMotionPredictionContainer;
    if(motion&&motion->trajectoryInterceptionIndication) {
      const auto& list=motion->trajectoryInterceptionIndication->list;
      for(int i=0;i<list.count;++i) {
        if(i){subjects+=",";probabilities+=",";confidences+=",";}
        auto* item=list.array[i];
        subjects+=item->subjectStation?std::to_string(*item->subjectStation):"unavailable";
        probabilities+=std::to_string(item->trajectoryInterceptionProbability);
        confidences+=item->trajectoryInterceptionConfidence?std::to_string(*item->trajectoryInterceptionConfidence):"unavailable";
      }
    }
    std::cout<<"true\t"<<message->header.stationId<<'\t'<<message->vam.generationDeltaTime<<'\t'<<subjects<<'\t'<<probabilities<<'\t'<<confidences<<'\n';
  }
}
