#ifndef OPENCDA_SENSOR_H
#define OPENCDA_SENSOR_H

#include "ns3/OpenCDAClient.h"
#include "ns3/ldm-utils.h"
#include "ns3/phPoints.h"
#include "ns3/core-module.h"
#include "ns3/LDM.h"
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>


namespace ns3 {

  class OpenCDASensor: public Object
  {
  public:
    OpenCDASensor();
    OpenCDASensor(Ptr<OpenCDAClient> opencda_client, std::string id);
    OpenCDASensor(Ptr<OpenCDAClient> opencda_client, int id);
    ~OpenCDASensor();

    void setStationID(std::string id){m_string_id=id;m_id=std::stoi(id);}
    void setStationID(int id){m_string_id=std::to_string (id);m_id=id;}
    void setOpenCDAClient(Ptr<OpenCDAClient> opencda_client){m_opencda_client=opencda_client;m_event_updateDetectedObjects = Simulator::Schedule(MilliSeconds (100),&OpenCDASensor::updateDetectedObjects,this);}
    void setVDP(VDP* vdp) {m_vdp=vdp;}

    void updateDetectedObjects();
    double insertCV(vehicleData_t vehData);
    double insertPOs(std::vector<vehicleData_t> POs, uint64_t fromId);
    void setLDM(Ptr<LDM> ldm){m_LDM=ldm;}
    void enableGUI(bool visualize);
    void updateGUI();
    void createCluster();
    void logPerceivedPedestrians();
    double get_circular_mean_heading(const std::vector<double>& headings_deg);
//    double get_angular_difference(double heading1_deg, double heading2_deg);

  private:

    void insertObject(vehicleData_t vehData);
    carla::ObjectIn createCARLAObjectIn(vehicleData_t vehData);
    carla::Object createCARLAObject(vehicleData_t vehData);

    // Function to calculate the intersection area of two polygons
    double calculateIoU(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2) ;

    Ptr<OpenCDAClient> m_opencda_client;
    std::string m_string_id;
    int m_id;
    Ptr<LDM> m_LDM;
    VDP* m_vdp;
    double m_avg_dwell = 0.0;
    int m_dwell_count = 0;
    EventId m_event_updateDetectedObjects;
    std::map<uint64_t,uint64_t> m_lastInserted;  //! Auxiliary map to avoid inserting the same perception in OpenCDA's LDM more than once
    bool m_GUI = false;
    bool m_pldm = false;
    bool m_visualize;
    cv::Mat m_cv_image;
    uint64_t m_count;
    bool m_clustering;
    std::map<std::string, std::vector<int>> g_perceived_pedestrians;

  };
}

#endif // OPENCDASENSOR
