//
// Created by diego on 13/03/26.
//

#ifndef NS3_TRAJECTORYPREDICTION_H
#define NS3_TRAJECTORYPREDICTION_H

#include "vector"
#include "tuple"
#include "cmath"
#include "ns3/nstime.h"


namespace ns3 {
class trajectoryPrediction
{

public:
  typedef struct
  {
    Time time;
    double x;
    double y;
    double speed;
    double acceleration;
  } TrajectoryItem;

  enum ActorType {
    HV,
    RV,
    HVAhead,
    RVAhead,
  };

  trajectoryPrediction (int horizon_time, int step_time, int negotiation_time, int deceleration_time);
  ~trajectoryPrediction() = default;
  std::tuple<trajectoryPrediction::TrajectoryItem, std::vector<trajectoryPrediction::TrajectoryItem>>
  predictConstantSpeed (double x, double y, double speed, double comfort_acceleration, int8_t sign, ActorType type);
  std::vector<trajectoryPrediction::TrajectoryItem>
  predictConstantAcceleration (double x, double speed, double comfort_deceleration, double acceleration, int8_t sign, bool is_RV = false);
  double IDMAcceleration(double v, double v_lead, double gap, double v0, double a_max, double b, double s0, double T, double delta = 4.0);
  std::tuple<double, std::vector<double>> estimateTimeFromPredictionIDM(std::vector<TrajectoryItem> leader, std::vector<TrajectoryItem> follower, double a_max, double desired_speed, double b, double s0, double T);

private:
  int m_horizon_time;
  int m_step_time; // In ms
  // double m_comfort_deceleration;
  int m_negotiation_time;
  int m_deceleration_time;

};
}

#endif //NS3_TRAJECTORYPREDICTION_H
