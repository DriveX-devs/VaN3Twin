//
// Created by diego on 13/03/26.
//

#ifndef NS3_TRAJECTORYPREDICTION_H
#define NS3_TRAJECTORYPREDICTION_H

#include "vector"
#include "tuple"
#include "cmath"

namespace ns3 {
class trajectoryPrediction
{

public:
  typedef struct
  {
    double time;
    double x;
    // double y;
    double speed;
    double acceleration;
  } TrajectoryItem;

  enum ActorType {
    HV,
    RV,
    RVAhead,
  };

  trajectoryPrediction (double horizon_time, double step_time, double negotiation_time, double deceleration_time);
  ~trajectoryPrediction() = default;
  std::vector<trajectoryPrediction::TrajectoryItem>
  predictConstantSpeed (double x, double speed, double comfort_acceleration, int8_t sign, ActorType type);
  std::vector<trajectoryPrediction::TrajectoryItem>
  predictConstantAcceleration (double x, double speed, double comfort_deceleration, double acceleration, int8_t sign, bool is_RV = false);
  double IDMAcceleration(double v, double v_lead, double gap, double v0, double a_max, double b, double s0, double T, double delta = 4.0);
  std::tuple<double, std::vector<double>> estimateTimeFromPredictionIDM(std::vector<TrajectoryItem> leader, std::vector<TrajectoryItem> follower, double a_max, double desired_speed, double b, double s0, double T);

private:
  double m_horizon_time;
  double m_step_time;
  // double m_comfort_deceleration;
  double m_negotiation_time;
  double m_deceleration_time;

};
}

#endif //NS3_TRAJECTORYPREDICTION_H
