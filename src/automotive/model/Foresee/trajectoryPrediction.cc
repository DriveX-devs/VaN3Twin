//
// Created by diego on 13/03/26.
//

#include "trajectoryPrediction.h"

namespace ns3
{

trajectoryPrediction::trajectoryPrediction (double horizon_time, double step_time,
                                            double negotiation_time, double deceleration_time)
{
  m_horizon_time = horizon_time;
  m_step_time = step_time;
  m_negotiation_time = negotiation_time;
  m_deceleration_time = deceleration_time;
}

std::vector<trajectoryPrediction::TrajectoryItem>
trajectoryPrediction::predictConstantSpeed(double x, double speed, double comfort_acceleration, int8_t sign, ActorType type)
{
  std::vector<trajectoryPrediction::TrajectoryItem> motion_plan;
  if (type == ActorType::HV)
    {
      double t = m_step_time;
      while (t <= m_horizon_time)
        {
          double delta = sign * (speed * m_step_time);
          x += delta;
          TrajectoryItem item {t, x, speed, 0};
          motion_plan.push_back (item);
          t += m_step_time;
        }
    }
  else if (type == ActorType::RV || type == ActorType::RVAhead)
    {
      double t = m_step_time;
      while (t <= m_horizon_time)
        {
          if (t <= m_negotiation_time || t > m_negotiation_time + m_deceleration_time)
            {
              double delta = sign * (speed * m_step_time);
              x += delta;
              TrajectoryItem item {t, x, speed, 0};
              motion_plan.push_back (item);
              t += m_step_time;
            }
          else if (t > m_negotiation_time && t <= m_negotiation_time + m_deceleration_time)
            {
              double delta = sign * (speed * m_step_time + 0.5 * comfort_acceleration * std::pow(m_step_time, 2));
              x += delta;
              double delta_speed = comfort_acceleration * m_step_time;
              speed += delta_speed;
              TrajectoryItem item {t, x, speed, comfort_acceleration};
              motion_plan.push_back (item);
              t += m_step_time;
            }
        }
    }
  return motion_plan;
}

std::vector<trajectoryPrediction::TrajectoryItem>
trajectoryPrediction::predictConstantAcceleration (double x, double speed, double comfort_deceleration, double acceleration, int8_t sign,
                                                   bool is_RV)
{
  std::vector<trajectoryPrediction::TrajectoryItem> motion_plan;
  if (!is_RV)
    {
      double t = m_step_time;
      while (t <= m_horizon_time)
        {
          double delta = sign * (speed * m_step_time + 0.5 * acceleration * std::pow (m_step_time, 2));
          x += delta;
          speed += (acceleration * m_step_time);
          TrajectoryItem item {t, x, speed, acceleration};
          motion_plan.push_back (item);
          t += m_step_time;
        }
    }
  else
    {
      double t = m_step_time;
      while (t <= m_horizon_time)
        {
          if (t <= m_negotiation_time || t > m_negotiation_time + m_deceleration_time)
            {
              double delta = sign * (speed * m_step_time + 0.5 * acceleration * std::pow (m_step_time, 2));
              x += delta;
              speed += (acceleration * m_step_time);
              TrajectoryItem item {t, x, speed, acceleration};
              motion_plan.push_back (item);
              t += m_step_time;
            }
          else if (t > m_negotiation_time && t <= m_negotiation_time + m_deceleration_time)
            {
              double delta = sign * (speed * m_step_time + 0.5 * comfort_deceleration * std::pow(m_step_time, 2));
              x += delta;
              double delta_speed = comfort_deceleration * m_step_time;
              speed += delta_speed;
              TrajectoryItem item {t, x, speed, comfort_deceleration};
              motion_plan.push_back (item);
              t += m_step_time;
            }
        }
    }
  return motion_plan;
}

double trajectoryPrediction::IDMAcceleration(
    double v,
    double v_lead,
    double gap,
    double v0,
    double a_max,
    double b,
    double s0,
    double T,
    double delta
)
{
  // Ensure the gap is not too small to avoid singularities
  gap = std::max(gap, 0.1);

  // Calculate the relative speed and desired gap
  double delta_v = v - v_lead;
  double s_star = s0 + v * T + std::max(0.0, (v * delta_v)) / (2.0 * std::sqrt(a_max * b));

  // Compute the acceleration based on the IDM formula
  double a_exp = a_max * (1.0 - std::pow(v / v0, delta) - std::pow(s_star / gap, 2.0));

  return a_exp;
}

std::tuple<double, std::vector<double>>
trajectoryPrediction::estimateTimeFromPredictionIDM(
    std::vector<TrajectoryItem> leader,
    std::vector<TrajectoryItem> follower,
    double a_max,
    double desired_speed,
    double b,
    double s0,
    double T)
{
  // Determine the minimum size of the leader and follower trajectories
  size_t N = std::min(leader.size(), follower.size());

  std::vector<double> accelerations;
  for (size_t k = 0; k < N; ++k)
    {
      auto lead = leader[k];
      auto foll = follower[k];

      // Calculate the gap between the leader and follower at this timestep
      double gap = std::abs(lead.x - foll.x);

      // Calculate the required braking for the follower
      double a_required_RV = IDMAcceleration(
          foll.speed,   // follower
          lead.speed,   // leader
          gap,
          desired_speed,
          a_max,
          -b,
          s0,
          T
      );
      accelerations.push_back (a_required_RV);

      // Check if the required braking is within comfort limits
      if (a_required_RV >= b)
        {
          return {(k + 1) * m_step_time, accelerations}; // EstimatedTime found
        }
    }

  return {-1.0, accelerations}; // No feasible coordination in time horizon
}
}
