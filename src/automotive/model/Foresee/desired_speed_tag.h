#ifndef DESIRED_SPEED_TAG_H
#define DESIRED_SPEED_TAG_H

#include "ns3/tag.h"
#include "ns3/type-id.h"

namespace ns3
{

class DesiredSpeedTag : public Tag
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::DesiredSpeedTag")
      .SetParent<Tag>()
      .AddConstructor<DesiredSpeedTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }

  void Serialize(TagBuffer i) const override { i.WriteDouble(m_desiredSpeed); }
  void Deserialize(TagBuffer i) override { m_desiredSpeed = i.ReadDouble(); }
  uint32_t GetSerializedSize() const override { return sizeof(double); }
  void Print(std::ostream& os) const override { os << "DesiredSpeed=" << m_desiredSpeed; }

  void SetDesiredSpeed(double v) { m_desiredSpeed = v; }
  double GetDesiredSpeed() const { return m_desiredSpeed; }

private:
  double m_desiredSpeed = 0.0;
};

} // namespace ns3

#endif // DESIRED_SPEED_TAG_H