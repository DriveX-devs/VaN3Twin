#ifndef DECELERATION_RV_TAG_H
#define DECELERATION_RV_TAG_H

#include "ns3/tag.h"
#include "ns3/type-id.h"

namespace ns3
{

class DecelerationRVTag : public Tag
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::DecelerationRVTag")
      .SetParent<Tag>()
      .AddConstructor<DecelerationRVTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }

  void Serialize(TagBuffer i) const override { i.WriteDouble(m_acceleration); }
  void Deserialize(TagBuffer i) override { m_acceleration = i.ReadDouble(); }
  uint32_t GetSerializedSize() const override { return sizeof(double); }
  void Print(std::ostream& os) const override { os << "Deceleration=" << m_acceleration; }

  void SetAcceleration(double a) { m_acceleration = a; }
  double GetAcceleration() const { return m_acceleration; }

private:
  double m_acceleration = 0.0;
};

} // namespace ns3

#endif // DECELERATION_RV_TAG_H