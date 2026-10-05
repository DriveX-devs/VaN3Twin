#ifndef DURATION_RV_TAG_H
#define DURATION_RV_TAG_H

#include "ns3/tag.h"
#include "ns3/type-id.h"

namespace ns3
{

class DurationRVTag : public Tag
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::DurationRVTag")
      .SetParent<Tag>()
      .AddConstructor<DurationRVTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }

  void Serialize(TagBuffer i) const override { i.WriteDouble(m_duration); }
  void Deserialize(TagBuffer i) override { m_duration = i.ReadDouble(); }
  uint32_t GetSerializedSize() const override { return sizeof(double); }
  void Print(std::ostream& os) const override { os << "Duration=" << m_duration; }

  void SetDuration(double a) { m_duration = a; }
  double GetDuration() const { return m_duration; }

private:
  double m_duration = 0.0;
};

} // namespace ns3

#endif // DURATION_RV_TAG_H