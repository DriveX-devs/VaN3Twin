#ifndef LDMUTILS_H
#define LDMUTILS_H

#include <unordered_map>
#include <vector>
#include <deque>
#include <stdint.h>
#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/geometries.hpp>
//#include <boost/geometry/geometries/register/point.hpp>
//#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>



namespace ns3 {
/**
 * \ingroup automotive
 *
 * @brief Class to store optional data
 *
 * If the data is not available, m_available is 'false' and no actual data is stored (getData() does not return any meaningful data)
 * If the data is available (isAvailable() returns 'true'), then the actual data can be retrieved with getData()
 *
 */
  template <class T> class OptionalDataItem
  {
          private:
                  bool m_available;
                  T m_dataitem;

	  public:
		  OptionalDataItem(T data): m_dataitem(data) {m_available=true;}
		  OptionalDataItem(bool availability) {m_available=availability;}
		  OptionalDataItem() {m_available=false;}
		  T getData() {return m_dataitem;}
		  bool isAvailable() {return m_available;}
		  T setData(T data) {m_dataitem=data; m_available=true;return m_dataitem;}
  };
  /**
   * \ingroup automotive
   *
   * @brief Structure to store the data of a vehicle in the LDM
   *
   * This structure contains all the data stored in the database for each vehicle (except for the PHPoints)
   */

  typedef enum {
      itsType_vehicle = 0,
      itsType_pedestrian = 1 ,
      itsType_bicycle = 2,
      itsType_motorcycle = 3,
      itsType_unknown = 4
  } itsType_t;

  typedef struct _vehicleData {
          bool detected;
          itsType_t itsType;
          uint64_t stationID;
          std::string ID;
          double lat;
          double lon;
          double x;
          double y;
          double elevation;
          double heading; // Heading between 0 and 360 degrees
          double speed_ms;
          long camTimestamp; // This is the CAM message GenerationDeltaTime
          long vamTimestamp; // This is the VAM message GenerationDeltaTime
          uint64_t timestamp_us;// Entry last update
          uint64_t age_us; //Entry lifetime
          long stationType;
          OptionalDataItem<long> vehicleWidth;
          OptionalDataItem<long> vehicleLength;
          // Low frequency container data
          OptionalDataItem<uint8_t> exteriorLights; // Bit string with exterior lights status

          //For detected objects only
          OptionalDataItem<long> xDistance;
          OptionalDataItem<long> yDistance;
          OptionalDataItem<long> xSpeed;
          OptionalDataItem<long> ySpeed;
          OptionalDataItem<long> longitudinalAcceleration;
          OptionalDataItem<long> confidence;
          OptionalDataItem<long> angle;
          OptionalDataItem<uint64_t> lastCPMincluded;
          OptionalDataItem<double> GTaccuracy;

          //patch for v2
          OptionalDataItem<long> xDistAbs;
          OptionalDataItem<long> yDistAbs;
          OptionalDataItem<long> xSpeedAbs;
          OptionalDataItem<long> ySpeedAbs;
          OptionalDataItem<long> xAccAbs;
          OptionalDataItem<long> yAccAbs;


          OptionalDataItem<std::vector<long>> associatedCVs;
          OptionalDataItem<long> perceivedBy;
          bool VAM;
  } vehicleData_t;

  /**
   * \ingroup automotive
   *
   * @brief Information stored for each PH point
   *
   */
  typedef struct PHData {
          double lat;
          double lon;
          double heading;
          double speed_ms;
          double timestamp;
          bool detected;
          uint64_t stationID;
          OptionalDataItem<long> perceivedBy;
          bool CPMincluded;
  } PHData_t;

  typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;

  using polygon_type = boost::geometry::model::polygon<point_type>;
  using linestring_type = boost::geometry::model::linestring<point_type>;

  using polygon_type = boost::geometry::model::polygon<point_type>;
  using linestring_type = boost::geometry::model::linestring<point_type>;

  typedef struct _vehiclePoints {
    point_type center;
    point_type front_left;
    point_type front_right;
    point_type back_right;
    point_type back_left;
  } vehiclePoints_t;

  typedef struct _POData {
          uint64_t objectID;
          uint64_t stationID; // Of the vehicle that perceived this objectID
          double lat;
          double lon;
          double elevation;
          double heading; // Heading between 0 and 360 degrees
          double speed_ms;
          uint64_t timestamp_us;
          long vehicleWidth;
          long vehicleLength;
          //For detected objects only
          long xDistance;
          long yDistance;
          long xSpeed;
          long ySpeed;
          long longitudinalAcceleration;
          long confidence;
          long angle;
  } POData_t;

  typedef enum {
          format_vLDM,
          format_CPM,
          format_pLDM,
          format_PLU,
          format_PMU
  } format_t;

  struct Position
  {
      using value_type = boost::units::quantity<boost::units::si::length>;

      Position() = default;
      Position(const Position&) = default;
      Position& operator=(const Position&) = default;

      explicit Position(double px, double py) :
          x(px * boost::units::si::meter),
          y(py * boost::units::si::meter) {}

      Position (value_type px , value_type py):
          x(px), y(py) {}

      value_type x;
      value_type y;
  };


}

#endif // LDMUTILS_H
