#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/types.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <memory>

#include "autoware_auto_msgs/msg/had_map_bin.hpp"

using autoware::common::types::float64_t;

struct LatLonAlt
{
  float64_t lat;  ///< latitude in degrees
  float64_t lon;  ///< longitude in degrees
  float64_t alt;  ///< altitude in meters
};


class Lanelet2MapProvider
{
public:
  Lanelet2MapProvider(
    const std::string & map_filename, const LatLonAlt map_frame_origin,
    const float64_t offset_lat = 0.0,
    const float64_t offset_lon = 0.0);

  std::shared_ptr<lanelet::LaneletMap> m_map;

private:
  void load_map(
    const std::string & map_filename, const LatLonAlt map_frame_origin);
};

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
