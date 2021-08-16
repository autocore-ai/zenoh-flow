#include "maploader/maploader_provider.hpp"

#include <string>

#include "common/types.hpp"
#include "had_map_utils/had_map_utils.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "lanelet2_core/primitives/GPSPoint.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using autoware::common::types::float64_t;

Lanelet2MapProvider::Lanelet2MapProvider(
  const std::string & map_filename,
  const LatLonAlt map_frame_origin,
  const float64_t offset_lat, const float64_t offset_lon)
{
  LatLonAlt adjusted_origin{map_frame_origin.lat + offset_lat, map_frame_origin.lon + offset_lon,
    map_frame_origin.alt};
    std::cout << "Load Map"<< std::endl;
    this->load_map(map_filename, adjusted_origin);
}

void Lanelet2MapProvider::load_map(
  const std::string & map_filename, const LatLonAlt map_frame_origin)
{
  lanelet::ErrorMessages errors;
  lanelet::GPSPoint originGps{map_frame_origin.lat, map_frame_origin.lon, map_frame_origin.alt};
  lanelet::Origin origin{originGps};

  lanelet::projection::UtmProjector projector(origin);
  m_map = lanelet::load(map_filename, projector, &errors);
  autoware::common::had_map_utils::overwriteLaneletsCenterline(m_map, true);
}
