#ifndef MAPLOADER_WRAPPER
#define MAPLOADER_WRAPPER

#include "maploader/maploader_node.hpp"

void *start_maploader_service(const char* map_osm_file_ptr, const float64_t origin_offset_lat, 
    const float64_t origin_offset_lon, const float64_t latitude, const float64_t longitude, const float64_t elevation);
void stop_maploader_service();

#endif //MAPLOADER_WRAPPER