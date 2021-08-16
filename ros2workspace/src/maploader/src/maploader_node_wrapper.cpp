#include "maploader/maploader_node.hpp"

extern "C" {
  #include "maploader/maploader_node_wrapper.hpp"
}

void start_maploader_service(const char* map_osm_file_ptr, const float64_t origin_offset_lat, 
  const float64_t origin_offset_lon, const float64_t latitude, const float64_t longitude, 
  const float64_t elevation) {
    std::cout << "Start C++ Service"<< std::endl;
    start_service(map_osm_file_ptr, origin_offset_lat, origin_offset_lon, latitude, longitude, elevation);
    std::cout << "Started C++ Service"<< std::endl;
}

void stop_maploader_service(){
    std::cout << "Stop C++ Service"<< std::endl;
    stop_service();
    std::cout << "stoped C++ Service"<< std::endl;
}
