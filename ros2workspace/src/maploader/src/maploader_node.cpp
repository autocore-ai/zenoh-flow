#include "maploader/maploader_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <common/types.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <utility>

#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"

#include <unistd.h>

#include <boost/interprocess/managed_shared_memory.hpp> 

#include "had_map_utils/had_map_utils.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "lanelet2_core/primitives/GPSPoint.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using autoware::common::types::bool8_t;

void *start_service(const char* map_osm_file_ptr, const float64_t origin_offset_lat, 
    const float64_t origin_offset_lon, const float64_t latitude, const float64_t longitude, const float64_t elevation) {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;

    std::string map_osm_file(map_osm_file_ptr);

    std::cout << "C++ Library Received Config:" << std::endl;

    std::cout << "map_filename:" << map_osm_file<< std::endl;
    std::cout << "origin_offset_lat:" << origin_offset_lat<< std::endl;
    std::cout << "origin_offset_lon:" << origin_offset_lon<< std::endl;
    std::cout << "origin_lat:" << latitude<< std::endl;
    std::cout << "origin_lon:" << longitude<< std::endl;
    std::cout << "origin_alt:" << elevation<< std::endl;


    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
    paramters.push_back(rclcpp::Parameter("map_osm_file", map_osm_file));
    paramters.push_back(rclcpp::Parameter("origin_offset_lat", origin_offset_lat));
    paramters.push_back(rclcpp::Parameter("origin_offset_lon", origin_offset_lon));
    paramters.push_back(rclcpp::Parameter("latitude", latitude));
    paramters.push_back(rclcpp::Parameter("longitude", longitude));
    paramters.push_back(rclcpp::Parameter("elevation", elevation));
    options.parameter_overrides(paramters);

    const auto map_node_ptr = std::make_shared<Lanelet2MapProviderNode>(options);


    MapConfig map_config = MapConfig{map_osm_file,origin_offset_lat, origin_offset_lon, latitude, longitude, elevation};
    boost::interprocess::shared_memory_object::remove("SharedMemory");
    boost::interprocess::managed_shared_memory managed_shm(boost::interprocess::open_or_create, "SharedMemory", 1024);
    managed_shm.construct<MapConfig>("MapConfig")(map_config);



    std::cout << "Create Node"<< std::endl;
    std::thread * executor_thread = new std::thread(start_thread, map_node_ptr);
    std::cout << "Thread Run"<< std::endl;
    executor_thread->detach();
    std::cout << "Thread Detach"<< std::endl;

    sleep(5);
    std::cout << "Sleep Complete"<< std::endl;
    return executor_thread;
}

void start_thread(std::shared_ptr<Lanelet2MapProviderNode> map_node_ptr){
    rclcpp::spin(map_node_ptr);
}

void stop_service() {
    if(rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

Lanelet2MapProviderNode::Lanelet2MapProviderNode(const rclcpp::NodeOptions & options)
: Node("MapLoaderProvider", options)
{
    const std::string map_filename = declare_parameter("map_osm_file").get<std::string>();
    const float64_t origin_offset_lat = declare_parameter("origin_offset_lat", 0.0);
    const float64_t origin_offset_lon = declare_parameter("origin_offset_lon", 0.0);

    if (has_parameter("latitude") && has_parameter("longitude") && has_parameter("elevation")) {
        const float64_t origin_lat = declare_parameter("latitude").get<float64_t>();
        const float64_t origin_lon = declare_parameter("longitude").get<float64_t>();
        const float64_t origin_alt = declare_parameter("elevation").get<float64_t>();
        LatLonAlt map_origin{origin_lat, origin_lon, origin_alt};

        m_map_provider = std::make_unique<Lanelet2MapProvider>(
            map_filename, map_origin, origin_offset_lat, origin_offset_lon);
    }

    m_map_service =
        this->create_service<autoware_auto_msgs::srv::HADMapService>(
            "HAD_Map_Service", std::bind(
            &Lanelet2MapProviderNode::handle_request, this,
        std::placeholders::_1, std::placeholders::_2));
}

void Lanelet2MapProviderNode::handle_request(
  std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Request> request,
  std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Response> response)
{
    std::cout << "Send Map Start"<< std::endl;
    auto primitive_sequence = request->requested_primitives;

    autoware_auto_msgs::msg::HADMapBin msg;
    msg.header.frame_id = "map";
    std::cout << "Start Open SharedMemory"<< std::endl;
    boost::interprocess::managed_shared_memory managed_shm(boost::interprocess::open_only, "SharedMemory");
    std::cout << "Find Lanelet2MapProvider"<< std::endl;

    // std::pair<int*, std::size_t> p = managed_shm.find<int>("Integer");
    // if (p.first) {
    //     std::cout << *p.first << std::endl;
    // }
    MapConfig *map_config = managed_shm.find<MapConfig>("MapConfig").first;

    LatLonAlt adjusted_origin{map_config->latitude + map_config->origin_offset_lat, map_config->longitude + map_config->origin_offset_lon, map_config->elevation};
    std::cout << "Load Map in handle_request"<< std::endl;

    lanelet::ErrorMessages errors;
    lanelet::GPSPoint originGps{adjusted_origin.lat, adjusted_origin.lon, adjusted_origin.alt};
    lanelet::Origin origin{originGps};
    lanelet::projection::UtmProjector projector(origin);

    std::shared_ptr<lanelet::LaneletMap> m_map = lanelet::load(map_config->map_osm_file, projector, &errors);
    autoware::common::had_map_utils::overwriteLaneletsCenterline(m_map, true);

    autoware::common::had_map_utils::toBinaryMsg(m_map, msg);
    response->map = msg;
    std::cout << "Send Map Complete"<< std::endl;
    return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapProviderNode)
