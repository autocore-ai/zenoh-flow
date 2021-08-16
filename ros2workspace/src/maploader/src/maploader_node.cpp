#include "maploader/maploader_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <common/types.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <utility>

#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/msg/had_map_bin.hpp"
#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"

#include <unistd.h>

using autoware::common::types::bool8_t;


void start_service(const char* map_osm_file_ptr, const float64_t origin_offset_lat, 
    const float64_t origin_offset_lon, const float64_t latitude, const float64_t longitude, const float64_t elevation) {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor executor;

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

    std::cout << "Create Node"<< std::endl;
    const auto map_node_ptr =
        std::make_shared<Lanelet2MapProviderNode>(options);
    std::cout << "Create Node Complete"<< std::endl;
    executor.add_node(map_node_ptr);
    std::cout << "Add Node To Executor"<< std::endl;
    std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));
    std::cout << "Thread Run"<< std::endl;
    executor_thread.detach();
    std::cout << "Thread Detach"<< std::endl;
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
    autoware_auto_msgs::msg::HADMapBin msg;
    msg.header.frame_id = "map";

    autoware::common::had_map_utils::toBinaryMsg(m_map_provider->m_map, msg);
    response->map = msg;
    return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapProviderNode)
