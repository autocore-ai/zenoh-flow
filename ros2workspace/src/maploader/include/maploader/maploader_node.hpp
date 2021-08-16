#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "maploader/maploader_provider.hpp"
#include <memory>
#include <string>

#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/msg/had_map_bin.hpp"

using autoware::common::types::float64_t;

struct MapConfig
{
    std::string map_osm_file;
    float64_t origin_offset_lat;
    float64_t origin_offset_lon;
    float64_t latitude;
    float64_t longitude;
    float64_t elevation;
};


void start_service(const char* map_osm_file_ptr, const float64_t origin_offset_lat, 
    const float64_t origin_offset_lon, const float64_t latitude, const float64_t longitude, const float64_t elevation);

void stop_service();

class Lanelet2MapProviderNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapProviderNode(const rclcpp::NodeOptions & options);

  void handle_request(
    std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Request> request,
    std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Response> response);

private:
  std::unique_ptr<Lanelet2MapProvider> m_map_provider;
  rclcpp::Service<autoware_auto_msgs::srv::HADMapService>::SharedPtr m_map_service;
};

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_NODE_HPP_
