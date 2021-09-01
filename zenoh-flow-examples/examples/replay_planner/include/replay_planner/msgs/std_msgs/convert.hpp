#ifndef std_msgs_convert
#define std_msgs_convert

#include <std_msgs/msg/header.hpp>

#include "replay_planner/msgs/std_msgs/CHeader.hpp"

CHeader convert_header_to_cheader(std_msgs::msg::Header &header);
std_msgs::msg::Header convert_cheader_to_header(CHeader &cheader);

#endif