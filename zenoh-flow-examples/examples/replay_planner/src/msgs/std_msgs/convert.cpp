#include "replay_planner/msgs/std_msgs/convert.hpp"
#include "replay_planner/msgs/builtin_interfaces/convert.hpp"
#include <string.h>
CHeader convert_header_to_cheader(std_msgs::msg::Header &header) {
    CHeader cheader;
    cheader.stamp = convert_time_to_ctime(header.stamp);
    // TODO 使用CXX穿越C++和Rust的边界
    cheader.frame_id = header.frame_id.c_str();
    cheader.size = header.frame_id.size();
    return cheader;
}

std_msgs::msg::Header convert_cheader_to_header(CHeader &cheader) {
    std_msgs::msg::Header header;
    header.stamp = convert_ctime_to_time(cheader.stamp);
    header.frame_id = std::string(cheader.frame_id);
    return header;
}
