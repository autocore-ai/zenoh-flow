#ifndef builtin_interfaces_convert
#define builtin_interfaces_convert

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "replay_planner/msgs/builtin_interfaces/CDuration.hpp"
#include "replay_planner/msgs/builtin_interfaces/CTime.hpp"

CDuration convert_duration_to_cduration(builtin_interfaces::msg::Duration &duration);
builtin_interfaces::msg::Duration convert_cduration_to_duration(CDuration &cduration);

CTime convert_time_to_ctime(builtin_interfaces::msg::Time &time);
builtin_interfaces::msg::Time convert_ctime_to_time(CTime &ctime);

#endif