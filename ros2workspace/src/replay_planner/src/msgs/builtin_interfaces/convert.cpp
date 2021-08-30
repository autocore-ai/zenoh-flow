#include "replay_planner/msgs/builtin_interfaces/convert.hpp"

CDuration convert_duration_to_cduration(builtin_interfaces::msg::Duration &duration) {
    CDuration cduration;
    cduration.sec = duration.sec;
    cduration.nanosec = duration.nanosec;
    return cduration;
}

builtin_interfaces::msg::Duration convert_cduration_to_duration(CDuration &cduration) {
    builtin_interfaces::msg::Duration duration;
    duration.sec = cduration.sec;
    duration.nanosec = cduration.nanosec;
    return duration;
}

CTime convert_time_to_ctime(builtin_interfaces::msg::Time &time) {
    CTime ctime;
    ctime.sec = time.sec;
    ctime.nanosec = time.nanosec;
    return ctime;
}

builtin_interfaces::msg::Time convert_ctime_to_time(CTime &ctime) {
    builtin_interfaces::msg::Time time;
    time.sec = ctime.sec;
    time.nanosec = ctime.nanosec;
    return time;
}
