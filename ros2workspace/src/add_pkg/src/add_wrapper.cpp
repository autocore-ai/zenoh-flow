#include "add_pkg/add_lib.hpp"

extern "C" {
  #include "add_pkg/add_wrapper.hpp"
}

double add_lib_wrapper(double number1, double number2) {
    std_msgs::msg::Float64 float1 = std_msgs::msg::Float64();
    std_msgs::msg::Float64 float2 = std_msgs::msg::Float64();
    float1.set__data(number1);
    float2.set__data(number2);
    return lib_add(float1, float2).data;
}