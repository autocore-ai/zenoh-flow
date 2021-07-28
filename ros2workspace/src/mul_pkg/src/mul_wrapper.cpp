#include "mul_pkg/mul_lib.hpp"

extern "C" {
  #include "mul_pkg/mul_wrapper.hpp"
}

double mul_lib_wrapper(double number1, double number2) {
    std_msgs::msg::Float64 float1 = std_msgs::msg::Float64();
    std_msgs::msg::Float64 float2 = std_msgs::msg::Float64();
    float1.set__data(number1);
    float2.set__data(number2);
    return lib_mul(float1, float2).data;
}