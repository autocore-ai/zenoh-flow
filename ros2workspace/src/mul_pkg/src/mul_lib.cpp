#include "mul_pkg/mul_lib.hpp"

std_msgs::msg::Float64 lib_mul(std_msgs::msg::Float64 number1, std_msgs::msg::Float64 number2) {
    std_msgs::msg::Float64 result = std_msgs::msg::Float64();
    result.set__data(number1.data * number2.data);
    return result;
}
