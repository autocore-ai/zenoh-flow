#include "cxx_test/cxx_test.hpp"

std::string cxx_test_string(double a, double b) {
    double add_result = a + b;
    double mul_result = a * b;
    
    std::string result_str("add result = " + std::to_string(add_result) + ", mul result = " + std::to_string(mul_result));
    return result_str;
}

std::vector<double> cxx_test_vector(double a, double b) {
    std::vector<double> result_vec = std::vector<double>();
    result_vec.push_back(a + b);
    result_vec.push_back(a * b);
    return result_vec;
}