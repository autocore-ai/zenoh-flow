#include "cxx_test/cxx_test.hpp"
#include "cxx_test/cxx_test_wrapper_cpp.hpp"

std::string cxx_test_string_cpp(double a, double b) {
    return cxx_test_string(a, b);
}
std::vector<double> cxx_test_vector_cpp(double a, double b) {
    return cxx_test_vector(a, b);
}
