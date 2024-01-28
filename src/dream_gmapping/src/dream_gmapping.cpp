#include "simple_robotics_cpp_utils/math_utils.hpp"
#include <iostream>

int main(){
    auto result = SimpleRoboticsCppUtils::draw_from_pdf_normal(1.0, 1.0);
    std::cout << result << std::endl;
}