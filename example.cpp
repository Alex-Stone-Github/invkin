#include <iostream>

#include "ik2j.h"

int main() {
    constexpr double close_len = 5; // 5 inches
    constexpr double far_len = 6;   // 6 inches

    auto point = IK::CylPoint::from_cartesian(10, 0, 1);
    auto solution = IK::solve(point, close_len, far_len);

    { // Log everything out
        auto todeg = [](double r) {return r / M_PI * 180.0;};
        std::cout << "Is Valid: " << (solution.valid? "true" : "false") << std::endl;
        std::cout << "Close Angle Deg: " << todeg(solution.near_angle) << std::endl;
        std::cout << "Far Angle Deg: " << todeg(solution.far_angle) << std::endl;
        std::cout << "Table Angle Deg: " << todeg(solution.table_angle) << std::endl;
    }

    return 0;
}
