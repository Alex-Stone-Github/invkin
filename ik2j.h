#pragma once

/// This library assumes that all angles are given in radians
#include <cmath>
#include <cassert>

namespace IK {

/// @brief Represents a cylindrical coordinate
struct CylPoint {
    double theta;
    double radius;
    double height;
    static CylPoint from_cartesian(double x, double y, double z) {
        return CylPoint {
            .theta = atan2(y, x),
            .radius = sqrt(x*x + y*y),
            .height = z,
        };
    }
};

/// @brief Represents a triangle, does not necessarily have to be a "valid" triangle
struct Triangle {
    double a, b, c;
    double A, B, C;
};

/// @brief Given an incomplete triangle, this function solves(by mutating) the angles of a triangle.
/// @param tri Incomplete Triangle
/// @return Whether or not the triangle exists
auto solve_triangle_angles(Triangle& tri) -> bool {
    /// All sides of a triangle must have a positive, nonzero length
    assert(tri.a > 0 && tri.b > 0 && tri.c > 0);
    // Law of cosines c^2 = a^2 + b^2 - 2abcos(C)
    tri.C = acos((tri.a*tri.a + tri.b*tri.b - tri.c*tri.c) / (2 * tri.a * tri.b));
    // Law of sines sinA/a = sinB/b = sinC/c
    tri.A = asin(sin(tri.C)*tri.a/tri.c);
    tri.B = asin(sin(tri.C)*tri.b/tri.c);
    //This is invalid

    return tri.a + tri.b > tri.c && tri.a + tri.c > tri.b && tri.b + tri.c > tri.a;
}

/// @brief These represent an inverse kinematics solution 
struct IKResults {
    /// @brief NearAngle, FarAngle (0rad = continuation)
    double near_angle, far_angle;
    /// @brief Table angle indicates which direction the arm is facing
    double table_angle;
    /// @brief  Is there a valid solution
    bool valid;
};

/// @brief Run an inverse kinematics simulation to determine joint angles to reach a point
/// @param point  The point arm attempts to "reach" to
/// @param nlen The "closest" joint's length
/// @param flen  The "further" joint's length
/// @return The results of a inverse kinematics simulation
auto solve(CylPoint const& point, double nlen, double flen) -> IKResults {
    // Solve relative angles if possible
    auto radius = point.radius;
    auto height = point.height;
    Triangle tri;
    tri.c = sqrt(height * height + radius * radius);
    tri.a = nlen;
    tri.b = flen;
    if (!solve_triangle_angles(tri)) return IKResults{.valid = false};

    // Find "absolute" angles
    IKResults solution{.valid = true};
    auto elevation = atan2(height, radius);
    solution.near_angle = (tri.B + elevation) - M_PI_2;
    solution.far_angle = -(M_PI - tri.C);
    solution.table_angle = point.theta;

    return solution;
}
}