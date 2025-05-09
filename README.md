# invkin
This is a header-only library for simple two-joint inverse kinematics simulations.

# Features
- Single header file `ik2j.h`
- C++11 Compatible
- Zero Safe

# Usage
```cpp
// This is a cartesian point z="up" x="right" y="forward"
auto point = IK::CylPoint::from_cartesian(10, 0, 1);
// The solution is a IKResult, the valid field indicates a solution is valid
auto solution = IK::solve(point, close_len, far_len);

// All of the following fields are only considered valid if the 'valid' field is true
// Table Angle - "Which direction are we facing"
solution.table_angle;
// Near Angle - "Base Angle relative to an orthogonal angle to the ground"
solution.near_angle;
// Far Angle - "Angle delta of the second joing tangent to the near segment"
solution.far_angle;
```
