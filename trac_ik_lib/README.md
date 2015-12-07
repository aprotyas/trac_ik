The TRAC-IK kinematics solver is built in trac\_ik\_lib as a .so library (this
has been tested using ROS Indigo using Catkin).  The headers and shared
objects in this package can be linked against by user programs.

This requires the Ubuntu packages for NLOpt Libraries to be installed (the
ros-indigo-nlopt packages do not use proper headers).  This can be done by
running ```sudo apt-get install libnlopt-dev``` on the trusty (and later)
standard Ubuntu distros.  Alternatively, you can run ```rosdep update &&
rosdep install trac_ik_lib```.

KDL IK:

```c++
KDL::ChainFkSolverPos_recursive fk_solver(chain);
KDL::ChainIkSolverVel_pinv vik_solver(chain);
KDL::ChainJntToJacSolver jac_solver(chain);

KDL::ChainIkSolverPos_NR_JL ik_solver(KDL::Frame chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, fk_solver, vik_solver, int num_iterations, double error);

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints);

% NOTE: CartToJnt succeeded if rc >=0

% NOTE: to set a timeout in ms, the iterations can be set to 1, and this
% can be called in a loop without seeing any real penalty in runtime.

% NOTE: error == 1e-3 is acceptable for most purposes
```

TRAC-IK:

```c++
#include <trac_ik/trac_ik.hpp>

TRAC_IK::TRAC_IK ik_solver(KDL::Frame chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_ms, double error, TRAC_IK::SolveType type=TRAC_IK::Speed);  

% NOTE: The last argument to the constructor is optional, and can be one of the following: 
% Speed: returns very quickly the first solution found
% Distance: runs for the full timeout_in_ms, then returns the solution that minimizes SSE from the seed
% Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T))
% Manip2: runs for full timeout, returns solution that minimizes cond(J) = |J|*|J^-1|

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);

% NOTE: CartToJnt succeeded if rc >=0	

% NOTE: tolerances on the end effector pose are optional, and if not
% provided, then by default are 0.  If given, the ABS() of the
% values will be used to set tolerances at -tol..0..+tol for each of
% the 6 Cartesian dimensions of the end effector pose.
```


