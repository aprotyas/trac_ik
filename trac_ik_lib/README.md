The TRAC-IK kinematics solver is built in trac\_ik\_lib as a .so library (this
has been tested using ROS Indigo using Catkin).  The headers and shared
objects in this package can be linked against by user programs.

###As of v1.6.0, this package is part of the ROS Noetic binaries: `sudo apt-get install ros-noetic-trac-ik`
###As of v1.4.3, this package is part of the ROS Indigo/Jade binaries: `sudo apt-get install ros-jade-trac-ik`

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

KDL::ChainIkSolverPos_NR_JL ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, fk_solver, vik_solver, int num_iterations, double error);

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints);

% NOTE: CartToJnt succeeded if rc >=0

% NOTE: to use a timeout in seconds (e.g., 0.005), the iterations can be set to 1, and this can be called in a loop with your own timer.

% NOTE: error == 1e-5 is acceptable for most purposes
```

TRAC-IK:

```c++
#include <trac_ik/trac_ik.hpp>

TRAC_IK::TRAC_IK ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);  

% OR

TRAC_IK::TRAC_IK ik_solver(string base_link, string tip_link, string URDF_param="/robot_description", double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);  

% NOTE: The last arguments to the constructors are optional.
% The type can be one of the following: 
% Speed: returns very quickly the first solution found
% Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
% Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
% Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.

int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);

% NOTE: CartToJnt succeeded if rc >=0	

% NOTE: tolerances on the end effector pose are optional, and if not
% provided, then by default are 0.  If given, the ABS() of the
% values will be used to set tolerances at -tol..0..+tol for each of
% the 6 Cartesian dimensions of the end effector pose.
```


