The ROS packages in this repository were created to provide an alternative
Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.
Specifically, KDL's convergence algorithms are based on Newton's method, which
does not work well in the presence of joint limits---common for many robotic
platforms.  TRAC-IK concurrently runs two IK implementations.  One is a simple
extension to KDL's Newton-based convergence algorithm that detects and
mitigates local minima due to joint limits by random jumps.  The second is an
SQP (Sequential Quadratic Programming) nonlinear optimization approach which
uses quasi-Newton methods that better handle joint limits.  Currently, the IK
search returns immediately when either of these algorithms converges to an
answer.  Future work plans to allow secondary constraints and sorting
functions to be provided to the main TRAC-IK call in order to receive back the
"best" IK solution.

###This repo contains 2 ROS packages:###

The TRAC-IK kinematics code in
[trac_ik_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib)
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. (This has been tested using ROS Indigo with catkin).

The
[trac_ik_kinematics_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin)
package builds a [MoveIt!
plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.

###A detailed writeup on TRAC-IK can be found here:###
[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html)

###Some sample results are below:###
![results.png](https://bitbucket.org/repo/7eA5MR/images/1410545029-results.png)
