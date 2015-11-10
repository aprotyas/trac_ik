The ROS packages in this repository were created to provide an alternative
Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.
Specifically, KDL's convergence algorithms are based on Newton's method, which
does not work well in the presence of joint limits --- common for many robotic
platforms.  TRAC-IK concurrently runs two IK implementations.  One is a simple
extension to KDL's Newton-based convergence algorithm that detects and
mitigates local minima due to joint limits by random jumps.  The second is an
SQP (Sequential Quadratic Programming) nonlinear optimization approach which
uses quasi-Newton methods that better handle joint limits.  Currently, the IK
search returns immediately when either of these algorithms converges to an
answer.  Future work plans to allow secondary constraints and sorting
functions to be provided to the main TRAC-IK call in order to receive back the
"best" IK solution.

#Note: 

While TRAC-IK is an improvement over KDL's IK for many joint-limited chains,
TRAC-IK does **not** significantly improve IK results over KDL for the PR2 arm
chains.  So, if your research focuses on the PR2 platform (real or simulated),
it may not be worthwhile changing your code to use TRAC-IK.  *In our tests,
KDL finds 98.75% IK solutions on 10,000 random PR2 arm configurations with an
average of 0.27 ms per successful solve.  TRAC-IK solves 100% of the same
random samples with an average of 0.34 ms per successful solve.*

###This repo contains 3 ROS packages:###

trac\_ik is simple a metapackage.

The TRAC-IK kinematics code in
[trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib)
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. (This has been tested using ROS Indigo with catkin).

The
[trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin)
package builds a [MoveIt!
plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html)

###Some sample results are below:### 

![results.png](https://bitbucket.org/repo/7eA5MR/images/1410545029-results.png)
