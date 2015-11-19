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

###This repo contains 3 ROS packages:###

- trac\_ik is simply a metapackage.  

- [trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib), the TRAC-IK kinematics code,
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md.

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html) (reported results are from v1.0.0 of TRAC-IK, see below for newer results).

###Some sample results are below: 

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

###This repo contains 3 ROS packages:###

- trac\_ik is simply a metapackage.  

- [trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib), the TRAC-IK kinematics code,
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md.

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html) (reported results are from v1.0.0 of TRAC-IK, see below for newer results).

###Some sample results are below: 

_Orocos' KDL_ (inverse Jacobian w/ joint limits), _KDL-RR_ (our fixes to KDL joint limit handling), and _TRAC-IK_ (our concurrent inverse Jacobian and non-linear optimization solver) are compared below.

IK success and average speed (for successful solves) as of TRAC-IK tag v1.1.0.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on timings**: The timings below are only for _SUCCESSFUL_ solves, and are meant to illustrate that TRAC-IK does not add significant overhead over KDL in the _WORST CASE_.  When an IK solution is not found, the IK solver implementations run for the full timeout requested.  If all solve requests (successful and unsuccessful) were included in the timing data, then for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times would be skewed to basically be the user provided timeout value (here 5 ms).  So, in general, if the solve rate of TRAC-IK >> KDL, then the overall expected time for TRAC-IK << KDL.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.


Chain | DOFs | Orocos' KDL solve rate | Orocos' KDL Avg Time for success | KDL-RR solve rate | KDL-RR Avg Time for success | TRAC-IK solve rate | TRAC-IK Avg Time for success
- | - | -  | - | - | - | - | -
Atlas 2013 arm | 6 | **75.5%** | 0.16ms | **97.1%** | 0.25ms | **99.8%** | 0.34ms
Atlas 2015 arm | 7 | **75.4%** | 0.4ms | **92.2%** | 0.52ms | **99.5%** | 0.43ms
Baxter arm | 7 | **60.7%** | 0.44ms | **89.1%** | 0.58ms | **99.4%** | 0.54ms
Denso VS-068 | 6 | **27.9%** | 0.27ms | **98.1%** | 0.33ms | **99.9%** | 0.35ms
Fanuc M-430iA/2F | 5 | **21.1%** | 0.18ms | **88.3%** | 0.39ms | **99.3%** | 0.53ms
Fetch arm | 7 | **87.5%** | 0.11ms | **98.4%** | 0.21ms | **100%** | 0.24ms
Jaco2 | 6 | **26.2%** | 0.37ms | **97.7%** | 0.46ms | **99.6%** | 0.49ms
KUKA LBR iiwa 14 R820 | 7 | **37.4%** | 0.69ms | **93.4%** | 0.48ms | **99.9%** | 0.4ms
KUKA LWR 4+ | 7 | **67.5%** | 0.41ms | **94.7%** | 0.43ms | **100%** | 0.35ms
PR2 arm | 7 | **98.8%** | 0.27ms | **99.7%** | 0.28ms | **100%** | 0.34ms
NASA Robonaut2 'grasping leg' | 7 | **60.9%** | 0.59ms | **86.6%** | 0.54ms | **99.5%** | 0.64ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.5%** | 0.79ms | **97.7%** | 0.79ms | **99.8%** | 0.84ms
NASA Robonaut2 arm | 7 | **85.7%** | 0.42ms | **93.4%** | 0.5ms | **99.6%** | 0.44ms
NASA Robosimian arm | 7 | **61.3%** | 0.87ms | **99.9%** | 0.34ms | **99.9%** | 0.45ms
TRACLabs modular arm | 7 | **78.7%** | 0.4ms | **94.5%** | 0.42ms | **100%** | 0.4ms
UR10 | 6 | **36.1%** | 0.25ms | **88.1%** | 0.24ms | **99.8%** | 0.4ms
UR5 | 6 | **35.9%** | 0.23ms | **88.7%** | 0.24ms | **99.5%** | 0.41ms
NASA Valkyrie arm | 7 | **44.8%** | 0.61ms | **88.6%** | 0.92ms | **99.8%** | 0.59ms

Feel free to [email Patrick] (mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
