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
functions for KDL chains. (This has been tested using ROS Indigo with catkin).
Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md.

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html)

###Some sample results are below: 

IK success and average speed (for successful solves) as of v1.1.0.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

Chain | DOFs | Orocos' KDL (inverse Jacobian w/ joint limits) | KDL-RR (our fixes to KDL joint limit handling) | TRAC-IK |
- | - | -  | - | - |
PR2 arm | 7 | **98.8%** (0.28ms) | **99.7%** (0.29ms) | **100%** (0.38ms) |
Jaco2 arm | 6 | **26.2%** (0.37ms) | **97.7%** (0.46ms) | **99.7%** (0.48ms) |
NASA Robosimian arm | 7 | **61.4%** (0.86ms) | **99.9%** (0.34ms) | **99.9%** (0.41ms) |
Atlas 2013 arm | 6 | **75.5%** (0.16ms) | **97.1%** (0.25ms) | **99.9%** (0.31ms) |
Atlas 2015 arm | 7 | **75.4%** (0.4ms) | **92.2%** (0.52ms) | **99.3%** (0.46ms) |
Baxter arm | 7 | **60.7%** (0.44ms) | **89.2%** (0.58ms) | **99.4%** (0.56ms) |
Fetch arm | 7 | **87.6%** (0.11ms) | **98.4%** (0.21ms) | **100%** (0.23ms) |
Denso vs-068 | 6 | **27.9%** (0.26ms) | **98.1%** (0.32ms) | **99.8%** (0.37ms) |
NASA Valkyrie arm | 7 | **44.7%** (0.62ms) | **88.5%** (0.92ms) | **99.5%** (0.68ms) |
NASA Robonaut2 arm | 7 | **85.7%** (0.41ms) | **93.5%** (0.5ms) | **99.5%** (0.46ms) |
NASA Robonaut2 grasping 'leg' | 7 | **60.9%** (0.59ms) | **86.7%** (0.54ms) | **99.8%** (0.57ms) |
NASA Robonaut2 'leg' + waist + arm | 15 | **97.6%** (0.78ms) | **97.7%** (0.78ms) | **99.9%** (0.76ms) |
TRACLabs modular arm | 7 | **78.8%** (0.4ms) | **94.6%** (0.42ms) | **100%** (0.38ms) |
Fanuc M-430iA/2F | 5 | **21.1%** (0.19ms) | **88.2%** (0.41ms) | **99.4%** (0.53ms) |
UR5 | 6 | **35.7%** (0.24ms) | **88.7%** (0.24ms) | **99.8%** (0.37ms) |
UR10 | 6 | **36.12%** (0.26ms) | **88.1%** (0.25ms) | **99.7%** (0.39ms) |

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
