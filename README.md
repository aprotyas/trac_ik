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

_Orocos' **KDL**_ (inverse Jacobian w/ joint limits), _KDL-RR_ (our fixes to KDL joint limit handling), and _TRAC-IK_ (our concurrent inverse Jacobian and non-linear optimization solver) are compared below.

IK success and average speed (for successful solves) as of TRAC-IK tag v1.1.0.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.

**Note on timings**: The first timings listed below are only for _SUCCESSFUL_ solves, and are meant to illustrate that TRAC-IK does not add significant overhead over KDL in the _WORST CASE_.  The second timings provided include both successful _AND UNSUCCESSFUL_ runs.  When an IK solution is not found, the numerical IK solver implementations will run for the full timeout requested, searching for an answer; thus for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times are skewed towards the timeout value (here 5 ms).  

[comment]: <> **Note on SSE movement**: The SSE movement values show the average Sum-of-Squared Error between the seed joints values (midway between the joint limits) and the joint values for _SUCCESSFUL_ IK solutions.  Lower is generally better in that the found IK solution is closer to the desired (Seed) value when SSE movement values are lower; however, please take into consideration that is an average over successful runs, which may vary drastically across different IK implementations. 

Chain | DOFs | Orocos' KDL solve rate | Orocos' KDL Avg Time for success | Orocos' KDL Avg Time (5ms timeout) | KDL-RR solve rate | KDL-RR Avg Time for success | KDL-RR Avg Time (5ms timeout) | TRAC-IK solve rate | TRAC-IK Avg Time for success | TRAC-IK Avg Time (5ms timeout)
- | - | - | - | - | - | - | - | - | - | -
Atlas 2013 arm | 6 | **75.53%** | 0.16ms | 1.35ms | **97.13%** | 0.26ms | 0.4ms | **99.86%** | 0.29ms | 0.29ms
Atlas 2015 arm | 7 | **75.32%** | 0.4ms | 1.54ms | **92.21%** | 0.52ms | 0.87ms | **99.31%** | 0.47ms | 0.5ms
Baxter arm | 7 | **60.76%** | 0.44ms | 2.23ms | **89.16%** | 0.58ms | 1.06ms | **99.34%** | 0.55ms | 0.58ms
Denso VS-068 | 6 | **27.93%** | 0.27ms | 3.68ms | **98.04%** | 0.33ms | 0.42ms | **99.73%** | 0.38ms | 0.39ms
Fanuc M-430iA/2F | 5 | **21.06%** | 0.18ms | 3.99ms | **88.30%** | 0.38ms | 0.92ms | **99.38%** | 0.51ms | 0.54ms
Fetch arm | 7 | **87.55%** | 0.11ms | 0.72ms | **98.40%** | 0.21ms | 0.28ms | **100.00%** | 0.22ms | 0.22ms
Jaco2 | 6 | **26.23%** | 0.36ms | 3.79ms | **97.69%** | 0.45ms | 0.56ms | **99.67%** | 0.49ms | 0.5ms
KUKA LBR iiwa 14 R820 | 7 | **37.40%** | 0.69ms | 3.39ms | **93.37%** | 0.48ms | 0.78ms | **99.91%** | 0.4ms | 0.41ms
KUKA LWR 4+ | 7 | **67.45%** | 0.41ms | 1.9ms | **94.75%** | 0.43ms | 0.67ms | **99.98%** | 0.34ms | 0.34ms
PR2 arm | 7 | **98.75%** | 0.27ms | 0.33ms | **99.70%** | 0.28ms | 0.29ms | **100.00%** | 0.37ms | 0.37ms
NASA Robonaut2 'grasping leg' | 7 | **60.89%** | 0.59ms | 2.32ms | **86.64%** | 0.54ms | 1.13ms | **99.66%** | 0.58ms | 0.6ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.59%** | 0.78ms | 0.89ms | **97.72%** | 0.78ms | 0.88ms | **99.85%** | 0.8ms | 0.81ms
NASA Robonaut2 arm | 7 | **85.78%** | 0.41ms | 1.07ms | **93.42%** | 0.5ms | 0.8ms | **99.62%** | 0.45ms | 0.47ms
NASA Robosimian arm | 7 | **61.40%** | 0.85ms | 2.46ms | **99.87%** | 0.34ms | 0.34ms | **99.92%** | 0.41ms | 0.41ms
TRACLabs modular arm | 7 | **78.81%** | 0.4ms | 1.37ms | **94.56%** | 0.41ms | 0.66ms | **99.92%** | 0.43ms | 0.43ms
UR10 | 6 | **36.14%** | 0.25ms | 3.29ms | **88.06%** | 0.24ms | 0.81ms | **99.79%** | 0.41ms | 0.42ms
UR5 | 6 | **35.89%** | 0.24ms | 3.29ms | **88.71%** | 0.24ms | 0.77ms | **99.62%** | 0.39ms | 0.4ms
NASA Valkyrie arm | 7 | **44.76%** | 0.61ms | 3.04ms | **88.61%** | 0.92ms | 1.39ms | **99.54%** | 0.65ms | 0.67ms

Feel free to [email Patrick] (mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
