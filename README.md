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

- trac\_ik is a metapackage with build and [Changelog](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik/CHANGELOG.rst) info.

- [trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib), the TRAC-IK kinematics code,
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md. (Note prior to v1.1.2, the plugin was not thread safe.)

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html) (reported results are from v1.0.0 of TRAC-IK, see below for newer results).

###Some sample results are below: 

_Orocos' **KDL**_ (inverse Jacobian w/ joint limits), _**KDL-RR**_ (our fixes to KDL joint limit handling), and _**TRAC-IK**_ (our concurrent inverse Jacobian and non-linear optimization solver; Speed mode) are compared below.

IK success and average speed (for successful solves) as of TRAC-IK tag v1.3.2.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.

**Note on timings**: The timings provided include both successful and unsuccessful runs.  When an IK solution is not found, the numerical IK solver implementations will run for the full timeout requested, searching for an answer; thus for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times are skewed towards the user requested timeout value (here 5 ms).  

Chain | DOFs | Orocos' _KDL_ solve rate | Orocos' _KDL_ Avg Time | _KDL-RR_ solve rate | _KDL-RR_ Avg Time | _TRAC-IK_ solve rate | _TRAC-IK_ Avg Time
- | - | - | - | - | - | - | -
Atlas 2013 arm | 6 | **75.53%** | 1.35ms | **97.11%** | 0.40ms | **99.79%** | 0.30ms
Atlas 2015 arm | 7 | **75.26%** | 1.55ms | **92.48%** | 0.87ms | **99.05%** | 0.55ms
Baxter arm | 7 | **60.70%** | 2.23ms | **89.24%** | 1.06ms | **99.11%** | 0.60ms
Denso VS-068 | 6 | **27.92%** | 3.69ms | **98.14%** | 0.42ms | **99.68%** | 0.42ms
Fanuc M-430iA/2F | 5 | **21.07%** | 3.99ms | **88.38%** | 0.91ms | **99.41%** | 0.56ms
Fetch arm | 7 | **87.55%** | 0.72ms | **98.53%** | 0.27ms | **99.99%** | 0.23ms
Jaco2 | 6 | **26.23%** | 3.79ms | **97.67%** | 0.57ms | **99.73%** | 0.52ms
KUKA LBR iiwa 14 R820 | 7 | **37.38%** | 3.40ms | **93.56%** | 0.78ms | **99.92%** | 0.40ms
KUKA LWR 4+ | 7 | **67.43%** | 1.91ms | **94.91%** | 0.66ms | **99.97%** | 0.35ms
PR2 arm | 7 | **98.75%** | 0.34ms | **99.71%** | 0.30ms | **100.00%** | 0.35ms
NASA Robonaut2 'grasping leg' | 7 | **60.82%** | 2.32ms | **86.99%** | 1.14ms | **99.32%** | 0.71ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.43%** | 0.92ms | **97.53%** | 0.91ms | **99.80%** | 0.86ms
NASA Robonaut2 arm | 7 | **85.58%** | 1.09ms | **93.37%** | 0.80ms | **99.50%** | 0.48ms
NASA Robosimian arm | 7 | **61.33%** | 2.47ms | **99.87%** | 0.35ms | **99.94%** | 0.45ms
TRACLabs modular arm | 7 | **78.64%** | 1.39ms | **94.41%** | 0.68ms | **99.83%** | 0.48ms
UR10 | 6 | **36.14%** | 3.29ms | **88.06%** | 0.81ms | **99.89%** | 0.38ms
UR5 | 6 | **35.88%** | 3.30ms | **88.69%** | 0.78ms | **99.56%** | 0.49ms
NASA Valkyrie arm | 7 | **44.66%** | 3.05ms | **88.66%** | 1.39ms | **99.65%** | 0.61ms

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
