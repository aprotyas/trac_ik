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

###This repo contains 4 ROS packages:###

- trac\_ik is a metapackage with build and [Changelog](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik/CHANGELOG.rst) info.

- trac\_ik\_examples contains examples on how to use the standalone TRAC-IK library.

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

IK success and average speed (for successful solves) as of TRAC-IK tag v1.3.8.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.

**Note on timings**: The timings provided include both successful and unsuccessful runs.  When an IK solution is not found, the numerical IK solver implementations will run for the full timeout requested, searching for an answer; thus for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times are skewed towards the user requested timeout value (here 5 ms).  

Chain | DOFs | Orocos' _KDL_ solve rate | Orocos' _KDL_ Avg Time | _KDL-RR_ solve rate | _KDL-RR_ Avg Time | _TRAC-IK_ solve rate | _TRAC-IK_ Avg Time
- | - | - | - | - | - | - | -
Atlas 2013 arm | 6 | **75.54%** | 1.35ms | **97.14%** | 0.39ms | **99.93%** | 0.34ms
Atlas 2015 arm | 7 | **75.73%** | 1.50ms | **93.24%** | 0.80ms | **99.05%** | 0.51ms
Baxter arm | 7 | **61.07%** | 2.21ms | **89.75%** | 1.02ms | **99.18%** | 0.59ms
Denso VS-068 | 6 | **27.92%** | 3.69ms | **98.16%** | 0.41ms | **99.66%** | 0.44ms
Fanuc M-430iA/2F | 5 | **21.07%** | 3.99ms | **88.34%** | 0.92ms | **99.10%** | 0.58ms
Fetch arm | 7 | **92.49%** | 0.73ms | **93.78%** | 0.72ms | **99.95%** | 0.38ms
Jaco2 | 6 | **26.22%** | 3.79ms | **97.65%** | 0.58ms | **99.67%** | 0.52ms
KUKA LBR iiwa 14 R820 | 7 | **37.68%** | 3.37ms | **93.99%** | 0.74ms | **99.94%** | 0.41ms
KUKA LWR 4+ | 7 | **67.80%** | 1.88ms | **95.36%** | 0.62ms | **99.95%** | 0.41ms
PR2 arm | 7 | **74.61%** | 1.59ms | **92.72%** | 0.86ms | **99.82%** | 0.49ms
NASA Robonaut2 'grasping leg' | 7 | **61.29%** | 2.28ms | **87.61%** | 1.10ms | **99.46%** | 0.65ms
NASA Robonaut2 'leg' + waist + arm | 15 | **98.03%** | 0.79ms | **98.16%** | 0.79ms | **99.86%** | 0.78ms
NASA Robonaut2 arm | 7 | **86.35%** | 1.02ms | **94.31%** | 0.73ms | **99.44%** | 0.49ms
NASA Robosimian arm | 7 | **61.73%** | 2.44ms | **99.87%** | 0.35ms | **99.89%** | 0.45ms
TRACLabs modular arm | 7 | **79.12%** | 1.34ms | **95.12%** | 0.63ms | **99.95%** | 0.40ms
UR10 | 6 | **36.16%** | 3.29ms | **88.05%** | 0.82ms | **99.59%** | 0.44ms
UR5 | 6 | **35.88%** | 3.30ms | **88.66%** | 0.78ms | **99.36%** | 0.43ms
NASA Valkyrie arm | 7 | **45.11%** | 3.02ms | **89.72%** | 1.32ms | **99.55%** | 0.64ms

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
