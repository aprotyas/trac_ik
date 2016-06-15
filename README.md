The ROS packages in this repository were created to provide an alternative
Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.
Specifically, KDL's convergence algorithms are based on Newton's method, which
does not work well in the presence of joint limits --- common for many robotic
platforms.  TRAC-IK concurrently runs two IK implementations.  One is a simple
extension to KDL's Newton-based convergence algorithm that detects and
mitigates local minima due to joint limits by random jumps.  The second is an
SQP (Sequential Quadratic Programming) nonlinear optimization approach which
uses quasi-Newton methods that better handle joint limits.  By default, the IK
search returns immediately when either of these algorithms converges to an
answer.  Secondary constraints of distance and manipulability are also provided 
in order to receive back the "best" IK solution.

###This repo contains 4 ROS packages:###

- trac\_ik is a metapackage with build and complete [Changelog](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik/CHANGELOG.rst) info.

- trac\_ik\_examples contains examples on how to use the standalone TRAC-IK library.

- [trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib), the TRAC-IK kinematics code,
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md. (Note prior to v1.1.2, the plugin was not thread safe.)

###As of v1.4.3, this package is part of the ROS Indigo/Jade binaries: `sudo apt-get install ros-jade-trac-ik`

###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html) (reported results are from v1.0.0 of TRAC-IK, see below for newer results).

###Some sample results are below: 

_Orocos' **KDL**_ (inverse Jacobian w/ joint limits), _**KDL-RR**_ (our fixes to KDL joint limit handling), and _**TRAC-IK**_ (our concurrent inverse Jacobian and non-linear optimization solver; Speed mode) are compared below.

IK success and average speed (for successful solves) as of TRAC-IK tag v1.4.1.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.

**Note on timings**: The timings provided include both successful and unsuccessful runs.  When an IK solution is not found, the numerical IK solver implementations will run for the full timeout requested, searching for an answer; thus for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times are skewed towards the user requested timeout value (here 5 ms).  

Chain | DOFs | Orocos' _KDL_ solve rate | Orocos' _KDL_ Avg Time | _KDL-RR_ solve rate | _KDL-RR_ Avg Time | _TRAC-IK_ solve rate | _TRAC-IK_ Avg Time
- | - | - | - | - | - | - | -
Atlas 2013 arm | 6 | **75.54%** | 1.35ms | **97.13%** | 0.39ms | **99.97%** | 0.33ms
Atlas 2015 arm | 7 | **75.71%** | 1.50ms | **93.13%** | 0.81ms | **99.18%** | 0.48ms
Baxter arm | 7 | **61.07%** | 2.21ms | **89.52%** | 1.02ms | **99.17%** | 0.60ms
Denso VS-068 | 6 | **27.92%** | 3.69ms | **98.13%** | 0.42ms | **99.78%** | 0.38ms
Fanuc M-430iA/2F | 5 | **21.07%** | 3.99ms | **88.34%** | 0.92ms | **99.16%** | 0.58ms
Fetch arm | 7 | **92.49%** | 0.73ms | **93.82%** | 0.72ms | **99.96%** | 0.44ms
Jaco2 | 6 | **26.23%** | 3.79ms | **97.66%** | 0.58ms | **99.51%** | 0.58ms
KUKA LBR iiwa 14 R820 | 7 | **37.71%** | 3.37ms | **94.02%** | 0.73ms | **99.63%** | 0.56ms
KUKA LWR 4+ | 7 | **67.80%** | 1.88ms | **95.40%** | 0.62ms | **99.95%** | 0.38ms
PR2 arm | 7 | **83.14%** | 1.37ms | **86.96%** | 1.27ms | **99.84%** | 0.59ms
NASA Robonaut2 'grasping leg' | 7 | **61.27%** | 2.29ms | **87.57%** | 1.10ms | **99.31%** | 0.67ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.99%** | 0.80ms | **98.00%** | 0.84ms | **99.86%** | 0.79ms
NASA Robonaut2 arm | 7 | **86.28%** | 1.02ms | **94.26%** | 0.73ms | **99.25%** | 0.50ms
NASA Robosimian arm | 7 | **61.74%** | 2.44ms | **99.87%** | 0.36ms | **99.93%** | 0.44ms
TRACLabs modular arm | 7 | **79.11%** | 1.35ms | **95.12%** | 0.63ms | **99.80%** | 0.53ms
UR10 | 6 | **36.16%** | 3.29ms | **88.05%** | 0.82ms | **99.47%** | 0.49ms
UR5 | 6 | **35.88%** | 3.30ms | **88.69%** | 0.78ms | **99.55%** | 0.42ms
NASA Valkyrie arm | 7 | **45.18%** | 3.01ms | **90.05%** | 1.29ms | **99.63%** | 0.61ms

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
