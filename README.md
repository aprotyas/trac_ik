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

###This repo contains 5 ROS packages:###

- trac\_ik is a metapackage with build and complete [Changelog](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik/CHANGELOG.rst) info.

- trac\_ik\_examples contains examples on how to use the standalone TRAC-IK library.

- [trac\_ik\_lib](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_lib), the TRAC-IK kinematics code,
builds a .so library that can be used as a drop in replacement for KDL's IK
functions for KDL chains. Details for use are in trac\_ik\_lib/README.md.

- [trac\_ik\_kinematics\_plugin](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin) builds a [MoveIt! plugin](http://moveit.ros.org/documentation/concepts/#kinematics) that can
replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in trac\_ik\_kinematics\_plugin/README.md. (Note prior to v1.1.2, the plugin was not thread safe.)

- [trac\_ik\_python](https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_python), SWIG based python wrapper to use TRAC-IK. Details for use are in trac\_ik\_python/README.md.


###As of v1.4.5, this package is part of the ROS Kinetic binaries: `sudo apt-get install ros-kinetic-trac-ik` (or indigo or jade).  Starting with v1.4.8, this has been released for ROS Lunar as well. Melodic packages have been released with 1.5.0.


###A detailed writeup on TRAC-IK can be found here:###

[Humanoids-2015](https://personal.traclabs.com/~pbeeson/publications/b2hd-Beeson-humanoids-15.html) (reported results are from v1.0.0 of TRAC-IK, see below for newer results).

###Some sample results are below: 

_Orocos' **KDL**_ (inverse Jacobian w/ joint limits), _**KDL-RR**_ (our fixes to KDL joint limit handling), and _**TRAC-IK**_ (our concurrent inverse Jacobian and non-linear optimization solver; Speed mode) are compared below.

IK success and average speed as of TRAC-IK tag v1.4.6.  All results are from 10,000 randomly generated, reachable joint configurations.  Full 3D pose IK was requested at 1e-5 Cartesian error for x,y,z,roll,pitch,yaw with a maximum solve time of 5 ms.  All IK queries are seeded from the chain's "nominal" pose midway between joint limits.

**Note on success**: Neither KDL nor TRAC-IK uses any mesh information to determine if _valid_ IK solutions result in self-collisions.  IK solutions deal with link distances and joint ranges, and remain agnostic about self-collisions due to volumes.  Expected future enhancements to TRAC-IK that search for multiple solutions may also include the ability to throw out solutions that result in self collisions (provided the URDF has valid geometry information); however, this is currently not the behaviour of any generic IK solver examined to date.

**Note on timings**: The timings provided include both successful and unsuccessful runs.  When an IK solution is not found, the numerical IK solver implementations will run for the full timeout requested, searching for an answer; thus for robot chains where KDL fails much of the time (e.g., Jaco-2), the KDL times are skewed towards the user requested timeout value (here 5 ms).  

Chain | DOFs | Orocos' _KDL_ solve rate | Orocos' _KDL_ Avg Time | _KDL-RR_ solve rate | _KDL-RR_ Avg Time | _TRAC-IK_ solve rate | _TRAC-IK_ Avg Time
- | - | - | - | - | - | - | -
ABB IRB120 | 6 | **39.41%** | 3.09ms | **98.48%** | 0.35ms | **99.91%** | 0.26ms
ABB Yumi 'single arm' | 7 | **77.28%** | 1.44ms | **91.37%** | 0.86ms | **99.60%** | 0.42ms
Atlas 2013 arm | 6 | **75.54%** | 1.35ms | **97.11%** | 0.39ms | **99.96%** | 0.24ms
Atlas 2015 arm | 7 | **75.67%** | 1.50ms | **93.24%** | 0.81ms | **99.56%** | 0.39ms
Baxter arm | 7 | **61.07%** | 2.21ms | **89.62%** | 1.02ms | **99.62%** | 0.43ms
Denso VS-068 | 6 | **27.92%** | 3.68ms | **98.18%** | 0.41ms | **99.81%** | 0.30ms
Fanuc M-430iA/2F | 5 | **21.07%** | 3.99ms | **88.37%** | 0.91ms | **99.69%** | 0.42ms
Fetch arm | 7 | **92.46%** | 0.73ms | **93.80%** | 0.72ms | **99.99%** | 0.26ms
Jaco2 | 6 | **26.23%** | 3.79ms | **97.65%** | 0.58ms | **99.78%** | 0.41ms
KUKA LBR iiwa 14 R820 | 7 | **37.64%** | 3.38ms | **93.70%** | 0.77ms | **99.84%** | 0.31ms
KUKA LWR 4+ | 7 | **67.81%** | 1.88ms | **95.35%** | 0.62ms | **99.99%** | 0.27ms
Motoman CSDA10F 'torso/1-arm' | 8 | **53.15%** | 2.78ms | **95.72%** | 0.65ms | **99.90%** | 0.32ms
Motoman MH180 | 6 | **68.46%** | 1.66ms | **99.38%** | 0.24ms | **100.00%** | 0.18ms
Franka Emika Panda | 7 | **61.77%** | 2.14ms | **92.67%** | 0.85ms | **99.70%** | 0.38ms
PR2 arm | 7 | **83.18%** | 1.37ms | **86.87%** | 1.28ms | **99.95%** | 0.38ms
NASA Robonaut2 'grasping leg' | 7 | **61.04%** | 2.30ms | **87.25%** | 1.13ms | **99.85%** | 0.46ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.85%** | 0.84ms | **98.01%** | 0.83ms | **99.83%** | 0.65ms
NASA Robonaut2 arm | 7 | **86.18%** | 1.04ms | **94.19%** | 0.74ms | **99.76%** | 0.33ms
NASA Robosimian arm | 7 | **61.69%** | 2.45ms | **99.87%** | 0.36ms | **99.94%** | 0.38ms
TRACLabs modular arm | 7 | **79.05%** | 1.36ms | **95.06%** | 0.63ms | **99.95%** | 0.33ms
NASA Valkyrie arm | 7 | **45.14%** | 3.01ms | **89.86%** | 1.31ms | **99.81%** | 0.46ms
Schunk LWA4D | 7 | **68.40%** | 1.81ms | **97.02%** | 0.47ms | **100.00%** | 0.23ms

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
