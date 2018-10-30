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
ABB Yumi 'single arm' | 7 | **76.88%** | 1.50ms | **90.84%** | 0.94ms | **99.08%** | 0.56ms
ABB IRB120 | 6 | **39.41%** | 3.10ms | **98.32%** | 0.41ms | **99.91%** | 0.33ms
Atlas 2013 arm | 6 | **75.54%** | 1.35ms | **97.14%** | 0.39ms | **99.91%** | 0.32ms
Atlas 2015 arm | 7 | **75.63%** | 1.51ms | **93.25%** | 0.81ms | **99.44%** | 0.42ms
Baxter arm | 7 | **60.98%** | 2.22ms | **89.64%** | 1.03ms | **99.44%** | 0.50ms
Denso VS-068 | 6 | **27.92%** | 3.69ms | **98.11%** | 0.42ms | **99.89%** | 0.34ms
Fanuc M-430iA/2F | 5 | **21.07%** | 3.99ms | **88.33%** | 0.92ms | **99.70%** | 0.46ms
Fetch arm | 7 | **92.40%** | 0.74ms | **93.69%** | 0.73ms | **99.98%** | 0.32ms
Franka Emika Panda | 7 | **61.52%** | 2.17ms | **92.22%** | 0.92ms | **99.49%** | 0.50ms
Jaco2 | 6 | **26.22%** | 3.79ms | **97.65%** | 0.58ms | **99.61%** | 0.50ms
KUKA LBR iiwa 14 R820 | 7 | **37.63%** | 3.38ms | **93.73%** | 0.76ms | **99.84%** | 0.35ms
KUKA LWR 4+ | 7 | **67.72%** | 1.89ms | **95.33%** | 0.64ms | **99.99%** | 0.29ms
Motoman MH180 | 6 | **68.44%** | 1.67ms | **99.34%** | 0.27ms | **99.99%** | 0.24ms
Motoman CSDA10F 'torso/1-arm' | 8 | **52.47%** | 2.85ms | **95.16%** | 0.72ms | **99.81%** | 0.44ms
PR2 arm | 7 | **82.92%** | 1.39ms | **86.52%** | 1.30ms | **99.97%** | 0.41ms
NASA Robonaut2 'leg' + waist + arm | 15 | **97.78%** | 0.86ms | **97.94%** | 0.85ms | **99.88%** | 0.72ms
NASA Robonaut2 arm | 7 | **86.11%** | 1.05ms | **93.95%** | 0.76ms | **99.56%** | 0.41ms
NASA Robonaut2 'grasping leg' | 7 | **61.10%** | 2.30ms | **87.36%** | 1.12ms | **99.80%** | 0.51ms
NASA Robosimian arm | 7 | **61.64%** | 2.46ms | **99.87%** | 0.37ms | **99.95%** | 0.44ms
NASA Valkyrie arm | 7 | **45.05%** | 3.02ms | **89.58%** | 1.32ms | **99.76%** | 0.48ms
Schunk LWA4D | 7 | **68.22%** | 1.85ms | **96.54%** | 0.53ms | **99.96%** | 0.34ms
TRACLabs modular arm | 7 | **78.99%** | 1.36ms | **94.89%** | 0.65ms | **99.96%** | 0.37ms
Universal UR3 | 6 | **34.12%** | 3.41ms | **88.79%** | 0.84ms | **99.34%** | 0.49ms
UR5 | 6 | **32.23%** | 3.49ms | **88.62%** | 0.79ms | **99.64%** | 0.36ms
UR10 | 6 | **30.67%** | 3.56ms | **88.38%** | 0.80ms | **99.68%** | 0.42ms

Feel free to [email Patrick](mailto:pbeeson@traclabs.com) if there is a robot chain that you would like to see added above.
