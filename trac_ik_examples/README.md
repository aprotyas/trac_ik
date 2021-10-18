## trac_ik_examples

This package provides examples programs to use the standalone TRAC-IK solver and related code.

Currently, there only exists an `ik_tests` program that compares KDL's Pseudoinverse Jacobian IK solver with TRAC-IK.
The [`pr2_arm.launch.py`](./pr2_arm.launch.py) files runs this test on the default PR2 robot's 7-DOF right arm chain.
