# trac_ik

ROS 2 port of `trac_ik`, an alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.

This repo contains 5 ROS 2 pacakges:
- [`trac_ik`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik): A "metapackage" for the examples, C++/Python libraries, and the MoveIt! plugin.
- [`trac_ik_examples`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_examples): Contains an example on how to use the standalone TRAC-IK C++ library.
- [`trac_ik_lib`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_lib): The TRAC-IK kinematics code, builds a .so library that can be used as a drop in replacement for KDL's IK functions for KDL chains.
Details for use are in [`trac_ik_lib/README.md`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_lib/README.md).
- [`trac_ik_kinematics_plugin`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_kinematics_plugin): Provides a MoveIt! plugin that can replace the default KDL plugin for MoveIt! with TRAC-IK for use in planning.
Details for use are in [`trac_ik_kinematics_plugin/README.md`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_kinematics_plugin/README.md).
- [`trac_ik_python`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_python): SWIG based Python wrapper to use TRAC-IK.
Details for use are in [`trac_ik_python/README.md`](https://github.com/aprotyas/trac_ik/tree/ros2/trac_ik_python/README.md).

Link to the original repository: https://bitbucket.org/traclabs/trac_ik/src/master/
