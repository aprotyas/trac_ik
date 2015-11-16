This package provides is a MoveIt! kinematics plugin that replaces the KDL IK
solver with the TRAC-IK solver.  To use:

- Add this and trac_ik_lib package to your catkin workspace.
- Find the MoveIt! [kinematics.yaml](http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/kinematics/src/doc/kinematics_configuration.html) file created for your robot.
- Replace
```kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin```
(or similar) with
```kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin```
- Set parameters as desired.   
   -- Unlike KDL, TRAC-IK solver already restarts when it gets stuck so the
   - _kinematics\_solver\_attempts_ parameter is unneeded.
   - _kinematics\_solver\_timeout_ and _position\_only\_ik_ **ARE** supported.
   - _kinematics\_solver\_search\_resolution_ is not applicable here.  
   - Note: The Cartesian error distance used to determine a valid solution is _1e-5_, as that is what is hard-coded into MoveIt's KDL plugin.

