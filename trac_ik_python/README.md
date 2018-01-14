# trac_ik_python

Python wrapper for TRAC IK library using SWIG to generate the bindings.
Please send any questions to [Sam Pfeiffer](mailto:Sammy.Pfeiffer@student.uts.edu.au)

# Example usage
Upload a robot to the param server:

    roslaunch pr2_description upload_pr2.launch

Now you can get IK's:

```python
#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link")

seed_state = [0.0] * ik_solver.number_of_joints

ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,  # X, Y, Z
                0.0, 0.0, 0.0, 1.0)  # QX, QY, QZ, QW
# Returns:
# (0.537242808640495,
#  0.04673341230604478,
#  -0.053508394352190486,
#  -1.5099959208163785,
#  2.6007509004432596,
#  -1.506431092603137,
#  -3.040949079090651)
```


You can also play with the bounds of the IK call:
```python
#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link")

seed_state = [0.0] * ik_solver.number_of_joints

ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,
                0.0, 0.0, 0.0, 1.0,
                0.01, 0.01, 0.01,  # X, Y, Z bounds
                0.1, 0.1, 0.1)  # Rotation X, Y, Z bounds
                )
# Returns:
# (0.5646018385887146,
#  0.04759637706046231,
#  0.026629718805901908,
#  -1.5106828886580062,
#  2.5541685245726535,
#  -1.4663448384900402,
#  -3.104163452483634)
```

The coordinate frame of the given poses must be in the base frame, you can check the links/joints being used:
```python
#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link")

ik_solver.base_link
# 'torso_lift_link'

ik_solver.tip_link
# 'r_wrist_roll_link'

ik_solver.joint_names
# ('r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint')

ik_solver.link_names
# ('r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link', 'r_upper_arm_link', 'r_elbow_flex_link', 'r_forearm_roll_link', 'r_forearm_link', 'r_wrist_flex_link', 'r_wrist_roll_link')
```

You can also initialize the IK from a string containing the URDF (by default it takes it from the `/robot_description` parameter in the param server):
```python
#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

# Get your URDF from somewhere
urdf_str = rospy.get_param('/robot_description')

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link",
               urdf_string=urdf_str)
```


You can also check and modify the joint limits:
```python
#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link")

lower_bound, upper_bound = ik_solver.get_joint_limits()
# lower_bound: (-2.1353981494903564, -0.35359999537467957, -3.75, -2.121299982070923, -3.4028234663852886e+38, -2.0, -3.4028234663852886e+38) 
# upper_bound: (0.5646018385887146, 1.2963000535964966, 0.6499999761581421, -0.15000000596046448, 3.4028234663852886e+38, -0.10000000149011612, 3.4028234663852886e+38)
ik_solver.set_joint_limits([0.0]* ik_solver.number_of_joints, upper_bound)
```

# Extra notes
Given that the Python wrapper is made using [SWIG](http://www.swig.org/) it could be extended to other languages.

You'll get extra output when instantiating the class that comes from C++:
```
[ WARN] [1486091331.089974163]: The root link base_footprint has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
```

When finishing the program you'll also get this error (which [is OK apparently](https://github.com/ros-planning/moveit/issues/331)):
```
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```

And as a final note I didn't simplify even more letting Pose/PoseStamped messages as input, or transforming using TF coordinate frames, adding forward kinematics, etc. to keep it simple.
