Change history
==============

1.2.0 (2015-12-04)
------------------

 * Extended TRAC-IK to both run in two ways: 1) the old mode of first IK solution found causes TRAC-IK to return immediately, versus 2) the new mode where TRAC-IK runs for the full requested timeout duration, then sorts all solutions according to distance from the seed and returns the minimum.
 * Made MoveIt! support this new IK run mode if the user desires.
 * Improved timing info to use a higher solution clock.
 * Fixed TRAC-IK's abort/reset of KDL-RR and NLOpt-IK to catch race conditions.

1.1.2 (2015-12-3)
------------------

 * Fixed issue where clamping a seed to be within the joint limits might still have values outside the limits.
 * Fixed issue where MoveIt! plugin was not thread safe.
 * Fixed an issue in MoveIt! plugin where error_code passed in uninitialized to SUCCESS could cause IK to say it failed when it did not.


1.1.1 (2015-11-19)
------------------

 * Prepared code to have auto test suite run to generate data in main README.md.

1.1.0 (2015-11-12)
------------------

 * Improvements to KDL-RR that better handle joint limits on rotational joints that can turn +- PI.
 * Fixed bug where continuous joints could cause problems.
 * Made NLOpt modes enums instead of integer parameters.

1.0.0 (2015-11-10)
------------------

 * Initial checkin of TRAC-IK as of Humanoids 2015 submission.  Pulled from private repo.
 * Made trac_ik packages conform to rosdep standards.
