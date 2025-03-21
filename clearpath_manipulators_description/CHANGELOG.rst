^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_manipulators_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2025-01-31)
------------------
* Ewellix Lift (`#136 <https://github.com/clearpathrobotics/clearpath_common/issues/136>`_) (`#153 <https://github.com/clearpathrobotics/clearpath_common/issues/153>`_)
  Ewellix Lift
  - Add lift description
  - Add lifts to generators
* Updated URDF
* Contributors: Luis Camero, luis-camero

2.0.3 (2025-01-21)
------------------

2.0.2 (2025-01-20)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-16)
------------------
* Add PTZ sim support (`#125 <https://github.com/clearpathrobotics/clearpath_common/issues/125>`_)
  * Now that axis_camera is released via OSRF, depend on the official package, remove duplicate meshes
  * Rename Gazebo plugins for Jazzy compatibility
  * Modify Axis camera URDFs to using the axis_camera meshes. This lets us control the gazebo topics. Fix the GZ topic names. Camera data is now visible in the simulation
  * Add joint controllers for the pan & tilt actuators. This provides velocity control over the simulated camera
* Rename ign\_ -> gz\_ for gazebo dependencies, comment-out missing jazzy dependencies (for now)
* Contributors: Chris Iverach-Brereton, Tony Baltovski, luis-camero

1.0.0 (2024-11-25)
------------------
* Add UR arm (`#110 <https://github.com/clearpathrobotics/clearpath_common/issues/110>`_)
* Contributors: luis-camero

0.3.4 (2024-10-08)
------------------

0.3.3 (2024-10-04)
------------------

0.3.2 (2024-09-29)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Fixed package version on new packages to match old packages.
* Changes.
* Add meshes and URDF for robotiq 2f 140
* Fixed all license headers
* Added dependencies to manipulators description
* Added Robotiq grippers for Kinova
* Added all joints to the Kinova gripper SRDF
* Removed unecessary parameter
* Gripper joint set through arm URD
* Added another variable to add namespace for MoveItt
* Added argument to toggle manipulator controllers
* Manipulator description to match platform description launch
* Added all descriptions for kinova manipulators
* Contributors: Luis Camero, Tony Baltovski

* Add meshes and URDF for robotiq 2f 140
* Fixed all license headers
* Added dependencies to manipulators description
* Added Robotiq grippers for Kinova
* Added all joints to the Kinova gripper SRDF
* Removed unecessary parameter
* Gripper joint set through arm URD
* Added another variable to add namespace for MoveItt
* Added argument to toggle manipulator controllers
* Manipulator description to match platform description launch
* Added all descriptions for kinova manipulators
* Contributors: Luis Camero

0.2.11 (2024-08-08)
-------------------

0.2.10 (2024-07-25)
-------------------

0.2.9 (2024-05-28)
------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-08)
------------------

0.2.6 (2024-01-18)
------------------

0.2.5 (2024-01-15)
------------------

0.2.4 (2024-01-11)
------------------

0.2.3 (2024-01-08)
------------------

0.2.2 (2024-01-04)
------------------

0.2.1 (2023-12-21)
------------------

0.2.0 (2023-12-08)
------------------

0.1.3 (2023-11-03)
------------------

0.1.2 (2023-10-02)
------------------

0.1.1 (2023-08-25)
------------------

0.1.0 (2023-08-17)
------------------

0.0.9 (2023-07-31)
------------------

0.0.8 (2023-07-24)
------------------

0.0.7 (2023-07-19)
------------------

0.0.6 (2023-07-13)
------------------

0.0.5 (2023-07-12)
------------------

0.0.4 (2023-07-07)
------------------

0.0.3 (2023-07-05)
------------------

0.0.2 (2023-07-04)
------------------

0.0.1 (2023-06-21)
------------------
