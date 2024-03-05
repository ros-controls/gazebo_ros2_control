^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros2_control_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2024-01-24)
------------------

0.7.0 (2024-01-04)
------------------
* replace Twist with TwistStamped (`#249 <https://github.com/ros-controls/gazebo_ros2_control/issues/249>`_)
* Rename cartpole (`#252 <https://github.com/ros-controls/gazebo_ros2_control/issues/252>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Replace double quotes with single ones (`#243 <https://github.com/ros-controls/gazebo_ros2_control/issues/243>`_)
* Cleanup controller config (`#232 <https://github.com/ros-controls/gazebo_ros2_control/issues/232>`_)
  * Remove wrong yaml entries
  * Rename effort_controller
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich

0.6.2 (2023-08-23)
------------------
* Set the C++ version to 17 (`#221 <https://github.com/ros-controls/gazebo_ros2_control/issues/221>`_)
* Update diff_drive_controller.yaml (`#224 <https://github.com/ros-controls/gazebo_ros2_control/issues/224>`_)
  The wrong base frame is set. The name of the link in the URDF is chassis.
* Contributors: Alejandro Hernández Cordero, David V. Lu!!

0.6.1 (2023-06-09)
------------------
* Add pre-commit and CI-format (`#206 <https://github.com/ros-controls/gazebo_ros2_control/issues/206>`_)
  * Add pre-commit and ci-format
* Contributors: Christoph Fröhlich

0.6.0 (2023-05-23)
------------------
* Clean shutdown position example (`#196 <https://github.com/ros-controls/gazebo_ros2_control/issues/196>`_)
* Remove publish_rate parameter (`#179 <https://github.com/ros-controls/gazebo_ros2_control/issues/179>`_)
* Contributors: Alejandro Hernández Cordero, Tony Najjar

0.5.1 (2023-02-07)
------------------

0.5.0 (2023-01-06)
------------------
* Add tricycle controller demo (`#145 <https://github.com/ros-controls/gazebo_ros2_control/issues/145>`_)
* Contributors: Tony Najjar

0.4.0 (2022-08-09)
------------------
* fix demo launch
* Fix setting initial values if command interfaces are not defined. (`#110 <https://github.com/ros-simulation/gazebo_ros2_control/issues/110>`_)
* Contributors: Bence Magyar, Denis Štogl, Maciej Bednarczyk

0.3.1 (2022-07-05)
------------------
* Fixed CMake source file extension (`#140 <https://github.com/ros-simulation/gazebo_ros2_control/issues/140>`_)
* Adding simulation time parameter for the controller manager (`#138 <https://github.com/ros-simulation/gazebo_ros2_control/issues/138>`_)
  Adding the simulation parameter so that the controller manager uses the simulation time instead of the ROS time.  The '/odom' and corresponding tf will only be published if this parameter is set to true.
* Adjust repo URL (`#134 <https://github.com/ros-simulation/gazebo_ros2_control/issues/134>`_)
* Changed launch variable name (`#130 <https://github.com/ros-simulation/gazebo_ros2_control/issues/130>`_)
* Contributors: Alejandro Hernández Cordero, Bence Magyar, Eslam Salah, Jakub "Deli" Delicat

0.3.0 (2022-05-27)
------------------
* [Forward port main] Added diff drive example (`#113 <https://github.com/ros-simulation/gazebo_ros2_control/issues/113>`_) (`#129 <https://github.com/ros-simulation/gazebo_ros2_control/issues/129>`_)
* Merge pull request `#120 <https://github.com/ros-simulation/gazebo_ros2_control/issues/120>`_ from ros-simulation/ahcorde/main/117
  Adapted to Humble
* make linters happy
* Update to Humble API
* Support for mimic joints and example with gripper. (`#107 <https://github.com/ros-simulation/gazebo_ros2_control/issues/107>`_)
* Contributors: Alejandro Hernández Cordero, Denis Štogl, ahcorde

0.0.8 (2022-01-28)
------------------
* Enable setting default position of the simulated robot using ros2_control URDF tag. (`#100 <https://github.com/ros-simulation/gazebo_ros2_control//issues/100>`_)
* Contributors: Denis Štogl

0.0.7 (2021-12-03)
------------------

0.0.6 (2021-11-18)
------------------
* Fix ros2_control resource manager in galatic (`#96 <https://github.com/ros-simulation/gazebo_ros2_control//issues/96>`_)
* Contributors: Alejandro Hernández Cordero

0.0.4 (2021-10-26)
------------------
* fix maintainer email (`#92 <https://github.com/ros-simulation/gazebo_ros2_control//issues/92>`_)
* Galactic: Pass time and period to update function (`#88 <https://github.com/ros-simulation/gazebo_ros2_control//issues/88>`_)
* Update severity of msgs to proper level (`#91 <https://github.com/ros-simulation/gazebo_ros2_control//issues/91>`_)
* Add Imu and FT state interfaces (`#65 <https://github.com/ros-simulation/gazebo_ros2_control//issues/65>`_)
  Co-authored-by: Jordan Palacios <jordan.palacios@pal-robotics.com>
* Contributors: Alejandro Hernández Cordero, Andy McEvoy, Bence Magyar, Victor Lopez

0.0.3 (2021-06-16)
------------------
* Update code with recent change in ros2_control (`#81 <https://github.com/ros-simulation/gazebo_ros2_control/issues/81>`_)
* Adding ros2_control dependency to demos (`#74 <https://github.com/ros-simulation/gazebo_ros2_control/issues/74>`_) (`#76 <https://github.com/ros-simulation/gazebo_ros2_control/issues/76>`_)
* Contributors: Alejandro Hernández Cordero, Ron Marrero

0.0.2 (2021-04-19)
------------------
* Remove Unnecessary parameter in demo (`#68 <https://github.com/ros-simulation/gazebo_ros2_control//issues/68>`_)
* Add effort_controller exec_depend on demos (`#69 <https://github.com/ros-simulation/gazebo_ros2_control//issues/69>`_)
* add ros parameters file to node context (`#60 <https://github.com/ros-simulation/gazebo_ros2_control//issues/60>`_)
  Co-authored-by: ahcorde <ahcorde@gmail.com>
* add ros2_controllers as exec dependency (`#56 <https://github.com/ros-simulation/gazebo_ros2_control//issues/56>`_)
  fixes `#49 <https://github.com/ros-simulation/gazebo_ros2_control//issues/49>`_
* Contributors: Alejandro Hernández Cordero, Karsten Knese

0.0.1 (2021-02-05)
------------------
* Updated with ros2-control Foxy API (`#44 <https://github.com/ros-simulation/gazebo_ros2_control/issues/44>`_)
  Co-authored-by: Karsten Knese <Karsten1987@users.noreply.github.com>
* Updated with recent ros2_control changes (`#34 <https://github.com/ros-simulation/gazebo_ros2_control/issues/34>`_)
* Added initial demos in gazebo_ros2_control_demos (`#2 <https://github.com/ros-simulation/gazebo_ros2_control/issues/2>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Karsten Knese, Bence Magyar
