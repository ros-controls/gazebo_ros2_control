^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros2_control_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.11 (2025-05-18)
-------------------
* Fix namespacing for multiple instances of gazebo_ros2_control plugin (`#181 <https://github.com/ros-controls/gazebo_ros2_control/issues/181>`_) (`#398 <https://github.com/ros-controls/gazebo_ros2_control/issues/398>`_)
* Contributors: Ben Holden, Alejandro Hernández Cordero, Christoph Froehlich

0.4.10 (2024-09-17)
-------------------
* Add support for getting PID parameters from loaded parameters (`#374 <https://github.com/ros-controls/gazebo_ros2_control//issues/374>`_) (`#375 <https://github.com/ros-controls/gazebo_ros2_control//issues/375>`_)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 6a4cc84344ed1a86807dc77f23f199598a205296)
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* Add missing dependency (`#350 <https://github.com/ros-controls/gazebo_ros2_control//issues/350>`_) (`#351 <https://github.com/ros-controls/gazebo_ros2_control//issues/351>`_)
  (cherry picked from commit 06da0b04fefc0fa8f0bb8d4f57425d0abb261654)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

0.4.9 (2024-07-02)
------------------

0.4.8 (2024-05-14)
------------------
* Change initial pose of pendulum (`#313 <https://github.com/ros-controls/gazebo_ros2_control//issues/313>`_) (`#315 <https://github.com/ros-controls/gazebo_ros2_control//issues/315>`_)
  (cherry picked from commit 40ee42da16af9f1bc78886dbaec8082fd3fdea26)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Add PID controller to control joint using effort (`#294 <https://github.com/ros-controls/gazebo_ros2_control//issues/294>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Add an example with a passive joint (backport `#172 <https://github.com/ros-controls/gazebo_ros2_control//issues/172>`_) (`#306 <https://github.com/ros-controls/gazebo_ros2_control//issues/306>`_)
  * Add an example with a passive joint (`#172 <https://github.com/ros-controls/gazebo_ros2_control//issues/172>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 7d5ec5dbad710d628bc14a82195c196f088621b8)
  # Conflicts:
  #	doc/index.rst
  * Fixed docs
  ---------
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: chameau5050, mergify[bot]

0.4.7 (2024-03-21)
------------------
* Cleanup of demos (`#290 <https://github.com/ros-controls/gazebo_ros2_control/issues/290>`_) (`#292 <https://github.com/ros-controls/gazebo_ros2_control/issues/292>`_)
  (cherry picked from commit 03b853b5337f6b8e9b8d4c0c8a3d814d7f34a97c)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

0.4.6 (2024-01-24)
------------------

0.4.5 (2024-01-04)
------------------
* Rename cartpole (`#252 <https://github.com/ros-controls/gazebo_ros2_control/issues/252>`_) (`#254 <https://github.com/ros-controls/gazebo_ros2_control/issues/254>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit b39074a4a1adf8a9319a6d4378ac26e2aa9e298a)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Replace double quotes with single ones (`#243 <https://github.com/ros-controls/gazebo_ros2_control/issues/243>`_) (`#244 <https://github.com/ros-controls/gazebo_ros2_control/issues/244>`_)
  (cherry picked from commit f991075a672a26d42c49504e07f8dbb46dfcbb4a)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Cleanup controller config (`#232 <https://github.com/ros-controls/gazebo_ros2_control/issues/232>`_) (`#233 <https://github.com/ros-controls/gazebo_ros2_control/issues/233>`_)
  * Remove wrong yaml entries
  * Rename effort_controller
  (cherry picked from commit 934621ee236dc9b32350b113c6b42a894bfbf092)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

0.4.4 (2023-08-21)
------------------
* Set the C++ version to 17 (`#221 <https://github.com/ros-controls/gazebo_ros2_control/issues/221>`_) (`#228 <https://github.com/ros-controls/gazebo_ros2_control/issues/228>`_)
  (cherry picked from commit 6da415cf82a75e2a5e9f9a41400957ad45b2be84)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Update diff_drive_controller.yaml (`#224 <https://github.com/ros-controls/gazebo_ros2_control/issues/224>`_) (`#225 <https://github.com/ros-controls/gazebo_ros2_control/issues/225>`_)
  The wrong base frame is set. The name of the link in the URDF is chassis.
  (cherry picked from commit c915939bfc13a43b2ab0e30029725f6c8023f3ca)
  Co-authored-by: David V. Lu!! <davidvlu@gmail.com>
* Add pre-commit and CI-format (`#206 <https://github.com/ros-controls/gazebo_ros2_control/issues/206>`_) (`#207 <https://github.com/ros-controls/gazebo_ros2_control/issues/207>`_)
  * Add pre-commit and ci-format
  (cherry picked from commit f2cf686a1a97cefc9b5e3daa115e0c4854ea5707)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, mergify[bot]

0.4.3 (2023-05-23)
------------------
* Clean shutdown position example (`#196 <https://github.com/ros-controls/gazebo_ros2_control/issues/196>`_) (`#199 <https://github.com/ros-controls/gazebo_ros2_control/issues/199>`_)
* Contributors: mergify[bot]

0.4.2 (2023-03-02)
------------------

0.4.1 (2023-02-07)
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
