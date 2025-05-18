^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add `hold_joints` parameter (backport `#251 <https://github.com/ros-controls/gazebo_ros2_control//issues/251>`_) (`#353 <https://github.com/ros-controls/gazebo_ros2_control//issues/353>`_)
  Co-authored-by: Christoph Froehlich <christoph.froehlich@ait.ac.at>
* Contributors: mergify[bot]

0.4.9 (2024-07-02)
------------------
* Initialize antiwindup variable properly (`#326 <https://github.com/ros-controls/gazebo_ros2_control/issues/326>`_) (`#327 <https://github.com/ros-controls/gazebo_ros2_control/issues/327>`_)
  (cherry picked from commit 1ef9652ac34ed883dbf8fed27bcf393f78f53d52)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

0.4.8 (2024-05-14)
------------------
* Add PID controller to control joint using effort (`#294 <https://github.com/ros-controls/gazebo_ros2_control//issues/294>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Update precommit config (`#298 <https://github.com/ros-controls/gazebo_ros2_control//issues/298>`_) (`#301 <https://github.com/ros-controls/gazebo_ros2_control//issues/301>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 105c0ba5b786a43e1e9266399ab027a12011c643)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Fix incorrect force-torque sensor vec population (`#296 <https://github.com/ros-controls/gazebo_ros2_control//issues/296>`_) (`#299 <https://github.com/ros-controls/gazebo_ros2_control//issues/299>`_)
  (cherry picked from commit fdcd7aa8c67ea57f44bbf2f8fba90a28d7f04b5d)
  Co-authored-by: Mateus Menezes <mateusmenezes95@gmail.com>
* Contributors: chameau5050, mergify[bot]

0.4.7 (2024-03-21)
------------------
* Update gazebo_ros2_control_plugin.cpp (`#286 <https://github.com/ros-controls/gazebo_ros2_control/issues/286>`_) (`#287 <https://github.com/ros-controls/gazebo_ros2_control/issues/287>`_)
  (cherry picked from commit 5e1f9a52bcd1bf4164186dbb3c8b5cf070ed156e)
  Co-authored-by: Tobias Fischer <info@tobiasfischer.info>
* set the robot description parameter (`#277 <https://github.com/ros-controls/gazebo_ros2_control/issues/277>`_)
* Fix crashing due to an invalid parameter in the initial value. (backport `#271 <https://github.com/ros-controls/gazebo_ros2_control/issues/271>`_) (`#282 <https://github.com/ros-controls/gazebo_ros2_control/issues/282>`_)
  * Fix crashing due to an invalid parameter in the initial value. (`#271 <https://github.com/ros-controls/gazebo_ros2_control/issues/271>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit cdae6b8a8f638d87146482e99bf76cf36530e5a6)
  Co-authored-by: Wiktor Bajor <69388767+Wiktor-99@users.noreply.github.com>
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Contributors: AB, mergify[bot]

0.4.6 (2024-01-24)
------------------
* Load the URDF to the resource_manager before parsing it to CM  (`#262 <https://github.com/ros-controls/gazebo_ros2_control//issues/262>`_) (`#266 <https://github.com/ros-controls/gazebo_ros2_control//issues/266>`_)
  * Load the URDF to the resource_manager before parsing it to CM constructor (fixes https://github.com/ros-controls/ros2_control/issues/1299)
  (cherry picked from commit f5baf71c4c7cb3c0a0af52f988c107b356c95ed0)
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Fix links in documentation (`#263 <https://github.com/ros-controls/gazebo_ros2_control//issues/263>`_) (`#264 <https://github.com/ros-controls/gazebo_ros2_control//issues/264>`_)
  (cherry picked from commit d44b879615a539fc7c6c53707ec518df7bfd4f47)
  Co-authored-by: Silvio Traversaro <silvio@traversaro.it>
* Contributors: mergify[bot]

0.4.5 (2024-01-04)
------------------
* Fix stuck passive joints (`#237 <https://github.com/ros-controls/gazebo_ros2_control/issues/237>`_) (`#238 <https://github.com/ros-controls/gazebo_ros2_control/issues/238>`_)
  (cherry picked from commit 5dba0f95a03b136f39145c846ec5ebbfb5a09599)
  Co-authored-by: Johannes Huemer <johannes.huemer@ait.ac.at>
* Contributors: mergify[bot]

0.4.4 (2023-08-21)
------------------
* Catch pluginlib exceptions (backport `#229 <https://github.com/ros-controls/gazebo_ros2_control/issues/229>`_) (`#230 <https://github.com/ros-controls/gazebo_ros2_control/issues/230>`_)
  * Catch pluginlib exceptions (`#229 <https://github.com/ros-controls/gazebo_ros2_control/issues/229>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit f8a475d3092e67b77846d76738ffad0861c680c1)
* Set the C++ version to 17 (`#221 <https://github.com/ros-controls/gazebo_ros2_control/issues/221>`_) (`#228 <https://github.com/ros-controls/gazebo_ros2_control/issues/228>`_)
  (cherry picked from commit 6da415cf82a75e2a5e9f9a41400957ad45b2be84)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Removed unused var (`#220 <https://github.com/ros-controls/gazebo_ros2_control/issues/220>`_) (`#226 <https://github.com/ros-controls/gazebo_ros2_control/issues/226>`_)
  (cherry picked from commit 174e6b85f82774e9e802a5540382999066734421)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Remove plugin export from ROS 1 (`#212 <https://github.com/ros-controls/gazebo_ros2_control/issues/212>`_) (`#215 <https://github.com/ros-controls/gazebo_ros2_control/issues/215>`_)
  (cherry picked from commit c15af63cb036cd1f36cffbc56e5e5bdb5224c7e2)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Forced zero vel in position mode to avoid sagging (`#213 <https://github.com/ros-controls/gazebo_ros2_control/issues/213>`_) (`#214 <https://github.com/ros-controls/gazebo_ros2_control/issues/214>`_)
  (cherry picked from commit 3e950618a1f82c72097f7c90a6b5d2ea2e32b7b8)
  Co-authored-by: gwalck <guillaume.walck@stoglrobotics.de>
* Various bug fixes (`#177 <https://github.com/ros-controls/gazebo_ros2_control/issues/177>`_) (`#208 <https://github.com/ros-controls/gazebo_ros2_control/issues/208>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Add pre-commit and CI-format (`#206 <https://github.com/ros-controls/gazebo_ros2_control/issues/206>`_) (`#207 <https://github.com/ros-controls/gazebo_ros2_control/issues/207>`_)
  * Add pre-commit and ci-format
  (cherry picked from commit f2cf686a1a97cefc9b5e3daa115e0c4854ea5707)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, mergify[bot]

0.4.3 (2023-05-23)
------------------
* add copy operator to SafeEnum (`#197 <https://github.com/ros-controls/gazebo_ros2_control/issues/197>`_) (`#198 <https://github.com/ros-controls/gazebo_ros2_control/issues/198>`_)
* Contributors: mergify[bot]

0.4.2 (2023-03-02)
------------------
* Export all dependencies (`#183 <https://github.com/ros-controls/gazebo_ros2_control/issues/183>`_)
  The ament_export_dependencies exports dependencies to downstream
  packages. This is necessary so that the user of the library does
  not have to call find_package for those dependencies.
* Contributors: Adrian Zwiener

0.4.1 (2023-02-07)
------------------
* Force setting use_sim_time parameter when using plugin. (`#171 <https://github.com/ros-controls/gazebo_ros2_control/issues/171>`_)
* Removed warning (`#162 <https://github.com/ros-controls/gazebo_ros2_control/issues/162>`_)
* Mimic joint should have the same control mode as mimicked joint. (`#154 <https://github.com/ros-controls/gazebo_ros2_control/issues/154>`_)
* Enable loading params from multiple yaml files (`#149 <https://github.com/ros-controls/gazebo_ros2_control/issues/149>`_)
* Contributors: Alejandro Hernández Cordero, Denis Štogl, Tony Najjar

0.4.0 (2022-08-09)
------------------
* Implemented perform_command_mode_switch override in GazeboSystem (`#136 <https://github.com/ros-simulation/gazebo_ros2_control/issues/136>`_)
* added namespace to controller manager (`#147 <https://github.com/ros-simulation/gazebo_ros2_control/issues/147>`_)
* Activate all hardware in URDF (`#144 <https://github.com/ros-simulation/gazebo_ros2_control/issues/144>`_)
* activated all hardware by default (`#143 <https://github.com/ros-simulation/gazebo_ros2_control/issues/143>`_)
* Fix setting initial values if command interfaces are not defined. (`#110 <https://github.com/ros-simulation/gazebo_ros2_control/issues/110>`_)
* changed name to GazeboSystem (`#142 <https://github.com/ros-simulation/gazebo_ros2_control/issues/142>`_)
* Contributors: Denis Štogl, Keegan Sotebeer, Maciej Bednarczyk

0.3.1 (2022-07-05)
------------------
* Added logic for activating hardware interfaces (`#139 <https://github.com/ros-simulation/gazebo_ros2_control/issues/139>`_)
* Adjust repo URL (`#134 <https://github.com/ros-simulation/gazebo_ros2_control/issues/134>`_)
* Contributors: Alejandro Hernández Cordero, Bence Magyar

0.3.0 (2022-05-27)
------------------
* Merge pull request `#120 <https://github.com/ros-simulation/gazebo_ros2_control/issues/120>`_ from ros-simulation/ahcorde/main/117
  Adapted to Humble
* make linters happy
* Merge remote-tracking branch 'denis/using-under-namespace' into ahcorde/main/117
* update read/write interface functions of ros2_control parts
  This is needed since the ros2_control interfaces have been update
* Declare dependency of gazebo_hardware_plugins to urdf in CMakeLists.txt (`#117 <https://github.com/ros-simulation/gazebo_ros2_control/issues/117>`_)
* ros2_control is now having usings under its namespace.
* Fix mimic joint for effort command (`#109 <https://github.com/ros-simulation/gazebo_ros2_control/issues/109>`_)
* Support for mimic joints and example with gripper. (`#107 <https://github.com/ros-simulation/gazebo_ros2_control/issues/107>`_)
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, Denis Štogl, Manuel M, Martin Wudenka, ahcorde

0.0.8 (2022-01-28)
------------------
* Enable setting default position of the simulated robot using ros2_control URDF tag. (`#100 <https://github.com/ros-simulation/gazebo_ros2_control//issues/100>`_)
* Contributors: Denis Štogl

0.0.7 (2021-12-03)
------------------
* Pass ROS time instead of SYSTEM time to update function (`#97 <https://github.com/ros-simulation/gazebo_ros2_control//issues/97>`_)
* Contributors: Błażej Sowa

0.0.6 (2021-11-18)
------------------
* Fix ros2_control resource manager in galatic (`#96 <https://github.com/ros-simulation/gazebo_ros2_control//issues/96>`_)
* Contributors: Alejandro Hernández Cordero

0.0.4 (2021-10-26)
------------------
* Added testing CI (`#93 <https://github.com/ros-simulation/gazebo_ros2_control//issues/93>`_)
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* fix maintainer email (`#92 <https://github.com/ros-simulation/gazebo_ros2_control//issues/92>`_)
* Galactic: Pass time and period to update function (`#88 <https://github.com/ros-simulation/gazebo_ros2_control//issues/88>`_)
* Export interfaces created in init (`#83 <https://github.com/ros-simulation/gazebo_ros2_control//issues/83>`_)
* Add Imu and FT state interfaces (`#65 <https://github.com/ros-simulation/gazebo_ros2_control//issues/65>`_)
  Co-authored-by: Jordan Palacios <jordan.palacios@pal-robotics.com>
* Contributors: Alejandro Hernández Cordero, Bence Magyar, Błażej Sowa, Victor Lopez

0.0.3 (2021-06-16)
------------------
* Forward sdf ros remappings to loaded controllers (`#80 <https://github.com/ros-simulation/gazebo_ros2_control/issues/80>`_)
  Co-authored-by: Jonatan Olofsson <jonatan.olofsson@saabgroup.com>
* Join with the controller manager's executor thread on exit (`#79 <https://github.com/ros-simulation/gazebo_ros2_control/issues/79>`_)
* Ensure that sim_joints\_ always has the same number of elements as the… (`#77 <https://github.com/ros-simulation/gazebo_ros2_control/issues/77>`_)
* Write joints on each simulation update period (`#78 <https://github.com/ros-simulation/gazebo_ros2_control/issues/78>`_)
* Contributors: Jonatan Olofsson, Kenneth Bogert, Victor Lopez

0.0.2 (2021-04-19)
------------------
* add ros parameters file to node context (`#60 <https://github.com/ros-simulation/gazebo_ros2_control//issues/60>`_)
  Co-authored-by: ahcorde <ahcorde@gmail.com>
* Expose include path (`#58 <https://github.com/ros-simulation/gazebo_ros2_control//issues/58>`_)
* Added License file (`#55 <https://github.com/ros-simulation/gazebo_ros2_control//issues/55>`_)
* Fixed state interfaces (`#53 <https://github.com/ros-simulation/gazebo_ros2_control//issues/53>`_)
* Contributors: Alejandro Hernández Cordero, Chen Bainian, Karsten Knese

0.0.1 (2021-02-05)
------------------
* Updated with ros2-control Foxy API (`#44 <https://github.com/ros-simulation/gazebo_ros2_control/issues/44>`_)
  Co-authored-by: Karsten Knese <Karsten1987@users.noreply.github.com>
* Added initial version of gazebo_ros2_control (`#1 <https://github.com/ros-simulation/gazebo_ros2_control/issues/1>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Karsten Knese, Bence Magyar
