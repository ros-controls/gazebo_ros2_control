^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2023-06-09)
------------------
* Add pre-commit and CI-format (`#206 <https://github.com/ros-controls/gazebo_ros2_control/issues/206>`_)
  * Add pre-commit and ci-format
* Compile with ROS iron and rolling (`#202 <https://github.com/ros-controls/gazebo_ros2_control/issues/202>`_)
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich

0.6.0 (2023-05-23)
------------------
* add copy operator to SafeEnum (`#197 <https://github.com/ros-controls/gazebo_ros2_control/issues/197>`_)
* Fixed rolling compilation (`#195 <https://github.com/ros-controls/gazebo_ros2_control/issues/195>`_)
* Export all dependencies (`#183 <https://github.com/ros-controls/gazebo_ros2_control/issues/183>`_) (`#184 <https://github.com/ros-controls/gazebo_ros2_control/issues/184>`_)
* Contributors: Alejandro Hernández Cordero, Noel Jiménez García, Adrian Zwiener

0.5.1 (2023-02-07)
------------------
* Various bug fixes (`#177 <https://github.com/ros-controls/gazebo_ros2_control/issues/177>`_)
* Contributors: AndyZe

0.5.0 (2023-01-06)
------------------
* Force setting use_sim_time parameter when using plugin. (`#171 <https://github.com/ros-controls/gazebo_ros2_control/issues/171>`_)
* Improve error message if robot_description\_ param is wrong (`#168 <https://github.com/ros-controls/gazebo_ros2_control/issues/168>`_)
* Rename hw info class type to plugin name (`#169 <https://github.com/ros-controls/gazebo_ros2_control/issues/169>`_)
* Removed warning (`#162 <https://github.com/ros-controls/gazebo_ros2_control/issues/162>`_)
* Mimic joint should have the same control mode as mimicked joint. (`#154 <https://github.com/ros-controls/gazebo_ros2_control/issues/154>`_)
* Enable loading params from multiple yaml files (`#149 <https://github.com/ros-controls/gazebo_ros2_control/issues/149>`_)
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, Denis Štogl, Tony Najjar

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
