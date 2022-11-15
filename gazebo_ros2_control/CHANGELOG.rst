^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-11-15)
------------------
* Enable loading params from multiple yaml files (`#149 <https://github.com/ros-controls/gazebo_ros2_control/issues/149>`_) (`#151 <https://github.com/ros-controls/gazebo_ros2_control/issues/151>`_)
  Co-authored-by: Tony Najjar <tony.najjar@logivations.com>
* [Backport Foxy] Support for mimic joints and example with gripper. (`#121 <https://github.com/ros-controls/gazebo_ros2_control/issues/121>`_)
* [backport Foxy] `#136 <https://github.com/ros-controls/gazebo_ros2_control/issues/136>`_ `#147 <https://github.com/ros-controls/gazebo_ros2_control/issues/147>`_ `#139 <https://github.com/ros-controls/gazebo_ros2_control/issues/139>`_ `#134 <https://github.com/ros-controls/gazebo_ros2_control/issues/134>`_ (`#150 <https://github.com/ros-controls/gazebo_ros2_control/issues/150>`_)
  Co-authored-by: Keegan Sotebeer <ksotebeer95@gmail.com>
  Co-authored-by: Maciej Bednarczyk <83034299+mcbed@users.noreply.github.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* Contributors: Alejandro Hernández Cordero, Leander Stephen D'Souza

0.1.0 (2022-05-27)
------------------
* Declare dependency of gazebo_hardware_plugins to urdf in CMakeLists.txt (`#117 <https://github.com/ros-simulation/gazebo_ros2_control/issues/117>`_) (`#119 <https://github.com/ros-simulation/gazebo_ros2_control/issues/119>`_)
  Co-authored-by: Martin Wudenka <Martin.Wudenka@gmx.de>
* Contributors: Alejandro Hernández Cordero

0.0.4 (2021-10-26)
------------------
* fix maintainer email (`#92 <https://github.com/ros-simulation/gazebo_ros2_control//issues/92>`_)
* Export interfaces created in init (`#83 <https://github.com/ros-simulation/gazebo_ros2_control//issues/83>`_)
* Add Imu and FT state interfaces (`#65 <https://github.com/ros-simulation/gazebo_ros2_control//issues/65>`_)
  Co-authored-by: Jordan Palacios <jordan.palacios@pal-robotics.com>
* Contributors: Alejandro Hernández Cordero, Błażej Sowa, Victor Lopez

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
