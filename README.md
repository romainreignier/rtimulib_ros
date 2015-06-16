# rtimu_ros
A simple package to use the nice RTIMULib from [RichardsTech](https://richardstechnotes.wordpress.com/) in ROS.

The RTIMULib needs to be installed. It can be found here : <https://github.com/richards-tech/RTIMULib>

## Topic
The fused orientation data are published on the `imu/data` topic.

## Calibration
The calibration needs to be performed by the RTIMULibCal utility provided by the library. Then, the file `RTIMULib.ini` file needs to be placed in the `config` directory.

## Launch
The node has to be launched with the `rtimulib_ros.launch` in order to load the calibration file.

        $ roslaunch rtimulib_ros rtimulib_ros.launch

## Visualization
The data from the IMU can be seen via the 3D visualization node from the `razor_imu_9dof` package.

Install it :

        $ sudo apt-get install ros-indogo-razor-imu-9dofs
 
 Launch the visualization node :

        $ roslaunch razor_imu_9dofs razor-display.launch
