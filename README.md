# rtimulib_ros
A simple package to use the nice RTIMULib (unfortunately , now unmaintained) from [RichardsTech](https://richardstechnotes.wordpress.com/) in ROS.

The RTIMULib needs to be installed. It can be found here : <https://github.com/RTIMULib/RTIMULib>

## Topic
The fused orientation data are published on the `imu/data` topic by default but can be modified via the `topic_name` parameter.

## Calibration
The calibration needs to be performed by the `RTIMULibCal` utility provided by the library.
In case of a several devices configuration, `RTIMULibCal` can be launched with an argument to change the name of the calibration file.
Example:

        $ RTIMULibCal toto

Will produce the `toto.ini` file in the current directory.

Then, the file calibration file `.ini` needs to be placed in the `config` directory of the package.

If the calibration file has a custom name, it must be specified with the `calibration_file_name` parameter.

## Launch
The node has to be launched with the `rtimulib_ros.launch` in order to load the calibration file.

        $ roslaunch rtimulib_ros rtimulib_ros.launch

## Visualization
The data from the IMU can be seen via the 3D visualization node from the `razor_imu_9dof` package. But this package subscribe to the `/imu` topic so the `topic_name` parameter of the must be set to `/imu`.

Install it :

        $ sudo apt-get install ros-indogo-razor-imu-9dofs
 
 Launch the visualization node :

        $ roslaunch razor_imu_9dofs razor-display.launch

## Parameters
- `calibration_file_path`: (Mandatory) used to dynamically find the package config directory. Does not need to be modified.
- `calibration_file_path`: (Optional) used to use a different calibration file name than `RTIMULib.ini`. Do not include the `.ini` suffix.
- `topic_name`: (Optional) used to change the default `/imu/data` topic to publish the IMU data.
- `frame_id`: (Optional) used to change the default `imu_link` frame.
- `update_rate`: (Optional) used to change the default 20 Hz update rate of the node.
