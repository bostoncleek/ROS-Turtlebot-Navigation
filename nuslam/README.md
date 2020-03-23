Boston Cleek

# How to run
To run with ground truth data from gazebo:

`roslaunch nulsam slam.launch debug:=true`

To run with lidar sensor and feature detection:

`roslaunch nulsam slam.launch debug:=false`

You can adjust the landmark visibility range and noise levels for the ground truth data in `landmarks.launch`. You can remove the noise completely by setting the `noise` parameter to false.

You can run the EKF with the know data or unknown data association. Set the parameter `known_data_association` in `slam.launch` to false to run with unknown data association. If running with the `known_data_association` set to true make sure to set `debug` to true as well.
