Boston Cleek

# Overview
The orange path is gazebo, the green path is odometry, and the blue path is EKF slam. The blue cylinders are the landmarks estimated by EKF slam and the green cylinders are either from the feature detector or from gazebo.

# L.002 SLAM Known Data Association


landmark radius: 0.6 m
sensor noise: mean = 0.0 sigma = 0.00001

|          |      X Error (cm)      |  Y Error (cm) |  Yaw Error  (degrees) |
|----------|:-----------------:|---------:|-----------:|
|  Odometry  |  0.030     |   0.099    |     -7.964      |
| EKF SLAM   |     0.000      |   0.000    |     0.008       |

<p align="center">
  <img src="nuslam/media/ekf_known.png" width="350" height="350"/>
</p>


# L.002 SLAM Unknown Data Association

landmark radius: 0.6 m
sensor noise: mean = 0.0 sigma = 0.01

|          |      X Error (cm)      |  Y Error (cm) |  Yaw Error  (degrees) |
|----------|:-----------------:|---------:|-----------:|
|  Odometry  |  0.015     |   0.084   |     -6.975      |
| EKF SLAM   |     -0.008      |   0.038    |     -1.633      |


<p align="center">
  <img src="nuslam/images/ekf_unknown.png" width="350" height="350"/>
</p>
