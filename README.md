# linear_kalman

Upsamples to 1000Hz using linear kalman filter for x,y,z measurements read from topic /predictions. Also filters using PT1.
Publishes upsampled on /flex_sensor/camera_estimate and PT1 filtered on /flex_sensor/camera_estimate_filtered

TODO: In KalmanEstimator.cpp change subscription and pub topics. Node expects geometry_msgs/PoseStamped, and publishes the same. 
Build package.

Run:
```
    rosrun linear_kalman upsampling_kalman
```


Kalman filter implementation from github.com/larics \[pozyx\]

