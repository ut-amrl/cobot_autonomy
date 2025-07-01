std_msgs/Header header

# Raw accelerometer readings.
geometry_msgs/Vector3 raw_acceleration

# Accelerometer readings transformed to robot coordinate frame using 
# extrinsic calibration of the IMU.
geometry_msgs/Vector3 acceleration