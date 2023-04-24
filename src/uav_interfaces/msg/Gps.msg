# Provides a pressure readings and associated covariance.
# A variance of 0 will be considered an unknown.
# If you have no estimate for one of the data elements (e.g., no differential pressure) then
# please set first element of covariance to -1.
# If you are interpreting this message, please check for a value of -1 and disregard the
# associated estimate.
geometry_msgs/PointStamped position
float64[9] position_transient_covariance
float64 ground_velocity
float64 ground_velocity_variance # i.e., std_dev squared
float64 course_ang
float64 course_angle_variance # i.e., std_dev squared
