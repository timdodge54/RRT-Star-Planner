# Provides line of sight information from the UAV to a feature/buoy.
# Any with a -1 in the first element of the covariance is considered not available.

# Range measurement from the camera to the feature
float64 range
float64 range_variance # i.e., std_dev squared
# The relative position vector from the camera to the feature, projected to the focal plane of the camera
float64[2] bearing
float64[4] bearing_covariance

