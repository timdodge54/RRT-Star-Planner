# Provides a pressure readings and associated covariance.
# A variance of 0 will be considered an unknown.
# If you have no estimate for one of the data elements (e.g., no differential pressure) then
# please set element variance to -1.
# If you are interpreting this message, please check for a value of -1 and disregard the
# associated estimate.
builtin_interfaces/Time stamp
float64 abs_pressure    # Absolute pressure measurement
float64 abs_variance    # variance of the absolute pressure (i.e., std_dev squared)
float64 diff_pressure   # Differential pressure measurement
float64 diff_variance   # variance of the differential pressure (i.e., std_dev squared)