"""simulation_parameters.py defines the timing parameters used for the simulation
"""

######################################################################################
#   sample times, etc
######################################################################################
ts_simulation: float = 0.01  # smallest time step for simulation
start_time: float = 0.0  # start time for simulation
end_time: float = 100.0  # end time for simulation

ts_plotting: float = 0.1  # refresh rate for plots

ts_video: float = 0.1  # write rate for video

ts_control: float = ts_simulation  # sample rate for the controller
