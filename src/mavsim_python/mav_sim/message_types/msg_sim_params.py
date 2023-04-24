"""msg_sim_params.py defines the parameters needed for simulation
"""
# pylint: disable=too-many-arguments

from typing import Optional


class MsgSimParams:
    """
    Provides parameters for running a simulation
    """

    def __init__(
        self,
        ts_simulation: float = 0.01,
        start_time: float = 0.0,
        end_time: float = 100.0,
        ts_plotting: float = 0.1,
        ts_video: float = 0.1,
        ts_control: Optional[float] = None,
        write_video: bool = False,
        video_name: str = "default.avi",
    ) -> None:
        """Store parameters"""
        self.ts_simulation: float = ts_simulation  # smallest time step for simulation
        self.start_time: float = start_time  # start time for simulation
        self.end_time: float = end_time  # end time for simulation
        self.ts_plotting: float = ts_plotting  # refresh rate for plots
        self.ts_video: float = ts_video  # write rate for video

        self.ts_control: float = ts_simulation  # sample rate for the controller
        if ts_control is not None:
            self.ts_control = ts_control

        self.write_video = write_video  # True => write video, False => do not
        self.video_name = video_name  # Name of the video file, if created
