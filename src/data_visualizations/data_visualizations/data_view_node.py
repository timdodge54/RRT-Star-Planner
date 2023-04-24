#!/usr/bin/env python3
import bisect
import string
from random import random
from turtle import color
from typing import Any, Dict, List, Optional

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus



def trim_data(time: List[float], data: List[float], val_thresh: float) -> None:
    """Trims the time and data vectors to only have time values over the latest 'val_thresh' window"""

    ind = bisect.bisect_left(time, time[-1] - val_thresh)
    if ind > 0:
        del data[0:ind]
        del time[0:ind]


def format_seconds(time: float, _: Any) -> str:
    if time < 0:
        return ""
    time = float(time % (24 * 60 * 60))
    hours, minutes, seconds = (
        int(time // 3600),
        int((time % 3600) // 60),
        int(time % 60),
    )
    if seconds == 0:
        return f"{hours:02d}:{minutes:02d}"
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


class Plotting(Node):

    def __init__(
        self,
        load_window_size: float,
    ) -> None:
        super().__init__("status_plotter")

        # Create the diagnostic publisher
        self.pub_diag = self.create_publisher(DiagnosticArray, "diagnostics", 1)

        # Initialize storage variables
        self.declare_parameter("bus_ids")
        bus_ids: List[int] = self.get_parameter("bus_ids").value  # type:ignore
        num_chargers = len(
            bus_ids
        )  # TODO: base this off of the actual number of chargers
        self.bus_charge: Dict[int, float] = {}  # Maps the bus_id to the bus charge
        self.bus_charge_max = 100  # Max charge value seen by the bus status
        self.charger_usage: Dict[int, float] = {}  # Maps the charger index to the usage
        self.load_window_size = load_window_size

        # Initialize bus variable names
        self.bus_ids = bus_ids.copy()
        self.bus_names: List[str] = []
        for bus_id in self.bus_ids:
            self.bus_names.append(str(bus_id))
            self.bus_charge[bus_id] = 0.0

        # Create a new figure
        # self.fig, axs = plt.subplots(2, 2, figsize=(20, 15))
        self.fig, axs = plt.subplot_mosaic("CU;LL", figsize=(20, 15))
        plt.subplots_adjust(wspace=0.25, hspace=0.25)
        self.charge_ax: plt.Axes = axs["C"]
        self.usage_ax: plt.Axes = axs["U"]
        self.load_ax: plt.Axes = axs["L"]

        # Initialize the bus charge plot
        self.charge_ax.set_title("Bus Charge Level")
        self.charge_ax.set_ylabel("Battery Charge (kWh)")
        self.charge_ax.set_xlabel("Bus ID")
        self.charge_ax.tick_params(axis="x", labelrotation=90)
        self.charge_ax.set_ylim(0, self.bus_charge_max)
        self.charge_plot = self.charge_ax.bar(
            self.bus_names,
            [0] * len(self.bus_names),
            color="green",
            edgecolor="black",
        )
        
        plt.pause(0.00000001)

        self.timer = self.create_timer(1, self.main_loop)

    def main_loop(self) -> None:

        # Update plots
        self.plot_charge()
        self.plot_usage()
        self.plot_loads()
        plt.show(block=False)
        plt.pause(0.00000001)

        publish_single_diagnostic(self, self.pub_diag, "Plotting data", DiagnosticStatus.OK)

    def get_data(self, time: List[float], data: List[float], val_thresh: float) -> None:
        """Create random data for a list, emlinates data past threshold."""
        data.append(random() * 1000)
        time.append(time[-1] + 1)

        ind = bisect.bisect_left(time, time[-1] - val_thresh)
        if ind > 0:
            del data[0:ind]
            del time[0:ind]

    def plot_charge(self):
        """Loop through the battery bar graph and updates the battery data."""
        for index, bus_id in enumerate(self.bus_ids):
            # Update the charge bar plot
            battery = self.bus_charge[bus_id]
            bar = self.charge_plot.patches[index]
            bar.set_height(battery)
            self.charge_ax.draw_artist(bar)

            # Store the greatest charge seen
            self.bus_charge_max = np.max([self.bus_charge_max, battery])

        # Update the x-axis to the largest charge seen
        self.charge_ax.set_ylim(0, self.bus_charge_max)

    def plot_usage(self):
        """Loop through charger usage graph and update the charger usage data."""
        for index, _ in enumerate(self.charger_names):
            usage = self.charger_usage[index]
            bar = self.usage_plot.patches[index]
            bar.set_height(usage)
            self.usage_ax.draw_artist(bar)

    def plot_loads(self):
        """Update the load plots."""
        err_str = ""
        try:
            # Update plot
            self.chargers_plot.set_data(self.chargers_time, self.chargers_data)
            err_str = (
                err_str
                + "len(chargers_time) = "
                + str(len(self.chargers_time))
                + ", len(chargers_data): "
                + str(len(self.chargers_data))
                + "\n"
            )

            # Resize the axes
            self.load_ax.relim()
            self.load_ax.autoscale_view(True, True, True)
        except Exception as e:
            print("plot_status_node::plot_loads() Failed to plot")
            print(err_str)
            print(e)

    def create_subscriptions(self) -> List[rclpy.subscription.Subscription]:
        """Creates the subscriptions needed for updating the data"""
        # Initialize the output
        subs: List[rclpy.subscription.Subscription] = []

        # Add subscribers for all the bus_ids
        for id in self.bus_ids:
            # Get the bus id topic and add a subscriber
            topic_request = TopicRequest.Request.BUS_STATUS_API
            status_topic = get_topic_name(id, topic_request, self, self.pub_diag)
            subs.append(
                self.create_subscription(
                    BusStatus, status_topic, self.bus_status_callback, 1
                )
            )

        # Add subscriber for the charger status
        subs.append(
            self.create_subscription(
                ChargerStatus,
                "ocpp/charger_status",
                self.charger_status_callback,
                1
            )
        )

        # Add subscriber for the power usage statistics
        subs.append(
            self.create_subscription(
                PowerUsage, "power_metrics", self.power_status_callback, 1
            )
        )

        return subs

    def bus_status_callback(self, data: BusStatus) -> None:
        """Read in a status update message and updates the bus SOC."""

        if data.bus_id in self.bus_charge.keys():
            self.bus_charge[data.bus_id] = data.soc
        else:
            self.get_logger().error(
                "Plotting::status_callback() bus_id not accounted for, id = "
                f"{str(data.bus_id)}"
            )

    def charger_status_callback(self, data: ChargerStatus) -> None:
        """Read in the charger status message and updates the charger status."""
        for status in data.statuses:
            if status.charger_id in self.charger_usage.keys():
                self.charger_usage[status.charger_id] = status.kw_util
            else:
                self.get_logger().error(
                    "Plotting::charge_status_callback()"
                    f"charge_id not accounted for, id = {str(status.charger_id)}"
                )

    def power_status_callback(self, data: PowerUsage) -> None:
        """Read in power usage and stores the data into the appropriate lists."""
        # Convert time to seconds and add it to each of the time vectors
        time_sec = data.time.sec
        self.peak_time.append(time_sec)
        self.p_15_time.append(time_sec)
        self.max_p_15_time.append(time_sec)
        self.chargers_time.append(time_sec)
        self.tpss_time.append(time_sec)

        # Extract the data
        self.peak_data.append(data.peak)
        self.p_15_data.append(data.p_15)
        self.max_p_15_data.append(data.max_p_15)
        self.chargers_data.append(data.chargers)
        self.tpss_data.append(data.tpss)

        # Trim the data to the appropriate window size
        trim_data(self.peak_time, self.peak_data, self.load_window_size)
        trim_data(self.p_15_time, self.p_15_data, self.load_window_size)
        trim_data(self.max_p_15_time, self.max_p_15_data, self.load_window_size)
        trim_data(self.chargers_time, self.chargers_data, self.load_window_size)
        trim_data(self.tpss_time, self.tpss_data, self.load_window_size)


def plot_data(args=None):
    """Repeatedly plots the aggregated bus data using the latest available data"""
    # Initialize the node
    rclpy.init(args=args)
    # Read in data from parameter file
    load_window_size = 30 * 120.0
    plots = Plotting(load_window_size)
    plots.create_subscriptions()

    rclpy.spin(plots)
    plots.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        plot_data()
    except KeyboardInterrupt:
        pass
