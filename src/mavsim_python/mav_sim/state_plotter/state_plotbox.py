"""Needs Documentation
"""
from typing import Optional, cast

import pyqtgraph as pg
from mav_sim.state_plotter.plotter_args import PlotArgs, PlotboxArgs
from mav_sim.state_plotter.state_data import StateData
from mav_sim.state_plotter.state_plot import StatePlot
from pyqtgraph import ViewBox


class StatePlotbox:
    """Needs documentation"""

    def __init__(self, window: pg.GraphicsWindow, args: PlotboxArgs) -> None:
        """Create a new plotbox wrapper object

        Arguments:
            window (pg.GraphicsWindow): pyqtgraph window object in which to
                place this plotbox
            args (PlotboxArgs object): PlotboxArgs object which holds all the
                appropriate arguments for the plotbox

        """
        if not isinstance(args, PlotboxArgs):
            raise TypeError("'args' argument must be of type PlotboxArgs")
        # Initlialize plotbox
        if args.labels is not None:
            self.plotbox = window.addPlot(title=args.title, labels=args.labels)
        else:
            self.plotbox = window.addPlot(labels={"left": args.title})

        # Handle dimension parameters
        self.dimension = len(args.plots[0].state_names)
        if self.dimension == 1:
            self.plotbox.setAutoVisible(y=True)
        else:
            self.plotbox.setAutoVisible(x=True, y=True)
            self.plotbox.setAspectLocked()  # Lock x/y ratio to be 1

        # Handle color parameters
        self.set_axis_color(args.axis_color, args.axis_width)
        self.distinct_plot_hues = args.plot_hues
        self.plot_min_hue = args.plot_min_hue
        self.plot_max_hue = args.plot_max_hue
        self.plot_min_value = args.plot_min_value
        self.plot_max_value = args.plot_max_value

        if args.legend:
            self.add_legend()

        # Plots related to this plotbox
        self.plots: dict[str, StatePlot] = {}
        for p in args.plots:
            self.add_plot(p)

        # Other args
        self.time_window = args.time_window

    def label_axes(
        self, x_label: Optional[str] = None, y_label: Optional[str] = None
    ) -> None:
        """Set label on axis"""
        if x_label is not None:
            self.plotbox.setLabel("bottom", x_label)
        if y_label is not None:
            self.plotbox.setLabel("left", y_label)

    def set_axis_color(self, color: str, width: float = 1) -> None:
        """Sets the color of the axis"""
        self.axis_pen = pg.mkPen(color=color, width=width)
        self.plotbox.getAxis("left").setPen(self.axis_pen)
        self.plotbox.getAxis("bottom").setPen(self.axis_pen)

    def add_legend(self) -> None:
        """Turns on a legend"""
        self.plotbox.addLegend(size=(1, 1), offset=(1, 1))

    def add_plot(self, plot_args: PlotArgs) -> None:
        """Adds an additional plot"""
        if plot_args.color is None:
            plot_args.set_color(self._get_color(len(self.plots)))
        self.plots[plot_args.name] = StatePlot(self.plotbox, plot_args)

    def get_states(self) -> dict[str, StateData]:
        """Gets the states from each of the plots"""
        states = {}
        for p in self.plots.values():
            states.update(p.get_states())  # Adds the (k,v) pairs from p to states
        return states

    def get_xrange(self) -> list[int]:
        """Needs documentation"""
        return cast(list[int], self.plotbox.vb.targetRange()[0])

    def get_yrange(self) -> list[int]:
        """Needs documentation"""
        return cast(list[int], self.plotbox.vb.targetRange()[1])

    def update(self, t: float) -> None:
        """Update the plot data and adjust viewing range

        Arguments:
            t (float): the current time in seconds. Used to adjust the rolling
                time window appropriately
        """
        for p in self.plots.values():
            p.update()

        if self.dimension == 1:
            x_min = max(t - self.time_window, 0)
            x_max = t
            self.plotbox.setXRange(x_min, x_max)
            self.plotbox.enableAutoRange(axis=ViewBox.YAxis)
        else:
            self.plotbox.enableAutoRange(axis=ViewBox.XYAxes)
            # TODO: Add 3D support here

    def _get_color(self, index: int) -> str:
        """Returns incremental plot colors based on index"""
        return cast(
            str,
            pg.intColor(
                index,
                minValue=self.plot_min_value,
                maxValue=self.plot_max_value,
                hues=self.distinct_plot_hues,
                minHue=self.plot_min_hue,
                maxHue=self.plot_max_hue,
            ),
        )
