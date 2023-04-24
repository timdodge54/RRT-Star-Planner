"""
scsim_python: spacecraft viewer (for appendix C)
    - Beard & McLain, PUP, 2012
    - Update history:
        1/13/2021 - TWM
        12/21 - GND
"""
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from mav_sim.appC.draw_sc import DrawSc
from mav_sim.message_types.msg_state import MsgState
from pyqtgraph import Vector


class ScViewer:
    """Object for viewing the spacecraft"""

    def __init__(self) -> None:
        """Initialize"""
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle("Spacecraft Viewer")
        self.window.setGeometry(
            0, 0, 1000, 1000
        )  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem()  # make a grid to represent the ground
        grid.scale(20, 20, 20)  # set the size of the grid (distance between each line)
        self.window.addItem(grid)  # add grid to viewer
        self.window.setCameraPosition(
            distance=200
        )  # distance from center of plot to camera
        self.window.setBackgroundColor("k")  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the spacecraft been plotted yet?
        self.sc_plot: DrawSc

    def update(self, state: MsgState) -> None:
        """Update drawing"""
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.sc_plot = DrawSc(state, self.window)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.sc_plot.update(state)
        # update the center of the camera view to the spacecraft location
        view_location = Vector(
            state.east, state.north, state.altitude
        )  # defined in ENU coordinates
        self.window.opts["center"] = view_location
        # redraw
        self.app.processEvents()
