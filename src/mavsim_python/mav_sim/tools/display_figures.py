"""display_figures.py contains functions for displaying figures in Jupyter
"""

from IPython.display import Image, display
from mav_sim.chap2.mav_viewer import MavViewer
from mav_sim.chap3.data_viewer import DataViewer
from pyqtgraph.exporters import ImageExporter


def display_data_view(data_view: DataViewer, filename: str = "tmp.png") -> str:
    """Saves and displays an image for the DataViewer

    Args:
        data_view: Instance of DataViewer for which to capture an image
        filename: Name where file should be saved

    Returns:
        filename: file where image was saved
    """
    # Create an exporter
    scene = data_view.plotter.window.scene()  # allows for the capturing of the window
    exporter = ImageExporter(
        scene
    )  # provies a capability for storing the window as an image

    # Store a file and then display it
    exporter.export(filename)  # Store and display image
    display(Image(filename=filename))
    return filename


def display_mav_view(mav_view: MavViewer, filename: str = "tmp.png") -> str:
    """Saves and displays an image for the MavViewer

    Args:
        mav_view: Instance of the MavViewer for which to capture an image
        filename: Name where file should be saved

    Returns:
        filename: file where image was saved
    """
    # Store a file and then display it
    mav_view.window.readQImage().save(filename)
    display(Image(filename=filename))
    return filename
