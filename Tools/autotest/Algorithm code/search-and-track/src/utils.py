import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path


def plot_polygon(ax, poly, **kwargs):
    """Plot a shapely polygon object using matplotlib.
    Source: https://stackoverflow.com/a/70533052

    :param ax: axes to plot the polygon object on.
    :param poly: polygon to plot.
    :returns: polygon collection.
    """
    path = Path.make_compound_path(
        Path(np.asarray(poly.exterior.coords)[:, :2]),
        *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors]
    )

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)

    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection
