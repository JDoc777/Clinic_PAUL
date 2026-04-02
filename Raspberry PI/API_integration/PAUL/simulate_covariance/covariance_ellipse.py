import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_covariance_ellipse(ax, x, y, P, n_std=2.0, edge_color='red'):
    """
    Plot the 2D covariance ellipse for pose (x, y).

    Parameters
    ----------
    ax : matplotlib axis
    x, y : float
        Center of ellipse (robot position)
    P : 3x3 covariance matrix
        Only the top-left 2x2 block is used
    n_std : float
        Number of standard deviations (2 = 95%)
    edge_color : str
        Color of the ellipse outline
    """

    # Extract 2x2 covariance of (x, y)
    cov = P[0:2, 0:2]

    # Eigen-decomposition
    eigenvals, eigenvecs = np.linalg.eigh(cov)

    # Sort eigenvalues largest -> smallest
    order = eigenvals.argsort()[::-1]
    eigenvals = eigenvals[order]
    eigenvecs = eigenvecs[:, order]

    # Width/height of ellipse (scaled by n_std)
    width  = 2 * n_std * np.sqrt(eigenvals[0])
    height = 2 * n_std * np.sqrt(eigenvals[1])

    # Angle of ellipse (in degrees)
    angle = np.degrees(np.arctan2(eigenvecs[1,0], eigenvecs[0,0]))

    # Create ellipse patch
    ell = Ellipse(
        xy=(x, y),
        width=width,
        height=height,
        angle=angle,
        edgecolor=edge_color,
        facecolor='none',
        linewidth=2
    )

    ax.add_patch(ell)
