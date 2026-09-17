"""A shared, local Matplotlib theme; importing it does not load plotting libraries."""

from functools import wraps


def styled(function):
    @wraps(function)
    def wrapped(*args, **kwargs):
        from matplotlib import rc_context

        with rc_context({
            "font.family": "DejaVu Sans", "font.size": 10,
            "figure.facecolor": "#fafafa", "axes.facecolor": "#fafafa",
            "text.color": "#334155", "axes.labelcolor": "#64748b",
            "axes.edgecolor": "#cbd5e1", "axes.labelsize": 9,
            "axes.titlesize": 11, "axes.titlepad": 12,
            "xtick.color": "#64748b", "ytick.color": "#64748b",
            "xtick.labelsize": 8, "ytick.labelsize": 8,
            "grid.color": "#e2e8f0", "grid.linewidth": .6,
            "savefig.facecolor": "auto",
        }):
            return function(*args, **kwargs)
    return wrapped
