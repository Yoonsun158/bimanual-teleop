"""Independent rotation-distance assertions for motion tests."""

import math


def orientation_distance(a, b) -> float:
    """Shortest rotation angle; opposite quaternion signs describe one pose."""
    a = tuple(v / math.sqrt(sum(x*x for x in a)) for v in a)
    b = tuple(v / math.sqrt(sum(x*x for x in b)) for v in b)
    chord = min(math.dist(a, b), math.dist(a, tuple(-x for x in b)))
    return 4 * math.asin(min(1.0, chord / 2))
