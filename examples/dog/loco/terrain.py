"""Terrain side of the dog's controller (SP4): the height-map interface SP5's elevation map will implement, the ground-truth stand-in, and
(Task 8) footholds, posture plane, step height."""

from typing import Protocol

import numpy as np


class HeightMap(Protocol):
    def height(
        self, x, y
    ) -> np.ndarray: ...  # world z, numpy-broadcast; nan where the map has no cell


class FlatMap:
    def __init__(self, z=0.0):
        self.z = float(z)

    def height(self, x, y):
        return np.full(np.broadcast(np.asarray(x), np.asarray(y)).shape, self.z)


class TruthMap:
    """SP5 stand-in: the terrain's exact ground truth (napl.terrain Field.height: MuJoCo's hfield triangulation, box tops)."""

    def __init__(self, field):
        self.f = field

    def height(self, x, y):
        return np.asarray(
            self.f.height(np.asarray(x, float), np.asarray(y, float)), float
        )


# ---- flat defaults until Task 8 replaces the bodies -------------------------------------------------------------------------------
def foothold(hmap, xy, r_foot, **_):
    z = float(hmap.height(xy[0], xy[1]))
    return np.array([xy[0], xy[1], z + r_foot])


def apex_over(hmap, p0, p1, apex, clear=0.05):
    return apex


def plane(hmap, xy, yaw):
    return 0.0, 0.0, float(np.nanmean(hmap.height(xy[0], xy[1])))


def rough_ahead(hmap, xy, yaw, dist):
    return False
