"""Obstacle.py

This file defines the class of a generic obstacle. This class is used as a base class for Robot.py and Static.py
"""

import numpy as np


class Obstacle():

    def __init__(self, position: np.ndarray, radius: float):
        # Initialize parameters
        self.pos: np.ndarray = position  # (x, y, z) (z is 0)
        self.radius: float = radius

        # Initialize move instructions as 0 for the time being
        self.Vref: np.ndarray = np.zeros(3)  # (Vref_x, Vref_y, Vref_z) (z is zero)
        self.Vcur: np.ndarray = np.zeros(3)  # (Vref_x, Vref_y, Vref_z) (z is zero)

    def toString(self) -> str:
        raise RuntimeError("Abstract obstacle should not have been created and has no toString method")
