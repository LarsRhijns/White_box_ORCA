import numpy as np

class Obstacle():

    # Super class for 
    def __init__(self, position: np.ndarray, radius: float):
        # Initialize parameters
        self.pos: np.ndarray = position # (x, y)
        self.radius: float = radius

        # Initialize move instructions as 0 for the time being
        self.Vref: np.ndarray = np.zeros(2) # (Vref_x, Vref_y)
        self.Vcur: np.ndarray = np.zeros(2) # (Vref_x, Vref_y)

    def toString(self) -> str:
        raise RuntimeError("Abstract obstacle should not have been created and has no toString method")
