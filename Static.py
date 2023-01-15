from Obstacle import Obstacle
import numpy as np


class Static(Obstacle):
    def __init__(self, position: np.ndarray, radius: float):
        # Initialize x, y and r
        # This also sets Vref and Vcur to 0
        super().__init__(position, radius)

        self.Vcur = np.zeros(3)
        self.Vref = np.zeros(3)
        self.collision_flag = False

    def toString(self) -> str:
        str = "(Static p={pos}, r={rad})".format(pos=self.pos, rad=self.radius, vel=self.Vcur)
        return str
