from Obstacle import Obstacle

class Static(Obstacle):
    def __init__(self, position: tuple, radius: int):
        # Initialize x, y and r
        # This also sets Vref and Vcur to 0
        super().__init__(position, radius)

    def toString(self) -> str:
        str = "(Static p={pos}, r={rad})".format(pos=self.pos, rad=self.r, vel=self.Vcur)
        return str