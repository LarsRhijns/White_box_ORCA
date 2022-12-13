class Obstacle():

    # Super class for 
    def __init__(self, position: tuple, radius: int):
        # Initialize parameters
        self.pos: tuple = position # (x, y)
        self.r: int = radius

        # Initialize move instructions as 0 for the time being
        self.Vref: tuple = (0, 0) # (Vref_x, Vref_y)
        self.Vcur: tuple = (0, 0) # (Vref_x, Vref_y)

    def toString(self) -> str:
        raise RuntimeError("Abstract obstacle should not have been created and has no toString method")
