from Obstacle import Obstacle

class Robot(Obstacle):
    def __init__(self, position: tuple, radius: int):
        # Initialize x, y and r
        super().__init__(position, radius)

        # Initialize Goal
        self.goal: tuple = None

        # Initialize plot information
        self.lines: list = None # Will probably have to be a numpy array

    # Calculate an orca cycle given a list containing Obstacles and a timestep dt
    # Robot should set its Vcur to this calculate velocity vector.
    # Return a tuple (TODO: or list?) with its calculated velocity vector
    def orca_cycle(self, obstacles: list, dt: int) -> tuple:
        # TODO: Implement orca_cycle for individual Robot
        print("Orca cycle of robot...")

        return (0, 0)        

    # Get function for its current velocity
    # No setter since only the Robot itself will calculate its Vcur. External sources can only set its Vref.
    def get_current_velocity(self) -> tuple:
        return self.Vcur

    # Get function for its reference velocity
    def get_reference_velocity(self) -> tuple:
        return self.Vref

    # Setter for the reference velocity. 
    def set_reference_velocity(self, v_ref: tuple):
        self.Vref = v_ref # (Vref_x, Vref_y)

    # Get function for its bounding radius
    def get_radius(self) -> int:
        return self.r

    # Get function for its current position
    def get_position(self) -> tuple:
        return self.pos

    # Set function for the Robots position
    def set_position(self, new_position: tuple):
        if not isinstance(new_position, tuple):
            raise TypeError
        
        self.pos = new_position # (x, y)

    # Set function for the Robots goal
    def set_goal(self, goal: tuple):
        if not isinstance(goal, tuple):
            raise TypeError

        self.goal = goal

    def toString(self) -> str:
        str = "(Robot p={pos}, r={rad}, v={vel})".format(pos=self.pos, rad=self.r, vel=self.Vcur)
        return str
