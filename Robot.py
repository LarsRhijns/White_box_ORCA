from Obstacle import Obstacle
import matplotlib.pyplot as plt
from VelocityObstacle import VelocityObstacle
from Solve import Solve
from matplotlib.patches import Polygon
import numpy as np


class Robot(Obstacle):
    def __init__(self, position: np.ndarray, radius: int):
        # Initialize x, y and r
        super().__init__(position, radius)

        # Initialize Goal
        self.goal: np.ndarray = np.NaN

        # Initialize plot information
        self.lines: list = [] # Will probably have to be a numpy array

    # Calculate an orca cycle given a list containing Obstacles and a timestep dt
    # Robot should set its Vcur to this calculate velocity vector.
    # Return a tuple (TODO: or list?) with its calculated velocity vector
    def orca_cycle(self, obstacles: list, dt: float) -> np.ndarray:
        # TODO: Implement orca_cycle for individual Robot
        print("Orca cycle of robot...")
        self.velocity_obstacles: list = []
        constraints = np.zeros((len(obstacles) - 1, 3))

        counter = 0
        for i in range(len(obstacles)):
            if self != obstacles[i]:
                self.velocity_obstacles.append(VelocityObstacle(self, obstacles[i], dt))
                constraints[i - counter] = self.velocity_obstacles[-1].constraint
            else:
                counter += 1

        print(constraints)
        solver = Solve(constraints)
        velocity = solver.solve(self.get_reference_velocity())

        return velocity

    # Get function for its current velocity
    # No setter since only the Robot itself will calculate its Vcur. External sources can only set its Vref.

    # Plot the velocity obstacle shape and constraints
    def plot_orca_info(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)  # note we must use plt.subplots, not plt.subplot
        for velocity_obstacle in self.velocity_obstacles:
            # Get the shapes to draw from the velocity obstacle
            vo_polygon, line, constrain = velocity_obstacle.define_plotting_shapes()

            # Plot the VO
            vo_polygon.set(color="red", alpha=0.5)
            ax1.add_patch(vo_polygon)

            # Plot the constraint
            constrain.set(color="green", alpha=0.5)
            ax2.add_patch(constrain)
            ax2.axline((0, line[1]), slope=line[0], color='black')

        ax2.plot(self.Vcur[0], self.Vcur[1], marker="o", color="blue")
        ax2.plot(self.Vref[0], self.Vref[1], marker="o", color="yellow")
        # ax2.arrow(0, 0, self.Vref[0], self.Vref[1], head_width=0.5, head_length=0.5, fc="black", ec="black")

        ax1.grid()
        ax2.grid()
        plt.setp(ax1, xlim=[-10, 10], ylim=[-10, 10], xlabel="Velocity x", ylabel="Velocity y", title="Velocity obstacle")
        plt.setp(ax2, xlim=[-10, 10], ylim=[-10, 10], xlabel="Velocity x", ylabel="Velocity y", title="Velocity constraints")
        plt.show(block=True)

    # Get function for its current velocity
    # No setter since only the Robot itself will calculate its Vcur. External sources can only set its Vref.
    def get_current_velocity(self) -> np.ndarray:
        return self.Vcur

    def set_current_velocity(self, v_cur):
        self.Vcur = v_cur

    # Get function for its reference velocity
    def get_reference_velocity(self) -> np.ndarray:
        return self.Vref

    # Setter for the reference velocity. 
    def set_reference_velocity(self, v_ref: np.ndarray):
        self.Vref = v_ref # (Vref_x, Vref_y)

    # Get function for its bounding radius
    def get_radius(self) -> int:
        return self.r

    # Get function for its current position
    def get_position(self) -> np.ndarray:
        return self.pos

    # Set function for the Robots position
    def set_position(self, new_position: np.ndarray):
        if not isinstance(new_position, np.ndarray):
            raise TypeError
        
        self.pos = new_position # (x, y)

    # Set function for the Robots goal
    def set_goal(self, goal: np.ndarray):
        if not isinstance(goal, np.ndarray):
            raise TypeError

        self.goal = goal

    def toString(self) -> str:
        str = "(Robot p={pos}, r={rad}, v={vel})".format(pos=self.pos, rad=self.r, vel=self.Vcur)
        return str
