from Obstacle import Obstacle
import matplotlib.pyplot as plt
from VelocityObstacle import VelocityObstacle
from Solve import Solve
from matplotlib.patches import Polygon
import numpy as np


class Robot(Obstacle):
    def __init__(self, position: np.ndarray, radius: float, index: int):
        # Initialize x, y and r
        super().__init__(position, radius)

        # Initialize Goal
        self.goal: np.ndarray = np.NaN

        # Initialize VO information
        self.velocity_obstacles: list = []

        # Initialize cooperation factor
        self.cooperation_factor = 0.5

        # Initialize dynamic properties
        self.Vmax = 1
        self.accel = 1
        self.index = index
        self.follow_orca = True
        self.states = []

    # Calculate an orca cycle given a list containing Obstacles and a timestep orca_update_cycle
    # Robot should set its Vcur to this calculate velocity vector.
    # Return a tuple
    def orca_cycle(self, obstacles: list, dt: float) -> np.ndarray:
        # print("Orca cycle of other_obstacle ", self.index)
        self.velocity_obstacles: list = []
        constraints = None

        for i in range(len(obstacles)):
            if self != obstacles[i]:
                VO = VelocityObstacle(self, obstacles[i], dt)
                self.velocity_obstacles.append(VO)
                constraint = self.velocity_obstacles[-1].get_constraint()
                if constraints is None and constraint is not None:
                    constraints = np.array([constraint])
                elif constraint is not None:
                    constraints = np.vstack((constraints, constraint))

        # print(constraints)

        if constraints is not None:
            solver = Solve(constraints)
            velocity = solver.solve(self.Vref)
            return velocity
        else:
            return self.get_reference_velocity()

    def update_velocity_reference(self):
        vector_to_goal = self.goal - self.pos
        norm_vector_to_goal = vector_to_goal / np.linalg.norm(vector_to_goal)

        self.set_reference_velocity(norm_vector_to_goal * 1)

    # Plot the velocity obstacle shape and constraints
    def plot_orca_info(self):
        vo_plot_list = []
        constrain_plot_list = []

        for velocity_obstacle in self.velocity_obstacles:
            vo_plot = {}
            constrain_plot = {}

            if isinstance(velocity_obstacle.constraint, np.ndarray):
                # Get the shapes to draw from the velocity obstacle
                vo_polygon, line, constrain, border_point = velocity_obstacle.define_plotting_shapes()

                # Add VO shape as polygon
                vo_polygon.set(color="red", alpha=0.5)
                vo_plot["VO_polygon"] = vo_polygon

                # Add relative velocity and point that defines the constraint at the border of the VO
                vo_plot["relative_vel"] = velocity_obstacle.rel_vel
                vo_plot["border_point"] = border_point

                # Add the constraint
                constrain.set(color="green", alpha=0.5)
                constrain_plot["constrain"] = constrain

                if line[1] == None:
                    constrain_plot["line"] = line[0]
                else:
                    constrain_plot["line"] = np.array([line[0], line[1]])

            constrain_plot["Vcur"] = self.Vcur
            constrain_plot["Vref"] = self.Vref

            vo_plot_list.append(vo_plot)
            constrain_plot_list.append(constrain_plot)

        return vo_plot_list, constrain_plot_list

    def plot_position_info(self, color):
        # Plot top view of all robots
        plot_dic = {}
        circle = plt.Circle((self.pos[0], self.pos[1]), self.radius)
        circle.set(color=color, alpha=0.5)
        # ax.add_patch(circle)
        plot_dic["position"] = self.pos
        plot_dic["radius"] = self.radius

        plot_dic["Vref"] = self.Vref
        plot_dic["Vcur"] = self.Vcur
        plot_dic["color"] = color

        return plot_dic

    # Get function for its current velocity
    # No setter since only the Robot itself will calculate its Vcur. External sources can only set its Vref.
    def get_current_velocity(self) -> np.ndarray:
        return self.Vcur

    def set_current_velocity(self, v_cur):
        if v_cur.shape[0] < 3:
            raise RuntimeError

        # Determines if other_obstacle follows current_obstacle velocity or orca velocity
        threshold = 0.001  # A threshold for checking if velocities the same
        velocity_difference = abs(np.linalg.norm(v_cur - self.get_reference_velocity()))

        if velocity_difference <= threshold:
            self.follow_orca = False
            # print("Following ref vel")
        else:
            self.follow_orca = True
            # print("Following orca cycle")

        self.Vcur = v_cur

    def set_matching_velocity(self):
        self.Vcur = self.Vref

    # Get function for its current_obstacle velocity
    def get_reference_velocity(self) -> np.ndarray:
        return self.Vref

    def get_goal(self):
        return self.goal

    # Setter for the current_obstacle velocity.
    def set_reference_velocity(self, v_ref: np.ndarray):
        self.Vref = v_ref  # (Vref_x, Vref_y, Vref_z) (z is zero)

    # Get function for its bounding radius
    def get_radius(self) -> int:
        return self.radius

    # Get function for its current position
    def get_position(self) -> np.ndarray:
        return self.pos

    # Set function for the Robots position
    def set_position(self, new_position: np.ndarray):
        if not isinstance(new_position, np.ndarray):
            raise TypeError

        self.pos = new_position  # (x, y, z) (z is zero)
        self.states.append(new_position)

    # Set function for the Robots goal
    def set_goal(self, goal: np.ndarray):
        if not isinstance(goal, np.ndarray):
            raise TypeError

        self.goal = goal

    def set_cooperation_factor(self, factor):
        self.cooperation_factor = factor
        print(factor)

    def toString(self) -> str:
        str = "(Robot {ind} p={pos}, r={rad}, v={vel})".format(ind=self.index, pos=self.pos, rad=self.radius, vel=self.Vcur)
        return str
