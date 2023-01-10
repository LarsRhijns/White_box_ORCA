import matplotlib.pyplot as plt
from Obstacle import Obstacle
from Robot import Robot
from Static import Static
import numpy as np


class Observation:
    def __init__(self, orca_update_cycle: float, simulation_cycle: float):
        self.obstacles: list[Obstacle] = []
        self.plot_index: int = 0  # Initialize to -1 since there are no robots to start with thus triggering an error on purpose when accessed
        self.orca_update_cycle: float = orca_update_cycle
        self.simulation_cycle: float = simulation_cycle

        self.vo_plots = []
        self.constraint_plots = []
        self.position_plots = []

    def get_obstacles(self):
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot(self, position: np.ndarray, radius: float, goal: np.ndarray, index: int, cooperation_factor=0.5) -> list:
        robot = Robot(position, radius, index)
        robot.set_goal(goal)
        robot.set_cooperation_factor(cooperation_factor)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a static obstacle to the obstacles
    def add_static(self, position: np.ndarray, radius: float) -> list:
        static = Static(position, radius)
        self.obstacles.append(static)
        return self.obstacles

    # Simple toString method. Could be adapted to provide more info
    def toString(self) -> str:
        str = "orca_update_cycle = {dt}\nObstacles = [".format(dt=self.orca_update_cycle)

        for obstacle in self.obstacles:
            str += obstacle.toString() + ", "

        str = str[:len(str) - 2]  # Remove the last ", " from the string
        str += "]"
        return str

    # Loop over the robots and make them all do an orca cycle
    # TODO this assumes that the obstacle list does not change order for the simulation and this class
    # Returns a list with all the new velocities for the obstacles (as tuples)
    def orca_cycle(self) -> list:
        new_velocities = []  # Will be a list with tuples
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                new_velocities.append(self.obstacles[i].orca_cycle(self.obstacles, self.orca_update_cycle))

        return new_velocities

    def update_positions(self, new_positions: list) -> list:
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                self.obstacles[i].set_position(new_positions[i])
                self.obstacles[i].update_velocity_reference(self.simulation_cycle)

        return self.obstacles

    def update_velocities(self, new_velocities: list) -> list:
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                self.obstacles[i].set_current_velocity(new_velocities[i])

        return self.obstacles

    def update_orca_plot(self):
        # self.ax1.cla()
        # self.ax2.cla()

        if isinstance(self.obstacles[self.plot_index], Robot):
            vo_plot, constrain_plot = self.obstacles[self.plot_index].plot_orca_info()
            self.vo_plots.append(vo_plot)
            self.constraint_plots.append(constrain_plot)
        else:
            raise RuntimeError("The selected plot index is not a robot")

        # plt.show(block=True)

    def update_position_plot(self):
        plot_per_robot = []
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                color = "gray"
                if i == self.plot_index:
                    color = "blue"

                plot_per_robot.append(self.obstacles[i].plot_position_info(color))

        self.position_plots.append(plot_per_robot)
