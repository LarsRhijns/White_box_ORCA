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

        # TODO This breaks gym
        # self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2)  # note we must use plt.subplots, not plt.subplot
        # self.fig3, self.ax3 = plt.subplots(1, 1)  # note we must use plt.subplots, not plt.subplot


    def get_obstacles(self):
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot1(self, position: np.ndarray, radius: float, index: int) -> list:
        robot = Robot(position, radius, index)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot2(self, position: np.ndarray, radius: float, vel_ref: np.ndarray, index: int) -> list:
        robot = Robot(position, radius, index)
        robot.set_reference_velocity(vel_ref)
        robot.set_current_velocity(vel_ref)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot3(self, position: np.ndarray, radius: float, vel_ref: np.ndarray, goal: np.ndarray, index: int) -> list:
        robot = Robot(position, radius, index)
        robot.set_reference_velocity(vel_ref)
        robot.set_current_velocity(vel_ref)
        robot.set_goal(goal)
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

    # New_positions is a list with tuples with positions for each robot (and static obstacles??)
    # TODO this now assumes that new_positions is always in the same order as robots in our list.
    def update_all(self, new_positions: np.ndarray, new_velocities: np.ndarray) -> list:
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                self.obstacles[i].set_position(new_positions[i])
                self.obstacles[i].update_velocity_reference(self.simulation_cycle)
                self.obstacles[i].set_current_velocity(new_velocities[i])

        return self.obstacles

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
        self.ax1.cla()
        self.ax2.cla()

        if isinstance(self.obstacles[self.plot_index], Robot):
            self.ax1, self.ax2 = self.obstacles[self.plot_index].plot_orca_info(self.ax1, self.ax2)
        else:
            raise RuntimeError("The selected plot index is not a robot")

        # plt.show(block=True)

    def update_position_plot(self):
        self.ax3.cla()
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                color = "gray"
                if i == self.plot_index:
                    color = "blue"

                self.ax3 = self.obstacles[i].plot_position_info(self.ax3, color)