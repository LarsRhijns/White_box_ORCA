"""Observation.py

This file consist of the observation class which keeps track of all the positions and velocities of each robot
"""

from Obstacle import Obstacle
from Robot import Robot
from Static import Static
import numpy as np


class Observation:
    def __init__(self, orca_update_cycle: float, simulation_cycle: float):
        self.obstacles: list[Obstacle] = []
        self.plot_index: int = 0
        self.orca_update_cycle: float = orca_update_cycle
        self.simulation_cycle: float = simulation_cycle

        self.vo_plots = []
        self.constraint_plots = []
        self.position_plots = []

    def get_obstacles(self):
        return self.obstacles

    # This method adds a robot to the observation
    def add_robot(self, position: np.ndarray, radius: float, goal: np.ndarray, index: int, dt: float, cooperation_factor=0.5, ) -> list:
        robot = Robot(position, radius, index)
        robot.set_goal(goal)
        robot.set_cooperation_factor(cooperation_factor)
        robot.update_velocity_reference(dt)
        robot.set_matching_velocity()

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

    # Do an ORCA cycle for each robot
    def orca_cycle(self) -> list:
        new_velocities = []  # Will be a list with tuples
        for obstacle in self.obstacles:
            if isinstance(obstacle, Robot):
                new_velocity = obstacle.orca_cycle(self.obstacles, self.orca_update_cycle)
                # obstacle.set_current_velocity(new_velocity)
                new_velocities.append(new_velocity)

        return new_velocities

    # This method checks if obstacles are in collision
    def check_for_collisions(self):
        for current in self.obstacles:
            pos1 = current.pos
            for other in self.obstacles:
                if other != current:
                    pos2 = other.pos
                    distance = np.linalg.norm(pos1 - pos2)

                    if distance <= current.radius + other.radius + 0.001:
                        current.collision_flag = True
                        other.collision_flag = True

    # Update all the positions in the observation
    def update_positions(self, new_positions: list, dt: float) -> list:
        for i, obstacle in enumerate(self.obstacles):
            if isinstance(obstacle, Robot):
                obstacle.set_position(new_positions[i])
                obstacle.update_velocity_reference(dt)

        self.check_for_collisions()

        return self.obstacles

    # Update the velocities of each robot in the observation
    def update_velocities(self, new_velocities: list) -> list:
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                self.obstacles[i].set_current_velocity(new_velocities[i])

        return self.obstacles

    # Update the information shown in the ORCA plot
    def update_orca_plot(self):
        if isinstance(self.obstacles[self.plot_index], Robot):
            vo_plot, constrain_plot = self.obstacles[self.plot_index].plot_orca_info()
            self.vo_plots.append(vo_plot)
            self.constraint_plots.append(constrain_plot)
        else:
            raise RuntimeError("The selected plot index is not a other_obstacle")

    # Update the information shown in the position plot
    def update_position_plot(self):
        plot_per_robot = []
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                color = "gray"
                if i == self.plot_index:
                    color = "blue"

                plot_per_robot.append(self.obstacles[i].plot_position_info(color))

        self.position_plots.append(plot_per_robot)
