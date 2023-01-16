"""Main_simulations.py

This file consist of the main function to run multiple simulations at ones. When the simulations are finished the results are
saved as csv file in a data directory.
"""

# Import multiple packages
import os

import gym
import sys
import pandas as pd

sys.path.append("gym_envs_urdfs")
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
import numpy as np
from Observation import Observation
import pybullet as pb
from Robot import Robot
import time

# Define simulation variables
simulation_cycle = 0.01  # Update cycle of the simulation
orca_update_cycle = 16  # Update cycle for ORCA
radius = 0.2  # Radius of robots
circle_radius = 8  # Circle radius used for initialization of robots
total_time = 60  # Total simulation time
obstacle_run = False  # Obstacle run

# Variables used for static obstacle run
obstacle_radius = 0.5  # Obstacle radius
obstacle_location = [0.0, 0.01, 0.0]  # Obstacle location

# Variables used to set a custom cooperation factor
custom_cooperation_factor_index = 0  # Select a robot based on index to change factor
cooperation_factor = 0.5  # Custom cooperation factor

steps = int(total_time // simulation_cycle)
obst1Dict = {
    "type": "sphere",
    "geometry": {"position": obstacle_location, "radius": obstacle_radius},
}


# Fetch positions to the simulation environment
def fetch_positions(last_observation):
    positions = []
    for robot in last_observation.keys():
        positions.append(last_observation[robot]["joint_state"]["position"])

    return positions


# Function that checks if all the robots reach there goal
def check_for_finished(obstacles):
    all_finished = True
    for robot in obstacles:
        if isinstance(robot, Robot):
            position_difference = abs(np.linalg.norm(robot.get_position() - robot.get_goal()))
            if position_difference > 0.01:
                all_finished = False
                return all_finished
    return all_finished


# This method is responsible for running the simulation
def run_point_robot(n_steps=2000, render=False, obser=None, robot_amount=None, tau=None):
    # Append all robot to the simulation environment
    robots = []
    for i in range(robot_amount):
        robots.append(GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"))

    # Make the environment and define the camera angle
    env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)
    w, h, vmat, projmat, camup, camfwd, hor, ver, yaw, pitch, dist, target = pb.getDebugVisualizerCamera()
    pb.resetDebugVisualizerCamera(dist, 0, -80, target)
    n = env.n()

    # Initialize some variables
    ns_per_robot = env.ns_per_robot()
    initial_positions = np.zeros((len(robots), ns_per_robot[0]))
    action = np.zeros(n)

    count = 0
    # Set the initial state and goal of every robot
    for i in range(robot_amount):
        angle = ((2 * np.pi) / robot_amount) * i
        position = np.array([circle_radius * np.cos(angle), circle_radius * np.sin(angle), 0])
        initial_positions[i, :] = position

        referece_velocity = np.zeros(3)
        action[count: count + ns_per_robot[i]] = referece_velocity
        count += ns_per_robot[i]

        # Add the robots to the observation
        if i == custom_cooperation_factor_index:
            obser.add_robot(position, radius, -position, i, simulation_cycle, cooperation_factor=cooperation_factor)
        else:
            obser.add_robot(position, radius, -position, i, simulation_cycle)

    # Set the initial positions in the environment
    ob = env.reset(pos=initial_positions)

    # Initialize the static obstacle
    if obstacle_run:
        obser.add_static(np.array(obstacle_location), obstacle_radius)
        env.add_obstacle(SphereObstacle(name="simpleSphere", content_dict=obst1Dict))

    print(len(obser.obstacles), "lengte")
    print(f"Initial observation : {ob}")

    history = [ob]
    new_velocities = None
    sim_time = 0

    # Simulation loop
    for step in range(n_steps):
        # Check if all the robots reached there goal
        if check_for_finished(obser.get_obstacles()):
            print("All robots reached the goal, closing in 2 seconds...")
            time.sleep(2)
            break

        # Update variables
        sim_time = step * simulation_cycle
        new_positions = fetch_positions(history[-1])
        obser.update_positions(new_positions, simulation_cycle)

        # Check if orca cycle needs to be performed
        if sim_time % orca_update_cycle == 0:
            new_velocities = obser.orca_cycle()  # Do orca cycle
            obser.update_velocities(new_velocities)  # Update all the velocities

            # Update plots
            obser.update_orca_plot()
            obser.update_position_plot()

        # Check which robots are following orca velocities
        for i, obstacle in enumerate(obser.obstacles):
            if isinstance(obstacle, Robot):  # Check if obstacle is an instance of Robot
                if not obstacle.follow_orca:  # Check if obstacle is following orca cycle
                    new_velocities[i] = obstacle.Vref  # Update the velocities

        # Define the new actions
        action = np.array(new_velocities).reshape(action.shape[0])

        # Update the simulation environment
        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()

    return history, obser, sim_time


# Main function
if __name__ == "__main__":
    # List containing the combinations that are simulated
    robots_amount = [3, 5]
    taus = [1, 2]

    # Loop over every tau and number of robots combination
    for robot_amount in robots_amount:
        data = np.zeros((len(taus), 7))
        for i, tau in enumerate(taus):
            # Initialize obeservation
            observation = Observation(tau, simulation_cycle)

            # Run the simulation
            hist, obser, sim_time = run_point_robot(n_steps=steps, render=True, obser=observation, robot_amount=robot_amount, tau=tau)

            # Calculate the simulation metrics
            distances = np.zeros(robot_amount)
            t_max = 0
            no_solution_count = 0
            collision_count = 0
            for j, obstacle in enumerate(obser.obstacles):
                if isinstance(obstacle, Robot):
                    distances[j] = obstacle.travelled_distance

                    if obstacle.no_solution_flag == True:
                        no_solution_count += 1

                    if obstacle.collision_flag == True:
                        collision_count += 1

                    if obstacle.total_time > t_max:
                        t_max = obstacle.total_time

            data[i] = np.array([tau, np.average(distances), circle_radius * 2, np.std(distances), sim_time, collision_count, no_solution_count])

        df = pd.DataFrame(data)

        # Create output directory if none exists yet
        outdir = './data'
        if not os.path.exists(outdir):
            os.mkdir(outdir)
        outname = "robot_amount" + str(robot_amount) + ".csv"
        fullname = os.path.join(outdir, outname)

        # Write to file
        df.to_csv(fullname, index=False, header=["Tau", "Avg distance", "Optimal path", "Std distance", "Sim time", "Collision count", "No solutions count"])
