import gym
import sys

sys.path.append("gym_envs_urdfs")
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
from Observation import Observation

robot_amount = None
radius = None
velocity = None
simulation_cycle = None
orca_update_cycle = None  # Verander deze waarde naar 1.0 voor een betere simulatie


def run_point_robot(n_steps=2000, render=False, goal=False, obstacles=False):
    robots = []
    for i in range(robot_amount):
        robots.append(GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"))

    env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)
    n = env.n()

    ns_per_robot = env.ns_per_robot()  # DoF per other_obstacle
    initial_positions = np.zeros((len(robots), ns_per_robot[0]))
    action = np.zeros(n)
    # mount_positions = np.array([np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))])

    ob = Observation(orca_update_cycle, simulation_cycle)
    ob.add_robot(np.array([5, 5]), radius, np.array([-velocity, -velocity]), np.array([-5, -5]), )
    ob.add_robot(np.array([-5, -5]), radius, np.array([velocity, velocity]), np.array([5, 5]), )
    ob.add_robot(np.array([-5, 5]), radius, np.array([velocity, -velocity]), np.array([5, -5]), )
    ob.add_robot(np.array([5, -5]), radius, np.array([-velocity, velocity]), np.array([-5, 5]), )

    count = 0
    for i, robot in enumerate(ob.obstacles):
        position = robot.get_position()
        initial_positions[i, :] = position

        vel = robot.get_reference_velocity()
        action[count: count + ns_per_robot[i]] = np.array([vel[0], vel[1], 0])
        count += ns_per_robot[i]

    ob = env.reset(pos=initial_positions)
    print(f"Initial observation : {ob}")

    history = []
    for _ in range(n_steps):
        gym_ob, _, _, _ = env.step(action)
        history.append(gym_ob)
    env.close()
    return history


if __name__ == "__main__":
    robot_amount = 4
    radius = 1
    velocity = 5
    simulation_cycle = 0.01
    orca_update_cycle = 1.0  # Verander deze waarde naar 1.0 voor een betere simulatie
    total_time = 20
    steps = int(total_time // simulation_cycle)

    run_point_robot(n_steps=steps, render=True)
