import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

robot_amount = 6
circle_radius = 2
velocity = 0.5

def run_point_robot(n_steps=2000, render=False, goal=False, obstacles=False):
    robots = []
    for i in range(robot_amount):
        robots.append(GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"))

    env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)
    n = env.n()

    ns_per_robot = env.ns_per_robot()  # DoF per robot
    initial_positions = np.zeros((len(robots), ns_per_robot[0]))
    action = np.zeros(n)
    # mount_positions = np.array([np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))])

    count = 0
    for i in range(robot_amount):
        angle = ((2 * np.pi) / robot_amount) * i
        position = np.array([circle_radius * np.cos(angle), circle_radius * np.sin(angle), 0])
        initial_positions[i, :] = position

        action[count: count + ns_per_robot[i]] = -np.array([velocity * np.cos(angle), velocity * np.sin(angle), 0])
        count += ns_per_robot[i]

    ob = env.reset(pos=initial_positions)
    print(f"Initial observation : {ob}")

    history = []
    for _ in range(n_steps):
        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(render=True)
