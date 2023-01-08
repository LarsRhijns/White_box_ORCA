import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
from Observation import Observation

orca_update_cycle = 0.5
simulation_cycle = 0.01
radius = 1
robot_amount = 4
circle_radius = 2
velocity = 0.5
total_time = 20
steps = int(total_time // simulation_cycle)

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

    obser = Observation(orca_update_cycle, simulation_cycle)
    obser.add_robot3(np.array([2, 2, 0]), radius, np.array([-velocity, -velocity, 0]), np.array([-2, -2, 0]), 0)
    obser.add_robot3(np.array([-2, -2, 0]), radius, np.array([velocity, velocity, 0]), np.array([2, 2, 0]), 1)
    obser.add_robot3(np.array([-2, 2, 0]), radius, np.array([velocity, -velocity, 0]), np.array([2, -2, 0]), 2)
    obser.add_robot3(np.array([2, -2, 0]), radius, np.array([-velocity, velocity, 0]), np.array([-2, 2, 0]), 3)

    count = 0
    obstacle_list = obser.get_obstacles()
    for i in range(robot_amount):
        robot = obstacle_list[i]
        position = robot.get_position()
        initial_positions[i, :] = position

        action[count: count + ns_per_robot[i]] = robot.get_reference_velocity()
        count += ns_per_robot[i]

    ob = env.reset(pos=initial_positions)
    print(f"Initial observation : {ob}")

    history = []
    for step in range(n_steps):
        current_time = step * simulation_cycle
        new_positions = []

        for i in range(len(obser.get_obstacles())):
            robot = obser.obstacles[i]
            # print(robot.toString())
            new_positions.append(robot.get_position() + robot.get_current_velocity() * simulation_cycle)

        # ob.update_positions(new_positions)

        if current_time % orca_update_cycle == 0:
            new_velocities = obser.orca_cycle()
            # print("New velocities: ", new_velocities)
            obser.update_velocities(new_velocities)
            # obser.update_orca_plot()

        # print("SHAPE: ", np.array(new_velocities).shape)
        # print("ACTION SHAPE: ", action.shape)
        action = np.array(new_velocities).reshape(action.shape[0])



        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(n_steps=steps, render=True)