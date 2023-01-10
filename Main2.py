import gym
import sys
sys.path.append("gym_envs_urdfs")
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
import numpy as np
from Observation import Observation
import pybullet as pb

orca_update_cycle = 3
simulation_cycle = 0.01
radius = 0.2 # As defined in pointRobot.urdf
robot_amount = 5
circle_radius = 2
velocity = 0.5
total_time = 20
steps = int(total_time // simulation_cycle)
obstacle_radius = 0.5
colors = [[1.0, 0.0, 0.0], # Red
          [0.0, 1.0, 0.0], # Green
          [0.0, 0.0, 1.0], # Blue
          [1.0, 1.0, 0.0], # Yellow
          [0.0, 1.0, 1.0], # Cyan
          [1.0, 0.0, 1.0], # Pink
          [0.0, 0.0, 0.0], # Black
          [0.5, 0.0, 0.0], # Dark red
          [0.0, 0.5, 0.0], # Dark green
          [0.0, 0.0, 0.5], # Dark blue
          [0.5, 0.5, 0.0], # Ugly green
          [0.5, 0.0, 0.5], # Purple
          [0.0, 0.5, 0.5], # Darker cyan
          ]

obst1Dict = {
        "type": "sphere",
        "geometry": {"position": [0, 0.0, 0.0], "radius": obstacle_radius},
    }


def fetch_positions(last_observation):
    positions = []
    for robot in last_observation.keys():
        positions.append(last_observation[robot]["joint_state"]["position"])

    return positions

def gym_plot_velocities(positions, velocities):
    z_offset = [0, 0, 0.1]

    for i, position in enumerate(positions):
        pb.addUserDebugLine(position + z_offset , position + velocities[i] * simulation_cycle + z_offset, lineColorRGB=colors[i], lifeTime=0, lineWidth=5)


def run_point_robot(n_steps=2000, render=False, goal=False, obstacles=False):
    robots = []
    for i in range(robot_amount):
        robots.append(GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"))

    env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)
    # pb.computeViewMatrix(cameraEyePosition=[0, 0, 5.0], cameraTargetPosition=[0.0, 0.0, 0.0], cameraUpVector=[1.0, 0.0, 0.0])
    w, h, vmat,projmat,camup,camfwd,hor,ver,yaw,pitch,dist,target = pb.getDebugVisualizerCamera()
    pb.resetDebugVisualizerCamera(dist, 0, -80, target)
    n = env.n()

    ns_per_robot = env.ns_per_robot()  # DoF per robot
    initial_positions = np.zeros((len(robots), ns_per_robot[0]))
    action = np.zeros(n)
    # mount_positions = np.array([np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))])

    obser = Observation(orca_update_cycle, simulation_cycle)
    count = 0
    for i in range(robot_amount):
        angle = ((2 * np.pi) / robot_amount) * i
        position = np.array([circle_radius * np.cos(angle), circle_radius * np.sin(angle), 0])
        initial_positions[i, :] = position

        referece_velocity = -np.array([velocity * np.cos(angle), velocity * np.sin(angle), 0])
        action[count: count + ns_per_robot[i]] = referece_velocity
        count += ns_per_robot[i]

        obser.add_robot3(position, radius, referece_velocity, -position, i)

    # obser.add_static(np.array([0, 0, 0]), obstacle_radius)

    ob = env.reset(pos=initial_positions)

    # env.add_obstacle(SphereObstacle(name="simpleSphere", content_dict=obst1Dict))

    print(f"Initial observation : {ob}")

    history = [ob]
    new_velocities = None
    for step in range(n_steps):
        current_time = step * simulation_cycle
        print("\n********************* TIME: ", current_time, " *********************")
        new_positions = fetch_positions(history[-1])
        obser.update_positions(new_positions)

        if current_time % orca_update_cycle == 0:
            new_velocities = obser.orca_cycle()
            print(new_velocities)
            obser.update_velocities(new_velocities)
            # obser.update_orca_plot()

        gym_plot_velocities(new_positions, new_velocities)

        action = np.array(new_velocities).reshape(action.shape[0])

        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(n_steps=steps, render=True)