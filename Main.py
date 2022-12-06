import gym
from urdfenvs.robots.prius import Prius
import numpy as np


def run_prius(n_steps=5000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([1.5, 0.5])
    pos0 = np.array([-1.0, 0.2, -1.0])
    ob = env.reset(pos=pos0)

    # Add shape rectangular shape to the environment
    env.add_shapes(shape_type="GEOM_BOX", dim=np.array([0.2, 8, 0.5]), mass=0, poses_2d=[[4, -0.1, 0]])
    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
