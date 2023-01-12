import gym
import sys
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

sys.path.append("gym_envs_urdfs")
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
import numpy as np
from Observation import Observation
import pybullet as pb
from Robot import Robot
import time
from Static import Static

orca_update_cycle = 4
simulation_cycle = 0.01
obser = Observation(orca_update_cycle, simulation_cycle)
radius = 0.2  # As defined in pointRobot.urdf
robot_amount = 3
circle_radius = 2
total_time = 40

obstacle_radius = 0.5
obstacle_run = True
obstacle_location = [0.0, 0.01, 0.0]
plot_velocities = False
steps = int(total_time // simulation_cycle)

custom_cooperation_factor_index = 0
cooperation_factor = 1

colors = [[1.0, 0.0, 0.0],  # Red
          [0.0, 1.0, 0.0],  # Green
          [0.0, 0.0, 1.0],  # Blue
          [1.0, 1.0, 0.0],  # Yellow
          [0.0, 1.0, 1.0],  # Cyan
          [1.0, 0.0, 1.0],  # Pink
          [0.5, 0.0, 0.0],  # Dark red
          [0.0, 0.5, 0.0],  # Dark green
          [0.0, 0.0, 0.5],  # Dark blue
          [0.5, 0.5, 0.0],  # Ugly green
          [0.5, 0.0, 0.5],  # Purple
          [0.0, 0.5, 0.5],  # Darker cyan
          [0.0, 0.0, 0.0]  # Black
          ]

obst1Dict = {
    "type": "sphere",
    "geometry": {"position": obstacle_location, "radius": obstacle_radius},
}


def fetch_positions(last_observation):
    positions = []
    for robot in last_observation.keys():
        positions.append(last_observation[robot]["joint_state"]["position"])

    return positions


def gym_plot_velocities(positions, velocities):
    for i, position in enumerate(positions):
        origin = position
        origin[2] = 0.1
        end = position + velocities[i] * simulation_cycle
        end[2] = 0.1
        pb.addUserDebugLine(origin, end, lineColorRGB=colors[i], lifeTime=0, lineWidth=5)


def check_for_finished(obstacles):
    all_finished = True
    for robot in obstacles:
        if isinstance(robot, Robot):
            position_difference = abs(np.linalg.norm(robot.get_position() - robot.get_goal()))
            if position_difference > 0.01:
                all_finished = False
                return all_finished
    return all_finished


def update_plots(val):
    index = index_slider.val

    ax1.cla()
    ax2.cla()
    ax3.cla()

    if index < len(obser.vo_plots):
        vo_cycle = obser.vo_plots[index]
        constrain_cycle = obser.constraint_plots[index]
        position_cycle = obser.position_plots[index]

        for vo_plot in vo_cycle:
            if len(vo_plot) != 0:
                ax1.add_patch(vo_plot["VO_polygon"])
                ax1.plot(vo_plot["relative_vel"][0], vo_plot["relative_vel"][1], marker="o", color="black")
                ax1.plot(vo_plot["border_point"][0], vo_plot["border_point"][1], marker="o", color="blue")

        for constrain_plot in constrain_cycle:
            if len(constrain_plot) != 0:
                if "constrain" in constrain_plot:
                    ax2.add_patch(constrain_plot["constrain"])

                    if isinstance(constrain_plot["line"], np.ndarray):
                        ax2.axline((0, constrain_plot["line"][1]), slope=constrain_plot["line"][0], color='black')
                    else:
                        ax2.axvline(constrain_plot["line"], color='black')

                ax2.plot(constrain_plot["Vcur"][0], constrain_plot["Vcur"][1], marker="o", color="yellow")
                ax2.arrow(0, 0, constrain_plot["Vref"][0], constrain_plot["Vref"][1], length_includes_head=True, head_width=0.1, head_length=0.1, fc="black", ec="black")

        for position_plot in position_cycle:
            circle = plt.Circle((position_plot["position"][0], position_plot["position"][1]), position_plot["radius"])
            circle.set(color=position_plot["color"], alpha=0.5)
            ax3.add_patch(circle)

            ax3.arrow(position_plot["position"][0], position_plot["position"][1], position_plot["Vcur"][0], position_plot["Vcur"][1], length_includes_head=True, head_width=0.1, head_length=0.1, fc="green", ec="green", label="V_cur")
            ax3.arrow(position_plot["position"][0], position_plot["position"][1], position_plot["Vref"][0], position_plot["Vref"][1], length_includes_head=True, head_width=0.1, head_length=0.1, fc="black", ec="black", label="V_pref")

    else:
        for i, obstacle in enumerate(obser.obstacles):
            if isinstance(obstacle, Robot):
                for sate in obstacle.states:
                    ax3.plot(sate[0], sate[1], marker="o", color=colors[i], markersize=1, alpha=0.5)

                circle = plt.Circle((obstacle.states[-1][0], obstacle.states[-1][1]), obstacle.radius)
                circle.set(color=colors[obstacle.index], alpha=0.5)
                ax3.add_patch(circle)

    for i, obstacle in enumerate(obser.obstacles):
        if isinstance(obstacle, Static):
            circle = plt.Circle((obstacle.pos[0], obstacle.pos[1]), obstacle.radius)
            circle.set(color="black", alpha=1)
            ax3.add_patch(circle)

    ax1.grid()
    ax2.grid()
    ax3.grid()

    ax1.set_box_aspect(1)
    ax2.set_box_aspect(1)
    ax3.set_box_aspect(1)

    plt.setp(ax1, xlim=[-5, 5], ylim=[-5, 5], xlabel="Velocity x", ylabel="Velocity y", title="Velocity obstacle(s)")
    plt.setp(ax2, xlim=[-2, 2], ylim=[-2, 2], xlabel="Velocity x", ylabel="Velocity y", title="Velocity constraint(s)")
    size = circle_radius + radius * 2
    plt.setp(ax3, xlim=[-size, size], ylim=[-size, size], xlabel="x", ylabel="y", title="Configuration space")


def run_point_robot(n_steps=2000, render=False, goal=False, obstacles=False):
    robots = []
    for i in range(robot_amount):
        robots.append(GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"))

    env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)
    w, h, vmat, projmat, camup, camfwd, hor, ver, yaw, pitch, dist, target = pb.getDebugVisualizerCamera()
    pb.resetDebugVisualizerCamera(dist, 0, -80, target)
    n = env.n()

    ns_per_robot = env.ns_per_robot()  # DoF per other_obstacle
    initial_positions = np.zeros((len(robots), ns_per_robot[0]))
    action = np.zeros(n)

    count = 0
    for i in range(robot_amount):
        angle = ((2 * np.pi) / robot_amount) * i
        position = np.array([circle_radius * np.cos(angle), circle_radius * np.sin(angle), 0])
        initial_positions[i, :] = position

        referece_velocity = np.zeros(3)
        action[count: count + ns_per_robot[i]] = referece_velocity
        count += ns_per_robot[i]

        if i == custom_cooperation_factor_index:
            obser.add_robot(position, radius, -position, i, simulation_cycle, cooperation_factor=cooperation_factor)
        else:
            obser.add_robot(position, radius, -position, i, simulation_cycle)

    ob = env.reset(pos=initial_positions)

    if obstacle_run:
        obser.add_static(np.array(obstacle_location), obstacle_radius)
        env.add_obstacle(SphereObstacle(name="simpleSphere", content_dict=obst1Dict))

    print(len(obser.obstacles), "lengte")
    print(f"Initial observation : {ob}")

    history = [ob]
    new_velocities = None
    for step in range(n_steps):
        if check_for_finished(obser.get_obstacles()):
            print("All robots reached the goal, closing in 2 seconds...")
            time.sleep(2)
            break

        current_time = step * simulation_cycle
        # print("\n********************* TIME: ", current_time, " *********************")
        new_positions = fetch_positions(history[-1])
        obser.update_positions(new_positions, simulation_cycle)

        if current_time % (orca_update_cycle / 8) == 0:
            new_velocities = obser.orca_cycle()
            obser.update_velocities(new_velocities)
            obser.update_orca_plot()
            obser.update_position_plot()

        if plot_velocities:
            gym_plot_velocities(new_positions, new_velocities)

        # If current_obstacle velocity is activated, velocity will be updated per step
        for i, obstacle in enumerate(obser.obstacles):
            if isinstance(obstacle, Robot):  # Check if obstacle is a other_obstacle
                if not obstacle.follow_orca:  # Check if other_obstacle is following orca cycle
                    new_velocities[i] = obstacle.Vref

        action = np.array(new_velocities).reshape(action.shape[0])

        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()

    return history, obser


if __name__ == "__main__":
    hist, obser = run_point_robot(n_steps=steps, render=True, goal=False, obstacles=obstacle_run)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)  # note we must use plt.subplots, not plt.subplot
    # fig3, ax3 = plt.subplots(1, 1)  # note we must use plt.subplots, not plt.subplot

    slider_axis = fig.add_axes([0.25, 0.1, 0.65, 0.03])
    index_slider = Slider(
        ax=slider_axis,
        label='Cycle index',
        valmin=0,
        valmax=len(obser.vo_plots),
        valinit=0,
        valstep=1
    )
    index_slider.on_changed(update_plots)

    plt.show()
