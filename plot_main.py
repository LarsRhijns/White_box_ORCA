import matplotlib.pyplot as plt
from Observation import Observation
import numpy as np
from IPython.display import display, clear_output

# Basic 2D simulation. Je kan de simulatie update cycle en orca update cycle aanpassen. Ook kunnen er meer robots toegevoegd of aangepast worden
# Zie de comment onderaan de loop voor het veranderen van het simulatie gedrag
if __name__ == "__main__":
    simulation_cycle = 0.01
    orca_update_cycle = 0.5  # Verander deze waarde naar 1.0 voor een betere simulatie
    ob = Observation(orca_update_cycle, simulation_cycle)

    # Add robots to the simulations
    speed = 5
    radius = 0.2

    ob.add_robot(np.array([2, 2, 0]), radius, np.array([-speed, -speed, 0]), np.array([-2, -2, 0]), 0)
    ob.add_robot(np.array([-2, -2, 0]), radius, np.array([speed, speed, 0]), np.array([2, 2, 0]), 1)
    ob.add_robot(np.array([-2, 2, 0]), radius, np.array([speed, -speed, 0]), np.array([2, -2, 0]), 2)
    ob.add_robot(np.array([2, -2, 0]), radius, np.array([-speed, speed, 0]), np.array([-2, 2, 0]), 3)

    for t in np.arange(0, 5, simulation_cycle):
        new_positions = []

        for i in range(len(ob.obstacles)):
            robot = ob.obstacles[i]
            # print(other_obstacle.toString())
            new_positions.append(robot.get_position() + robot.get_current_velocity() * simulation_cycle)

        ob.update_positions(new_positions)

        if t % orca_update_cycle == 0:
            print("Time: ", t)
            new_velocities = ob.orca_cycle()
            ob.update_velocities(new_velocities)
            ob.update_orca_plot()

        ob.update_position_plot()

        plt.show(block=False)  # TODO: als False veranderd loopt de simulatie door!!!
        clear_output(wait=True)
        plt.pause(0.001)
