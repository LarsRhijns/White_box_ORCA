import numpy as np
import matplotlib.pyplot as plt

# Static variables
Vmax = 1
accel = 5
t = 0
T = 20

def euc_dist(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_vref(position, current_velocity, goal, dt):
    # Euclidean distance
    distance_to_goal = euc_dist(position, goal)
    # print("Dist to goal: ", distance_to_goal)

    magnitude_Vcur = np.sqrt(np.sum(np.power(current_velocity, 2)))
    # print("Magnitude Vcur: ", magnitude_Vcur)


    accel_distance = (0.5 * magnitude_Vcur**2 / accel)
    # print("Accel_dist: ", accel_distance)

    vector_to_goal = np.subtract(goal, position)
    norm_vector_to_goal = vector_to_goal / np.linalg.norm(vector_to_goal)
    # print("Norm vec to goal: ", norm_vector_to_goal)

    if (distance_to_goal <= accel_distance):
        # print("Decelerating...")
        vnew = magnitude_Vcur - accel * dt
        return norm_vector_to_goal * vnew
    elif (magnitude_Vcur >= Vmax):
        # print("Max speed...")
        return norm_vector_to_goal * Vmax
    elif (distance_to_goal > accel_distance):
        # print("Accelerating...")
        vnew = Vmax

        return norm_vector_to_goal * vnew


# Example main for testing purposes
if __name__ == "__main__":
    initial_position = np.array([0, 0])
    goal_position = np.array([20, 20])
    position = np.zeros(2)
    Vcur = np.zeros(2)
    dt = 0.001

    positions = []
    velocities = []

    positions.append(initial_position)
    velocities.append(0)

    for i in range(0, int(T / dt)):
        Vref = calculate_vref(position, Vcur, goal_position, dt)
        Vcur = Vref
        position += Vcur * dt

        positions.append(position)

        magnitude_Vcur = np.sqrt(np.sum(np.power(Vcur, 2)))
        # print("Mag: ", magnitude_Vcur)
        velocities.append(magnitude_Vcur)

        # print("Pos: ", position)
        # print("Vel: ", Vcur)
        # print("\n")

        if (euc_dist(position, goal_position) < 0.001) and (euc_dist(Vcur, np.zeros(2)) < 0.001):
            print("Final velocity: ", Vcur)
            print("Final position: ", position)
            print("Got there in {time:.3f} seconds".format(time=t))
            break

        t += dt

    positions = np.array(positions)
    velocities = np.array(velocities)

    plt.plot(positions[:, 0], positions[:, 1], 'b')
    time_lin = np.arange(0, velocities.shape[0]*dt - 0.9 * dt, dt)
    plt.plot(time_lin, velocities, 'r')
    plt.legend(["Position", "Velocity"])
    plt.xlabel("Time for velocity, x-coordinate for position")
    plt.show()

