from Obstacle import Obstacle
from Robot import Robot
from Static import Static

class Observation():
    def __init__(self, dt: float):
        self.obstacles: list[Obstacle] = []
        self.plot_index: int = -1 # Initialize to -1 since there are no robots to start with thus triggering an error on purpose when accessed
        self.dt: float = dt

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot1(self, position: tuple, radius: int) -> list:
        robot = Robot(position, radius)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot2(self, position: tuple, radius: int, vel_ref: tuple) -> list:
        robot = Robot(position, radius)
        robot.set_reference_velocity(vel_ref)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a robot to the obstacles
    # Python has no overloading therefore the indexation
    def add_robot3(self, position: tuple, radius: int, vel_ref: tuple, goal: tuple) -> list:
        robot = Robot(position, radius)
        robot.set_reference_velocity(vel_ref)
        robot.set_goal(goal)
        self.obstacles.append(robot)
        return self.obstacles

    # Add a static obstacle to the obstacles
    def add_static(self, position: tuple, radius: int) -> list:
        static = Static(position, radius)
        self.obstacles.append(static)
        return self.obstacles

    # Simple toString method. Could be adapted to provide more info
    def toString(self) -> str:
        str = "dt = {dt}\nObstacles = [".format(dt=self.dt)

        for obstacle in self.obstacles:
            str += obstacle.toString() + ", "

        str = str[:len(str) - 2] # Remove the last ", " from the string
        str += "]"
        return str

    # New_positions is a list with tuples with positions for each robot (and static obstacles??)
    # TODO this now assumes that new_positions is always in the same order as robots in our list.
    def update_all(self, new_positions: list) -> list:
        for i in range(len(self.obstacles)):
            if isinstance(self.obstacles[i], Robot):
                self.obstacles[i].set_position(new_positions[i])

        return self.obstacles

    # Loop over the robots and make them all do an orca cycle
    # TODO this assumes that the obstacle list does not change order for the simulation and this class
    # Returns a list with all the new velocities for the obstacles (as tuples)
    def orca_cycle_all(self) -> list:
        new_velocities = [] # Will be a list with tuples
        for obstacle in self.obstacles:
            if isinstance(obstacle, Robot):
                v_new = obstacle.orca_cycle(self.obstacles, self.dt)
                new_velocities.append(v_new)
            else:
                new_velocities.append((0, 0)) # For a static obstacle of course add (0, 0) since it doesn't move

        return new_velocities

# Example main for testing purposes
if __name__ == "__main__":
    ob = Observation(0.1)
    ob.add_robot2((0, 0), 1, (1, 1))
    ob.add_robot2((0, 1), 1, (1, 1))
    ob.add_robot2((0, 2), 1, (1, 1))
    ob.add_robot2((0, 3), 1, (1, 1))
    ob.add_static((1, 0), 1)
    ob.obstacles[0].set_position((5, 5))
    print(ob.orca_cycle_all())

    