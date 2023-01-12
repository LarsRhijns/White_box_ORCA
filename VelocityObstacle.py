from matplotlib import patches
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points
from Static import Static
from Obstacle import Obstacle
import matplotlib.pyplot as plt


class VelocityObstacle:
    def __init__(self, current_obstacle, other_obstacle, tau):
        self.current_obstacle = current_obstacle
        self.other_obstacle = other_obstacle

        self.tau = tau
        self.rel_pos = (other_obstacle.pos - current_obstacle.pos) / self.tau
        self.rel_vel = current_obstacle.Vref - other_obstacle.Vref
        if isinstance(other_obstacle, Static):
            self.radius = (current_obstacle.radius + other_obstacle.radius + 0.0) / self.tau
        else:
            self.radius = (current_obstacle.radius + other_obstacle.radius + 0.2) / self.tau
        self.constraint = None
        self.polygon_points = None
        self.closest_point = np.NaN

        # Private class attributes
        self.__angle = None
        self.__tangent_rc = None
        self.__intersection = np.zeros(2)

        self.define_shape()  # Defines the shape of the VO

    def define_shape(self):
        circle_centre = self.rel_pos[:2]
        self.__angle = np.arctan2(circle_centre[1], circle_centre[0])  # Calculate the angle of the relative position vector (circle centre)
        circle_centre = self.rotation(circle_centre, -self.__angle)  # Rotate the relative position vector, so it lies on the x-axis

        self.__tangent_rc = abs(self.radius / (np.sqrt(abs(circle_centre[0] ** 2 - self.radius ** 2))))  # Calculate the direction coefficient of a line from the origin tangent to the circle (always positive)
        x_cord = (circle_centre[0] + np.sqrt(abs(self.radius ** 2 + self.__tangent_rc ** 2 * self.radius ** 2 - circle_centre[0] ** 2 * self.__tangent_rc ** 2))) / (1 + self.__tangent_rc ** 2)  # Calculate x coordinate from intersection point between circle and line from the origin
        self.__intersection = np.array([x_cord, self.__tangent_rc * x_cord])  # Define the intersection

        # Define velocity obstacle polygon
        angle_tangent_centre = np.arctan2((self.__intersection - circle_centre)[1], (self.__intersection - circle_centre)[0])
        angles = np.arange(angle_tangent_centre, 2 * np.pi - angle_tangent_centre, step=(np.pi / 180) * 3)
        # angles = np.arange(angle_tangent_centre, -angle_tangent_centre, step=-(np.pi / 700))

        circle_points = np.array([np.cos(angles) * self.radius, np.sin(angles) * self.radius]).T + circle_centre
        polygon_points_rotated = np.vstack(([20, self.__tangent_rc * 20], circle_points, [20, -self.__tangent_rc * 20], [20, self.__tangent_rc * 20]))

        self.polygon_points = self.rotation(polygon_points_rotated.T, self.__angle).T

    def get_constraint(self):
        polygon = Polygon(self.polygon_points)
        point = Point(self.rel_vel[:2])

        point1, point2 = nearest_points(polygon.exterior, Point(self.rel_vel))
        self.closest_point = np.array(list(point1.coords)[0])
        distance = np.linalg.norm(self.rel_vel[:2] - self.closest_point)

        if polygon.contains(point):  # or distance < 0.01:
            # point1, point2 = nearest_points(polygon.exterior, Point(self.rel_vel))
            # self.closest_point = np.array(list(point1.coords)[0])
            u = self.closest_point - self.rel_vel[:2]

            # if distance < 0.01:
            #     u = -u

            if isinstance(self.other_obstacle, Static):
                constraint_point = self.current_obstacle.Vref[:2] + u
            else:
                constraint_point = self.current_obstacle.Vref[:2] + 0.5 * u

            # Check if the solution is a vertical line, in other words a very high direction coefficient:
            if abs(u[1]) < 0.001:
                self.constraint = np.array([constraint_point[0], None, int(u[0] / abs(u[0]))])
            else:
                self.constraint = np.array([-1 / (u[1] / u[0]), constraint_point[1] - (-1 / (u[1] / u[0])) * constraint_point[0], -int(u[1] / abs(u[1]))])

            return self.constraint
        else:
            return None

    def define_plotting_shapes(self):
        vo_polygon = patches.Polygon(self.polygon_points)

        # Define constrains
        polygon_points = []
        if self.constraint[1] == None:  # Check if vertical line
            polygon_points.append(np.array([self.constraint[0], 30]))
            polygon_points.append(np.array([self.constraint[0], -30]))
            polygon_points.append(np.array([self.constraint[0] + 0.1 * self.constraint[2], -30]))
            polygon_points.append(np.array([self.constraint[0] + 0.1 * self.constraint[2], 30]))

        else:
            polygon_points.append(np.array([-30, -30 * self.constraint[0] + self.constraint[1]]))
            polygon_points.append(np.array([30, 30 * self.constraint[0] + self.constraint[1]]))
            polygon_points.append(polygon_points[1] - np.array([0, 0.1 * self.constraint[-1]]))
            polygon_points.append(polygon_points[0] - np.array([0, 0.1 * self.constraint[-1]]))

        constrain_polygon = patches.Polygon(polygon_points)

        return vo_polygon, self.constraint, constrain_polygon, self.closest_point

    def rotation(self, vector, angle):
        vector = vector[:2]
        matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return np.dot(matrix, vector)
