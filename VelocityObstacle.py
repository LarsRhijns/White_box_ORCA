import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

class VelocityObstacle:
    def __init__(self, reference, robot, dt):
        self.reference = reference
        self.robot = robot

        self.dt = dt
        self.rel_pos = robot.pos - reference.pos
        self.rel_vel = reference.Vcur - robot.Vcur
        self.radius = reference.radius + robot.radius
        self.constraint = np.array([np.NaN, np.NaN, np.NaN])

        # Private class attributes
        self.__angle = np.NaN
        self.__circle_centre = np.zeros(2)
        self.__tangent_rc = np.NaN
        self.__intersection = np.zeros(2)

        self.define_shape() # Defines the shape of the VO
        self.define_constraint() # Calculates the constraints

    def define_shape(self):
        self.__angle = np.arctan2(self.rel_pos[1], self.rel_pos[0])  # Calculate the angle of the relative position vector (circle centre)
        self.__circle_centre = self.rotation(self.rel_pos, -self.__angle)  # Rotate the relative position vector, so it lies on the x-axis

        self.__tangent_rc = abs(self.radius / (np.sqrt(self.__circle_centre[0] ** 2 - self.radius ** 2)))  # Calculate the direction coefficient of a line from the origin tangent to the circle (always positive)
        x_cord = (self.__circle_centre[0] + np.sqrt(abs(self.radius ** 2 + self.__tangent_rc ** 2 * self.radius ** 2 - self.__circle_centre[0] ** 2 * self.__tangent_rc ** 2))) / (1 + self.__tangent_rc ** 2)  # Calculate x coordinate from intersection point between circle and line from the origin
        self.__intersection = np.array([x_cord, self.__tangent_rc * x_cord])  # Define the intersection

    def define_constraint(self):
        rot_rel_vel = self.rotation(self.rel_vel, -self.__angle)
        rot_rel_vel[1] = abs(rot_rel_vel[1])

        # Check if the relative velocity is between the tangent line and the x-axis
        if self.__tangent_rc >= rot_rel_vel[1] / rot_rel_vel[0]:

            # Check if the relative velocity is in the polygon
            if rot_rel_vel[0] >= self.__intersection[0] or np.linalg.norm(rot_rel_vel - self.__circle_centre) <= self.radius:
                angle_tangent_centre = np.arctan2((self.__intersection - self.__circle_centre)[1], (self.__intersection - self.__circle_centre)[0])
                angle_rel_vel_centre = np.arctan2((rot_rel_vel - self.__circle_centre)[1], (rot_rel_vel - self.__circle_centre)[0])

                u = np.zeros(2)

                # Check if the relative velocity in within the semi-circle
                if angle_rel_vel_centre >= angle_tangent_centre:
                    norm = self.radius - np.linalg.norm(rot_rel_vel - self.__circle_centre)
                    u = np.array([np.cos(angle_rel_vel_centre) * norm, np.sin(angle_rel_vel_centre) * norm])
                else:
                    angle_tangent = np.arctan2(self.__tangent_rc, 1)
                    norm = self.rotation(np.array([1, self.__tangent_rc]), angle_tangent)[1]
                    u = np.array([np.cos(angle_rel_vel_centre - np.pi) * norm, np.sin(angle_rel_vel_centre - np.pi) * norm])

                # Check if the y value has been made positive
                if self.rotation(self.rel_vel, -self.__angle)[1] < 0:
                    u[1] = -u[1]

                u = self.rotation(u, self.__angle)

                point = self.reference.Vcur + 0.5 * u
                self.constraint = np.array([-1 / (u[1] / u[0]), point[1] - (-1 / (u[1] / u[0])) * point[0], int(u[1] / abs(u[1]))])

    def define_plotting_shapes(self):
        # Define velocity obstacle polygon
        angle_tangent_centre = np.arctan2((self.__intersection - self.__circle_centre)[1], (self.__intersection - self.__circle_centre)[0])
        angles = np.arange(angle_tangent_centre, 2 * np.pi - angle_tangent_centre, step=np.pi / 180)
        circle_points = np.array([np.cos(angles) * self.radius, np.sin(angles) * self.radius]).T + self.__circle_centre
        polygon_points = np.vstack(([20, self.__tangent_rc * 20], circle_points, [20, -self.__tangent_rc * 20]))

        polygon_points = self.rotation(polygon_points.T, self.__angle).T
        vo_polygon = Polygon(polygon_points)

        # Define constrains
        points = np.array([[-20, np.dot([-20, 1], self.constraint[:2].T)],
                           [20, np.dot([20, 1], self.constraint[:2].T)]])

        polygon_points = []
        polygon_points.append(points[0])
        polygon_points.append(points[1])
        polygon_points.append(polygon_points[1] - np.array([0, 0.5 * self.constraint[-1]]))
        polygon_points.append(polygon_points[0] - np.array([0, 0.5 * self.constraint[-1]]))
        constrain = Polygon(polygon_points)

        return vo_polygon, self.constraint, constrain

    def rotation(self, vector, angle):
        matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return np.dot(matrix, vector)

#
# init_pos = np.array([[0, 0], [4, 6]])
# init_vel = np.array([[1, 1], [-2, -4]])
#
# a = Robot(init_pos[0], init_vel[0], 1)
# b = Robot(init_pos[1], init_vel[1], 1)
#
# obs = VelocityObstacle(a, b)