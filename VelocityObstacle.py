from matplotlib import patches
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points

class VelocityObstacle:
    def __init__(self, reference, robot, dt):
        self.reference = reference
        self.robot = robot

        self.dt = dt
        self.rel_pos = robot.pos - reference.pos
        self.rel_vel = reference.Vcur - robot.Vcur
        self.radius = (reference.radius + robot.radius + 0.2) / self.dt
        self.constraint = None
        self.polygon_points = None
        self.closest_point = np.NaN

        # Private class attributes
        self.__angle = None
        self.__tangent_rc = None
        self.__intersection = np.zeros(2)

        self.define_shape() # Defines the shape of the VO

    def define_shape(self):
        circle_centre = self.rel_pos / self.dt
        self.__angle = np.arctan2(circle_centre[1], circle_centre[0])  # Calculate the angle of the relative position vector (circle centre)
        circle_centre = self.rotation(circle_centre, -self.__angle)  # Rotate the relative position vector, so it lies on the x-axis

        self.__tangent_rc = abs(self.radius / (np.sqrt(circle_centre[0] ** 2 - self.radius ** 2)))  # Calculate the direction coefficient of a line from the origin tangent to the circle (always positive)
        x_cord = (circle_centre[0] + np.sqrt(abs(self.radius ** 2 + self.__tangent_rc ** 2 * self.radius ** 2 - circle_centre[0] ** 2 * self.__tangent_rc ** 2))) / (1 + self.__tangent_rc ** 2)  # Calculate x coordinate from intersection point between circle and line from the origin
        self.__intersection = np.array([x_cord, self.__tangent_rc * x_cord])  # Define the intersection

        # Define velocity obstacle polygon
        print(self.__intersection)
        print(circle_centre)
        angle_tangent_centre = np.arctan2((self.__intersection - circle_centre)[1], (self.__intersection - circle_centre)[0])
        print(angle_tangent_centre)
        angles = np.arange(angle_tangent_centre, 2 * np.pi - angle_tangent_centre, step=np.pi / 180)
        circle_points = np.array([np.cos(angles) * self.radius, np.sin(angles) * self.radius]).T + circle_centre
        self.polygon_points = np.vstack(([20, self.__tangent_rc * 20], circle_points, [20, -self.__tangent_rc * 20], [20, self.__tangent_rc * 20])).T
        self.polygon_points = self.rotation(self.polygon_points, self.__angle).T

    def get_constraint(self):
        polygon = Polygon(self.polygon_points)
        point = Point(self.rel_vel)

        if polygon.contains(point):
            point1, point2 = nearest_points(polygon.exterior, Point(self.rel_vel))
            self.closest_point = np.array(list(point1.coords)[0])
            u = self.closest_point - self.rel_vel[:2]

            constrain_point = self.reference.get_current_velocity()[:2] + 0.5 * u

            # Check if the solution is a vertical line, in other words a very high direction coaficient:
            if abs(u[1]) < 0.01:
                self.constraint = np.array([constrain_point[0], None, int(u[0] / abs(u[0]))])
            else:
                self.constraint = np.array([-1 / (u[1] / u[0]), constrain_point[1] - (-1 / (u[1] / u[0])) * constrain_point[0], -int(u[1] / abs(u[1]))])

            return self.constraint
        else:
            return None

        # rot_rel_vel = self.rotation(self.rel_vel, -self.__angle)
        # rot_rel_vel[1] = abs(rot_rel_vel[1])
        #
        # # Check if the relative velocity is between the tangent line and the x-axis
        # if self.__tangent_rc >= rot_rel_vel[1] / rot_rel_vel[0]:
        #
        #     # Check if the relative velocity is in the polygon
        #     if rot_rel_vel[0] >= self.__intersection[0] or np.linalg.norm(rot_rel_vel - self.__circle_centre) <= self.radius:
        #         angle_tangent_centre = np.arctan2((self.__intersection - self.__circle_centre)[1], (self.__intersection - self.__circle_centre)[0])
        #         angle_rel_vel_centre = np.arctan2((rot_rel_vel - self.__circle_centre)[1], (rot_rel_vel - self.__circle_centre)[0])
        #
        #         u = np.zeros(2)
        #
        #         # Check if the relative velocity in within the semi-circle
        #         if abs(angle_rel_vel_centre) >= angle_tangent_centre:
        #             norm = self.radius - np.linalg.norm(rot_rel_vel - self.__circle_centre)
        #             u = np.array([np.cos(angle_rel_vel_centre) * norm, np.sin(angle_rel_vel_centre) * norm])
        #         else:
        #             angle_tangent = np.arctan2(self.__tangent_rc, 1)
        #             norm = self.rotation(np.array([1, self.__tangent_rc]), angle_tangent)[1]
        #             u = np.array([np.cos(angle_rel_vel_centre - np.pi) * norm, np.sin(angle_rel_vel_centre - np.pi) * norm])
        #
        #         # Check if the y value has been made positive
        #         if self.rotation(self.rel_vel, -self.__angle)[1] < 0:
        #             u[1] = -u[1]
        #
        #         u = self.rotation(u, self.__angle)
        #
        #         point = self.reference.Vcur + 0.5 * u
        #
        #         # Check if the solution is a vertical line, in other words a very high direction coaficient:
        #         if abs(u[1]) < 0.01:
        #             self.constraint = np.array([point[0], None, int(u[0] / abs(u[0]))])
        #         else:
        #             self.constraint = np.array([-1 / (u[1] / u[0]), point[1] - (-1 / (u[1] / u[0])) * point[0], -int(u[1] / abs(u[1]))])
        #
        #         return self.constraint
        #     else:
        #         return None
        # else:
        #     return None

    def define_plotting_shapes(self):
        vo_polygon = patches.Polygon(self.polygon_points)

        # Define constrains
        polygon_points = []
        if self.constraint[1] == None:  # Check if vertical line
            polygon_points.append(np.array([self.constraint[0], 30]))
            polygon_points.append(np.array([self.constraint[0], -30]))
            polygon_points.append(np.array([self.constraint[0] + 0.5 * self.constraint[2], -30]))
            polygon_points.append(np.array([self.constraint[0] + 0.5 * self.constraint[2], 30]))

        else:
            polygon_points.append(np.array([-30, -30 * self.constraint[0] + self.constraint[1]]))
            polygon_points.append(np.array([30, 30 * self.constraint[0] + self.constraint[1]]))
            polygon_points.append(polygon_points[1] - np.array([0, 0.5 * self.constraint[-1]]))
            polygon_points.append(polygon_points[0] - np.array([0, 0.5 * self.constraint[-1]]))

        constrain_polygon = patches.Polygon(polygon_points)

        return vo_polygon, self.constraint, constrain_polygon, self.closest_point

        # # Define velocity obstacle polygon
        # angle_tangent_centre = np.arctan2((self.__intersection - self.__circle_centre)[1], (self.__intersection - self.__circle_centre)[0])
        # angles = np.arange(angle_tangent_centre, 2 * np.pi - angle_tangent_centre, step=np.pi / 180)
        # circle_points = np.array([np.cos(angles) * self.radius, np.sin(angles) * self.radius]).T + self.__circle_centre
        # polygon_points = np.vstack(([20, self.__tangent_rc * 20], circle_points, [20, -self.__tangent_rc * 20]))
        #
        # polygon_points = self.rotation(polygon_points.T, self.__angle).T
        # vo_polygon = patches.Polygon(polygon_points)
        #
        # # Define constrains
        # polygon_points = []
        # if self.constraint[1] == None: # Check if vertical line
        #     polygon_points.append(np.array([self.constraint[0], 30]))
        #     polygon_points.append(np.array([self.constraint[0], -30]))
        #     polygon_points.append(np.array([self.constraint[0] + 0.5 * self.constraint[2], -30]))
        #     polygon_points.append(np.array([self.constraint[0] + 0.5 * self.constraint[2], 30]))
        #
        # else:
        #     points = np.array([[-20, np.dot([-20, 1], self.constraint[:2].T)],
        #                        [20, np.dot([20, 1], self.constraint[:2].T)]])
        #
        #     polygon_points = []
        #     polygon_points.append(points[0])
        #     polygon_points.append(points[1])
        #     polygon_points.append(polygon_points[1] - np.array([0, 0.5 * self.constraint[-1]]))
        #     polygon_points.append(polygon_points[0] - np.array([0, 0.5 * self.constraint[-1]]))
        # constrain = Polygon(polygon_points)
        #
        # return vo_polygon, self.constraint, constrain, self.rotation(self.__intersection, -self.__angle)

    def rotation(self, vector, angle):
        vector = vector[:2]
        matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return np.dot(matrix, vector)

# points = np.array([[-2, 0], [2, 0], [0, 2], [-2, 0]])
# print(points.shape)
# poly = Polygon(points)
# point = Point(0, 3)
#
# print(poly.contains(point))
#
# p1, p2 = nearest_points(poly.exterior, point)
# print(np.array(list(p1.coords)[0]))