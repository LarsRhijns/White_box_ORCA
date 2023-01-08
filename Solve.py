import numpy as np
from scipy.spatial import HalfspaceIntersection, ConvexHull
from scipy.optimize import linprog
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

class Solve:
    def __init__(self, constraints):
        self.constraints = constraints

    def solve(self, reference_point):
        # Reshape constraints matrix to Ax + b <= 0 format for scipy halfspace calculation
        A = []
        b = []
        for i in range(self.constraints.shape[0]):
            if self.constraints[i, 2] == -1 and self.constraints[i, 1] != None:
                A.append([self.constraints[i, 0], -1])
                if self.constraints[i, 0] < 0:
                    b.append(-1 * (self.constraints[i, 1]))
                elif self.constraints[i, 1] < 0 and self.constraints[i, 0] > 0:
                    b.append(-1 * (self.constraints[i, 1]))
                elif self.constraints[i, 1] > 0 and self.constraints[i, 0] > 0:
                    b.append(-1 * (self.constraints[i, 1]))
                else:
                    b.append(self.constraints[i, 1])
            elif self.constraints[i, 1] is None and self.constraints[i, 2] == 1:
                A.append([-self.constraints[i, 2], 0])
                b.append(-self.constraints[i, 0])
            elif self.constraints[i, 1] is None and self.constraints[i, 2] == -1:
                A.append([-self.constraints[i, 2], 0])
                b.append(self.constraints[i, 0])
            elif self.constraints[i, 2] == 1 and self.constraints[i, 1] != None:
                A.append([-1 * self.constraints[i, 0], 1])
                b.append(self.constraints[i, 1])

        A = np.array(A)
        b = np.array(b)

        bbox = [(-30, 30), (-30, 30)]
        points, int_point = self.plot_convex_set(A, b, bbox)
        if points is not None:

            poly = Polygon(points)
            point = Point(reference_point[0], reference_point[1])
            p1, p2 = nearest_points(poly, point)
            solution = np.array(list(list(p1.coords)[0]) + [0])

            return solution

    def feasible_point(self, A, b):
        # finds the center of the largest sphere fitting in the convex hull
        norm_vector = np.linalg.norm(A, axis=1)
        A_ = np.hstack((A, norm_vector[:, None]))
        c = np.zeros((A.shape[1] + 1,))
        c[-1] = -1
        res = linprog(c, A_ub=A_, b_ub=b[:, None], bounds=(None, None))
        return res.x[:-1]

    def hs_intersection(self, A, b):
        # Try to calculate halfspace intersection if possible
        interior_point = self.feasible_point(A, b)
        halfspaces = np.hstack((A, -b[:, None]))
        try:
            hs = HalfspaceIntersection(halfspaces, interior_point)
            return hs
        except:
            import warnings
            warnings.warn("No solution found", Warning)

    def add_bbox(self, A, b, xrange, yrange):
        # Determine area near robot so solution isn't inf
        A = np.vstack((A, [
            [-1, 0],
            [1, 0],
            [0, -1],
            [0, 1],
        ]))
        b = np.hstack((b, [-xrange[0], xrange[1], -yrange[0], yrange[1]]))
        return A, b

    def plot_convex_set(self, A, b, bbox):
        # solve and plot just the convex set (no lines for the inequations)
        A_, b_ = self.add_bbox(A, b, *bbox)
        interior_point = self.feasible_point(A_, b_)
        hs = self.hs_intersection(A_, b_)
        if hs is not None:
            points = hs.intersections
            hull = ConvexHull(points)
            points = points[hull.vertices]
            return points, interior_point
        else:
            return None, None

    def calculate_polygon(self, A, b):
        # Calculate feasible polygon points
        hs = self.hs_intersection(A, b)
        if hs is not None:
            points = hs.intersections
            hull = ConvexHull(points)
            points = points[hull.vertices]
            return points
        else:
            return None