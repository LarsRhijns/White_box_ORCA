import numpy as np
from scipy.spatial import HalfspaceIntersection, ConvexHull
from scipy.optimize import linprog


class Solve:
    def __init__(self, constraints):
        self.constraints = constraints

    def solve(self, reference_point):
        A = []
        b = []
        for i in range(self.constraints.shape[0]):
            if self.constraints[i, 2] == -1:
                A.append([self.constraints[i, 0], -1])
                if self.constraints[i, 0] < 0:
                    b.append(-1 * (self.constraints[i, 1]))
                elif self.constraints[i, 1] < 0 < self.constraints[i, 0]:
                    b.append(-1 * (self.constraints[i, 1]))
                else:
                    b.append(self.constraints[i, 1])
            else:
                A.append([-1 * self.constraints[i, 0], 1])
                b.append(self.constraints[i, 1])

        A = np.array(A)
        b = np.array(b)

        points = self.calculate_polygon(A, b)
        return points

    def feasible_point(self, A, b):
        # finds the center of the largest sphere fitting in the convex hull
        norm_vector = np.linalg.norm(A, axis=1)
        A_ = np.hstack((A, norm_vector[:, None]))
        c = np.zeros((A.shape[1] + 1,))
        c[-1] = -1
        res = linprog(c, A_ub=A_, b_ub=b[:, None], bounds=(None, None))
        return res.x[:-1]

    def hs_intersection(self, A, b):
        interior_point = self.feasible_point(A, b)
        halfspaces = np.hstack((A, -b[:, None]))
        try:
            hs = HalfspaceIntersection(halfspaces, interior_point)
            return hs
        except:
            import warnings
            warnings.warn("No solution found", Warning)

    def calculate_polygon(self, A, b):
        hs = self.hs_intersection(A, b)
        if hs is not None:
            points = hs.intersections
            hull = ConvexHull(points)
            points = points[hull.vertices]
            return points
        else:
            return None

# lines = np.array([[0.5, -5, -1],
#                   [2, -4, 1],
#                   [4, -8, -1],
#                   [-1, 10, 1]])
#
# solution = free_space(lines)
# print(solution)
