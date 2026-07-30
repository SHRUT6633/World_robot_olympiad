import numpy as np


class BezierCurve:
    @staticmethod
    def quadratic(p0, p1, p2, num=20):
        t = np.linspace(0, 1, num)
        x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
        y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
        return np.column_stack((x, y))

    @staticmethod
    def cubic(p0, p1, p2, p3, num=30):
        t = np.linspace(0, 1, num)
        x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
        y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
        return np.column_stack((x, y))
