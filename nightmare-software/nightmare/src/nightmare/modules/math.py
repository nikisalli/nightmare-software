import numpy as np
from numpy import sin, cos, arccos, arctan2, sqrt
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from nightmare.config import PI, EPSILON

# ros imports
import rospy
import tf_conversions

# module imports
from nightmare.config import *


def no_zero(a):   # make a value always not zero
    if abs(a) < EPSILON:
        return EPSILON
    else:
        return a


def fmap(x: float, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def asymmetrical_sigmoid(val):
    return 1 / (1 + np.e**(-13 * (val - 0.5)))  # width set to 13


def vmult(matrix, vector):
    return (matrix.T * vector).T


def rotate(pose, rot, pivot=None, inverse=False):
    # check if the argument contains an euler rotation vector or a quaternion
    if len(rot) == 3:  # euler
        r = R.from_rotvec(rot)
    elif len(rot) == 4:  # quaternion
        r = R.from_quat(rot)

    if inverse:
        r = r.inv()

    if pivot is not None:  # check if we have a pivot to rotate around
        p = pose - pivot
        p = r.apply(p)
        return p + pivot
    else:
        return r.apply(pose)


def quat2euler(quat):
    return tf_conversions.transformations.euler_from_quaternion(quat)


def euler2quat(euler):
    return tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])


def limit(val, maxv, minv):
    return min(max(val, minv), maxv)


def line_segment_circle_lintersection(pt1, pt2, circle_center, circle_radius, full_line=True, tangent_tol=1e-9):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:  # If line is tangent to circle, return just one point (as both intersections have same location)
            return [intersections[0]]
        else:
            return intersections


def line_circle_intersection(pt1, pt2, circle_center, circle_radius, tangent_tol=1e-9):
    return line_segment_circle_lintersection(pt1, pt2, circle_center, circle_radius, True, tangent_tol)


def segment_circle_intersection(pt1, pt2, circle_center, circle_radius, tangent_tol=1e-9):
    return line_segment_circle_lintersection(pt1, pt2, circle_center, circle_radius, False, tangent_tol)
