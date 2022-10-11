from math import sqrt


def line_btw(c1, c2):
    """Computes the line between two points

    argument : c1 (tuple of float), c2 (tuple of float)
    return : m, c (float)
    """
    m = (c2[1] - c1[1])/(c2[0] - c1[0])
    c = c2[1] - m*c2[0]

    return m, c


def dist_btw(c1, c2):
    """Computes the distance between two points

    argument: c1 (tuple of float), c2 (tuple of float)
    return: d (float)
    """
    d = sqrt((c2[1] - c1[1])**2 + (c2[0] - c1[0])**2)
    return d


def point_perp(c, l):
    """Computes the perpendicular distance between a line and a point

    argument: c (tuple of float) representing a point
              l (tuple of float) representing a line ( (m,c) )
    return: d (float)
    """
    return abs(1 - l[0] - l[1]) / sqrt(1 + l[0]**2)


def dist_btw_polygon(c, p):
    """Computes the shortest perpendicular distance to a line
    argument: c (tuple of float) representing a point
              p (tuple of tuple of float) representing a polygon
    return: d (float)
    """
    min_dist = 999

    for line in p:
        calculated_dist = point_perp(c, line)
        if calculated_dist < min_dist:
            min_dist = calculated_dist

    return min_dist


def tangent_to_polygon(p):

    pass