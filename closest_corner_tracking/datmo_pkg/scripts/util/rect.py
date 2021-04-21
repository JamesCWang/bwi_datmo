import numpy as np
from rectangle_fitting import LShapeFitting
l_shape_fitting = LShapeFitting()


# Fit rectangle to collection of points
def fit_rect(segment):
	rect = l_shape_fitting._rectangle_search(segment[0], segment[1])
	rect.calc_rect_contour()
	return [rect.rect_c_x[:4], rect.rect_c_y[:4]]


# Calculate Distance between p1 and p2
def calc_dist(p1, p2):
	return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


# Get index of rectangle corner closest to a point
def get_closest_corner_index(rect, point):
	min_index = 0
	min_dist = calc_dist([rect[0][0], rect[1][0]], point)
	for i in range(1, 4):
		d = calc_dist([rect[0][i], rect[1][i]], point)
		if d < min_dist:
			min_dist = d
			min_index = i

	return min_index


# Closeness of two rectangles via their center distance
def rect_closeness(rect1, rect2):
	center1 = (rect1[0][0]+rect1[0][2])/2, (rect1[1][0]+rect1[1][2])/2
	center2 = (rect2[0][0]+rect2[0][2])/2, (rect2[1][0]+rect2[1][2])/2
	return calc_dist(center1, center2)


# Get index of rectangle closest to a rectangle
def get_closest_rect_index(rects, next_rect):
	if len(rects) == 0:
		return -1, np.NaN

	min_i = 0
	min_c = rect_closeness(rects[0], next_rect)
	for i in range(1, len(rects)):
		c = rect_closeness(rects[i], next_rect)
		if c < min_c:
			min_i = i
			min_c = c

	return min_i, min_c
