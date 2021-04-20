import numpy as np
from rectangle_fitting import LShapeFitting
l_shape_fitting = LShapeFitting()


# Fit rectangle to collection of points
def fit_rect(segment):
	rect = l_shape_fitting._rectangle_search(segment[0], segment[1])
	rect.calc_rect_contour()
	return [rect.rect_c_x[:4], rect.rect_c_y[:4]]


# Distance between (x1, y1) and (x2, y2)
def dist(x1, y1, x2, y2):
	return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


# Get index of rectangle corner closest to a point
def get_closest_corner_index(rect, point):
	min_index = 0
	min_dist = dist(rect[0][0], rect[1][0], point[0], point[1])
	for i in range(1, 4):
		d = dist(rect[0][i], rect[1][i], point[0], point[1])
		if d < min_dist:
			min_dist = d
			min_index = i

	return min_index


# Closeness of two rectangles via their center distance
def rect_closeness(rect1, rect2):
	rect1 = sum(rect1[0]) / 4, sum(rect1[1]) / 4
	rect2 = sum(rect2[0]) / 4, sum(rect2[1]) / 4
	return dist(rect1[0], rect1[1], rect2[0], rect2[1])


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
