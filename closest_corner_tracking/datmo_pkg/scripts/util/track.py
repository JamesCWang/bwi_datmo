import math
import numpy as np
from kalman import KalmanFilter
from rect import calc_dist, get_closest_corner_index
from scipy import stats


# Calculate signed angle between v1 and v2
def calc_signed_angle(v1, v2):
	return math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1])


class Track:
	next_id = 0

	def __init__(self, t, rect, closest_corner_index):
		self.id = Track.next_id
		Track.next_id += 1
		self.initial_rects = [[t], [rect]]
		self.rect = rect
		self.closest_corner_index = closest_corner_index
		self.km = KalmanFilter(t, rect[0][closest_corner_index], rect[1][closest_corner_index], 0, 0)

	def calc_rect_params(self):
		closest_corner = [self.rect[dim][self.closest_corner_index] for dim in range(2)]
		furthest_corner = [self.rect[dim][(self.closest_corner_index + 2) % 4] for dim in range(2)]

		rect_center = np.asarray([closest_corner[dim] + furthest_corner[dim] for dim in range(2)])
		counter_clock_corner = np.asarray([self.rect[dim][(self.closest_corner_index - 1) % 4] for dim in range(2)])
		clock_corner = np.asarray([self.rect[dim][(self.closest_corner_index + 1) % 4] for dim in range(2)])
		if calc_signed_angle(closest_corner - rect_center, clock_corner - rect_center) > 0:
			counter_clock_corner, clock_corner = clock_corner, counter_clock_corner

		x = self.km.X0[0] + (furthest_corner[0] - closest_corner[0])/2
		y = self.km.X0[1] + (furthest_corner[1] - closest_corner[1])/2
		l = calc_dist(closest_corner, counter_clock_corner)
		w = calc_dist(closest_corner, clock_corner)
		theta = math.atan2(self.km.X0[1]-y, self.km.X0[0]-x) - math.atan2(-w, -l)
		return x, y, l, w, theta

	def update(self, t, rect, closest_corner_index):
		self.rect = rect
		self.closest_corner_index = closest_corner_index

		# See if this track is fresh
		if len(self.initial_rects[0]) < 5:
			self.initial_rects[0].append(t)
			self.initial_rects[1].append(rect)

			# See if we can guess its velocity
			if len(self.initial_rects[0]) == 5:
				associated_corners = [[rect[0][closest_corner_index]], [rect[1][closest_corner_index]]]
				for i in range(3, -1, -1):
					associated_rect = self.initial_rects[1][i]
					prev_associated_corner = [associated_corners[0][0], associated_corners[1][0]]
					associated_corner_index = get_closest_corner_index(associated_rect, prev_associated_corner)
					associated_corners[0].insert(0, associated_rect[0][associated_corner_index])
					associated_corners[1].insert(0, associated_rect[1][associated_corner_index])

				dx = stats.linregress(self.initial_rects[0], associated_corners[0]).slope
				dy = stats.linregress(self.initial_rects[0], associated_corners[1]).slope
				self.km = KalmanFilter(t, rect[0][closest_corner_index], rect[1][closest_corner_index], dx, dy)
			else:
				self.km.update(t, rect[0][closest_corner_index], rect[1][closest_corner_index])
		else:
			self.km.update(t, rect[0][closest_corner_index], rect[1][closest_corner_index])
