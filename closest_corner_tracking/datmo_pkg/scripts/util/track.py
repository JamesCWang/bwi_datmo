from kalman import KalmanFilter
from rect import get_closest_corner_index
from scipy import stats


class Track:
	def __init__(self, t, rect, closest_corner_index):
		self.initial_rects = [[t], [rect]]
		self.rect = rect
		self.closest_corner_index = closest_corner_index
		self.km = KalmanFilter(t, rect[0][closest_corner_index], rect[1][closest_corner_index], 0, 0)

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
