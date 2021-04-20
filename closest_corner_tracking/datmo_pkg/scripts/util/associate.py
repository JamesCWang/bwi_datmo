from rect import get_closest_rect_index, get_closest_corner_index


# Associate a collection of tracks with rectangles/closest corners at time t
def associate(t, tracks, next_rects, next_closest_corner_indexes):
	next_closest_corners = [[rect[dim][index] for dim in range(2)] for rect, index in zip(next_rects, next_closest_corner_indexes)]
	predicted_rects = []
	for j in range(len(tracks)):
		dt = t - tracks[j].km.t0
		dx = tracks[j].km.X0[2] * dt
		dy = tracks[j].km.X0[3] * dt
		predicted_x = [x+dx for x in tracks[j].rect[0]]
		predicted_y = [y+dy for y in tracks[j].rect[1]]
		predicted_rects.append([predicted_x, predicted_y])

	association_count = [0] * len(predicted_rects)
	associated_track_indexes = []
	associated_corner_indexes = []
	for next_rect, next_closest_corner in zip(next_rects, next_closest_corners):
		associated_track_index, associated_track_dist = get_closest_rect_index(predicted_rects, next_rect)

		# Associate with a track if within certain distance
		if associated_track_dist < 0.2:
			association_count[associated_track_index] += 1
			associated_track_indexes.append(associated_track_index)

			# Find index of which corner we're tracking of the associated rectangle
			associated_predicted_rect = predicted_rects[associated_track_index]
			associated_corner_index = get_closest_corner_index(associated_predicted_rect, next_closest_corner)
			associated_corner_indexes.append(associated_corner_index)
		else:
			associated_track_indexes.append(-1)
			associated_corner_indexes.append(-1)

	return association_count, associated_track_indexes, associated_corner_indexes
