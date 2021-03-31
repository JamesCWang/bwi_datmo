# calculate distance between two points
def dist(x1, y1, x2, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5


# segment out lidar scan, origin [0, 0]
def range_segmentation(laser_x, laser_y):
	if len(laser_x) == 0:
		return []

	segments = [[[laser_x[0]], [laser_y[0]]]]
	for i in range(1, len(laser_x)):
		d = dist(laser_x[i-1], laser_y[i-1], laser_x[i], laser_y[i])
		r = dist(0, 0, laser_x[i], laser_y[i])
		if d > 0.2 + (0.05 + 0.02*r):
			segments.append([[], []])

		segments[-1][0].append(laser_x[i])
		segments[-1][1].append(laser_y[i])

	return segments
