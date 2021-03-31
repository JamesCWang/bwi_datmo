from rectangle_fitting import LShapeFitting
l_shape_fitting = LShapeFitting()

def extract_feature(segment):
	rect = l_shape_fitting._rectangle_search(segment[0], segment[1])
	rect.calc_rect_contour()
	return sum(rect.rect_c_x[:4])/4, sum(rect.rect_c_y[:4])/4
	#return np.mean(segment[0]), np.mean(segment[1])


def dist(x1, y1, x2, y2):
	return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def closeness(feature1, feature2):
	return dist(feature1[0], feature1[1], feature2[0], feature2[1])


def closest(features, next_feature):
	if len(features) == 0:
		return -1, np.NaN

	min_index = 0
	min_closeness = closeness(features[0], next_feature)
	for i in range(len(features)):
		if closeness(features[i], next_feature) < min_closeness:
			min_index = i
			min_closeness = closeness(features[i], next_feature)

	return min_index, min_closeness