#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg

# tarnsform point cloud
import tf2_ros
import tf2_py as tf2
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


# extract points from point cloud
import sensor_msgs.point_cloud2 as pc2
import array as arr

# map reading stuff
import copy
import numpy as np
import math
import struct

# kalman filter
from util.kalman import KalmanFilter

from util.range_segmentation import range_segmentation
from util.association import extract_feature, closest

# message
from std_msgs.msg import Header
from datmo_pkg.msg import TrackArray, Track

# get name of src
import rospkg
rospack = rospkg.RosPack()


class Map:
    def __init__(self, width, height, resolution, origin, free_value, image_path, image_header):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin

        self.obstacles = np.zeros((width, height))  # origin is -13.15, -2, resolution 0.05
        with open(image_path, 'rb') as f:
            for i in range(image_header):
                f.readline()

            i = 0
            j = 0
            while not (i == 0 and j == height):
                c = struct.unpack('>B', f.read(1))[0]
                self.obstacles[i][height - 1 - j] = c != free_value

                i = (i + 1) % width
                if i == 0:
                    j = j + 1

    # x, y in map frame; radius is pixel
    def is_obstacle(self, x, y, radius):
        map_x = int(math.floor((x - origin[0]) / resolution)) # might wanna change to like including origin in constructor
        map_y = int(math.floor((y - origin[1]) / resolution))

        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if 0 <= map_x+dx < self.width and 0 <= map_y+dy < self.height and self.obstacles[map_x+dx][map_y+dy]:
                    return True

        return False


# http://docs.ros.org/en/diamondback/api/tf2_geometry_msgs/html/tf2__geometry__msgs_8py_source.html
import PyKDL
class Transformer:

    # t is a tf transform
    def __init__(self, t):
        self.frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

    def transform(self, x, y, z):
        return self.frame * PyKDL.Vector(x, y, z)


def scanCallback(scan, args):
    map = args['map']
    laser_projector = args['laser_projector']
    kalman_filters = args['kalman_filters']
    features = args['features']
    publisher = args['publisher']
    tf_buffer = args['tf_buffer']

    cloud_relative = laser_projector.projectLaser(scan)

    try:
        tf_transform = tf_buffer.lookup_transform('map', scan.header.frame_id, scan.header.stamp, rospy.Duration(0.2))
    except (tf2.LookupException, tf2.ExtrapolationException) as ex:
        rospy.logwarn(ex)
        return

    #cloud_fixed = do_transform_cloud(cloud_relative, tf_transform)
    transformer = Transformer(tf_transform)

    filtered_x, filtered_y = arr.array('d', []), arr.array('d', [])
    for point in pc2.read_points(cloud_relative, skip_nans=True):
        point_global = transformer.transform(point[0], point[1], point[2])
        if not map.is_obstacle(point_global[0], point_global[1], 2):
            filtered_x.append(point[0])
            filtered_y.append(point[1])


    # make it global
    #global_center = transformer.transform(np.mean(filtered_x), np.mean(filtered_y), 0)
    #x_center, y_center = global_center[0], global_center[1]

    next_segments = range_segmentation(filtered_x, filtered_y)
    next_features = [transformer.transform(*extract_feature(next_segment), 0) for next_segment in next_segments] # GLOBAL

    copied_kalman_filters = [0] * len(kalman_filters)
    for next_feature in next_features:
        closest_index, closest_distance = closest(features, next_feature)
        if closest_distance < 0.2:
            if copied_kalman_filters[closest_index] == 0:
                copied_kalman_filters[closest_index] = 1
                kalman_filter = kalman_filters[closest_index]
            else:
                kalman_filter = copy.deepcopy(kalman_filters[closest_index])
            kalman_filter.update(scan.header.stamp.to_sec(), next_feature[0], next_feature[1])  # should i make it as if it didn't move? for split
            next_kalman_filters.append(kalman_filter)
        else:
            kalman_filter = KalmanFilter(scan.header.stamp.to_sec(), next_feature[0], next_feature[1])
            next_kalman_filters.append(kalman_filter)

    # if len(kalman_filters) == 0:
    #     kalman_filters.append(KalmanFilter(scan.header.stamp.to_sec(), x_center, y_center))
    # else:
    #     kalman_filters[0].update(scan.header.stamp.to_sec(), x_center, y_center)

    #rospy.loginfo(str(kms[0].t0))
    #rospy.loginfo(str(kms[0].X0))
    # can use tf transform but with empty header? also inverse for frames in pykdl

    track_array_msg = TrackArray()

    for i in range(len(next_segments)):
        track_msg = Track()
        track_msg.header = Header()
        track_msg.header.stamp = scan.header.stamp
        track_msg.header.frame_id = 'map'
        track_msg.x = kalman_filters[i].X0[0]
        track_msg.y = kalman_filters[i].X0[1]
        track_msg.dx = kalman_filters[i].X0[2]
        track_msg.dy = kalman_filters[i].X0[3]
        if len(filtered_y) == 0:
            track_msg.width = 0
        else:
            track_msg.width = np.max(next_segments[i][1]) - np.min(next_segments[i][1])  # note that these are upside down
        track_array_msg.tracks.append(track_msg)

    publisher.publish(track_array_msg)

    args['kalman_filters'] = next_kalman_filters
    args['features'] = next_features


if __name__ == '__main__':
    rospy.init_node('datmo_node')

    image_name = rospy.get_param('~image_name')
    image_header = rospy.get_param('~image_header')
    width = rospy.get_param('~width')
    height = rospy.get_param('~height')
    resolution = rospy.get_param('~resolution')
    origin = rospy.get_param('~origin')
    free_value = rospy.get_param('~free_value')
    robot = rospy.get_param('~robot')

    # 3netest.pgm converted to a 2d bool array where [0,0] is lower left
    map = Map(width, height, resolution, origin, free_value, rospack.get_path('datmo_pkg') + '/launch/' + image_name, image_header)
    laser_projector = lg.LaserProjection()
    kalman_filters = []
    features = []
    publisher = rospy.Publisher('datmo', TrackArray, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)  # let listener buffer a bit

    args = {'map': map, 'laser_projector': laser_projector, 'kalman_filters': kalman_filters, 'features': features, 'publisher': publisher, 'tf_buffer': tf_buffer}
    sub = rospy.Subscriber('/'+robot+'/scan_filtered', LaserScan, scanCallback, args)
    rospy.spin()
