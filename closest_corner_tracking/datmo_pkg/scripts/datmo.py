#!/usr/bin/env python

# general modules
import copy
import math
import struct
import numpy as np
import array as arr

# general ros modules
import rospy
import rospkg
rospack = rospkg.RosPack()

# ros message modules
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import datmo_pkg.msg

# ros processing modules
import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_py as tf2

# transformation module
import PyKDL

# datmo modules
from util.range_segmentation import range_segmentation
from util.rect import fit_rect, get_closest_corner_index
from util.associate import associate
from util.track import Track


# Converts specified pgm map to a 2d array where [0,0] is lower left of map
class Map:
    def __init__(self, width, height, resolution, origin, free_value, image_path, image_header):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin

        self.obstacles = np.zeros((width, height))
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
        map_x = int(math.floor((x - origin[0]) / resolution))
        map_y = int(math.floor((y - origin[1]) / resolution))

        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if 0 <= map_x+dx < self.width and 0 <= map_y+dy < self.height and self.obstacles[map_x+dx][map_y+dy]:
                    return True

        return False


# http://docs.ros.org/en/diamondback/api/tf2_geometry_msgs/html/tf2__geometry__msgs_8py_source.html
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


# Lidar scan processing
def scanCallback(scan, args):
    map = args['map']
    laser_projector = args['laser_projector']
    tracks = args['tracks']
    publisher = args['publisher']
    tf_buffer = args['tf_buffer']

    # Convert polar coordinates to cartesian coordinates
    cloud_relative = laser_projector.projectLaser(scan)

    # Get transform to map frame
    try:
        tf_transform = tf_buffer.lookup_transform('map', scan.header.frame_id, scan.header.stamp, rospy.Duration(0.2))
    except (tf2.LookupException, tf2.ExtrapolationException) as ex:
        rospy.logwarn(ex)
        return
    transformer = Transformer(tf_transform)

    # Filter out background
    filtered_x, filtered_y = arr.array('d', []), arr.array('d', [])
    for point in pc2.read_points(cloud_relative, skip_nans=True):
        point_global = transformer.transform(point[0], point[1], point[2])
        if not map.is_obstacle(point_global[0], point_global[1], 2):
            filtered_x.append(point[0])
            filtered_y.append(point[1])

    # Extract features from laser scan in fixed coordinates
    next_origin = [0, 0]
    next_segments = range_segmentation(filtered_x, filtered_y, next_origin)
    # can do further processing of next_segments via min and max dot length and then check connectivity

    next_rects = [fit_rect(next_segment) for next_segment in next_segments]
    next_closest_corner_indexes = [get_closest_corner_index(rect, next_origin) for rect in next_rects]
    for next_rect in next_rects:
        for i in range(4):
            corner_fixed = transformer.transform(next_rect[0][i], next_rect[1][i], 0)
            next_rect[0][i], next_rect[1][i] = corner_fixed[0], corner_fixed[1]
    association_count, associated_track_indexes, associated_corner_indexes = associate(scan.header.stamp.to_sec(), tracks, next_rects, next_closest_corner_indexes)

    next_tracks = []
    for i in range(len(next_rects)):
        next_rect = next_rects[i]
        next_closest_corner_index = next_closest_corner_indexes[i]
        associated_track_index = associated_track_indexes[i]
        associated_corner_index = associated_corner_indexes[i]

        # Update some existing track if successfully associated
        if associated_track_index != -1:
            next_track = copy.deepcopy(tracks[associated_track_index])

            # See if the associated corner and associated closest corner match
            associated_track = tracks[associated_track_index]
            associated_closest_corner_index = associated_track.closest_corner_index
            if associated_closest_corner_index != associated_corner_index:
                associated_closest_corner = [associated_track.rect[dim][associated_closest_corner_index] for dim in range(2)]
                associated_corner = [associated_track.rect[dim][associated_corner_index] for dim in range(2)]
                next_track.km.X0[0] += associated_corner[0] - associated_closest_corner[0]
                next_track.km.X0[1] += associated_corner[1] - associated_closest_corner[1]
            next_track.update(scan.header.stamp.to_sec(), next_rect, next_closest_corner_index)  # should i make it as if it didn't move? for split
            next_tracks.append(next_track)

        # Create new track if not close to any existing ones
        else:
            next_track = Track(scan.header.stamp.to_sec(), next_rect, next_closest_corner_index)
            next_tracks.append(next_track)

    # Create tracks and publish them
    track_array_msg = datmo_pkg.msg.TrackArray()
    track_array_msg.header = Header()
    track_array_msg.header.stamp = scan.header.stamp
    track_array_msg.header.frame_id = 'map'
    for next_segment, next_track in zip(next_segments, next_tracks):
        track_msg = datmo_pkg.msg.Track()
        track_msg.x1 = next_track.km.X0[0]
        track_msg.y1 = next_track.km.X0[1]
        furthest_corner_index = (next_track.closest_corner_index + 2) % 4
        track_msg.x2 = next_track.km.X0[0] + next_track.rect[0][furthest_corner_index] - next_track.rect[0][next_track.closest_corner_index]
        track_msg.y2 = next_track.km.X0[1] + next_track.rect[1][furthest_corner_index] - next_track.rect[1][next_track.closest_corner_index]
        track_msg.dx = next_track.km.X0[2]
        track_msg.dy = next_track.km.X0[3]
        track_array_msg.tracks.append(track_msg)
    publisher.publish(track_array_msg)

    # Update tracks
    args['tracks'] = next_tracks


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

    map = Map(width, height, resolution, origin, free_value, rospack.get_path('datmo_pkg') + '/launch/' + image_name, image_header)
    laser_projector = lg.LaserProjection()
    tracks = []
    publisher = rospy.Publisher('datmo', datmo_pkg.msg.TrackArray, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)  # let listener buffer a bit

    args = {'map': map, 'laser_projector': laser_projector, 'tracks': tracks, 'publisher': publisher, 'tf_buffer': tf_buffer}
    sub = rospy.Subscriber('/'+robot+'/scan_filtered', LaserScan, scanCallback, args)
    rospy.spin()
