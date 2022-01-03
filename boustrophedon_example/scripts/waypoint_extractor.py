#!/usr/bin/env python
import rospy
import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import visualization_msgs.msg
import yaml

from actionlib import SimpleActionClient
from geometry_msgs.msg import PolygonStamped, Point, Point32, Pose, PoseStamped
from boustrophedon_msgs.msg import PlanMowingPathAction, PlanMowingPathActionGoal, PlanMowingPathActionResult
from boustrophedon_msgs.msg import PlanParameters, PlanMowingPathParamAction, PlanMowingPathParamActionGoal, PlanMowingPathParamActionResult
from boustrophedon_msgs.srv import ConvertPlanToPath
from math import sqrt, pow, atan2, sin, cos, pi
from nav_msgs.msg import *
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler as qfe
from visualization_msgs.msg import Marker, MarkerArray

output_file = 'pure_pursuit.csv'
# map_file = 'helipad-L.yaml'
map_file = 'local_map.yaml'
file_dir = os.path.dirname(os.path.abspath(__file__))
file_full_path = file_dir + '/waypoints/' + output_file
maps_path = file_dir + '/../cutting/mowing/maps/' + map_file

br = tf2_ros.TransformBroadcaster()
count = 0

path_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.MarkerArray, queue_size=10)
points = []
types = []


# dummy publisher only. should not be needed when integrated
def publish_tf():
    global br
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # rospy.loginfo("robot tf timestamp:" + str(t.header.stamp))
    br.sendTransform(t)


# should be relative to map
def get_robot_pose():
    t = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), timeout=rospy.Duration(5, 0))
    robot_pose = Pose()
    robot_pose.position.x = t.transform.translation.x
    robot_pose.position.y = t.transform.translation.y
    robot_pose.position.z = t.transform.translation.z
    robot_pose.orientation.x = t.transform.rotation.x
    robot_pose.orientation.y = t.transform.rotation.y
    robot_pose.orientation.z = t.transform.rotation.z
    robot_pose.orientation.w = t.transform.rotation.w

    return robot_pose


# just a helper to boustrophedon planner's internal converter. Unused
def plan_to_path(plan):
    rospy.wait_for_service('convert_striping_plan_to_path')
    try:
        convertToNavPath = rospy.ServiceProxy('convert_striping_plan_to_path', ConvertPlanToPath)
        resp = convertToNavPath(plan)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


# read waypoints csv file
def read_points_from_file(file_path):
    with open(file_path) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    print path_points
    return path_points


# helper to load from yaml file
def load_yaml_file():
    with open(maps_path, 'r') as f:
        field_polygon = yaml.safe_load(f)
    return field_polygon


# read loaded yaml data
def read_field_file(field_yaml):
    # Parse out the points and return resulting Polygon

    print "reading from", field_yaml
    polygon_points = []
    point_count = 0
    for point in field_yaml:
        point_count += 1
        if point['fix_type'] < 3:
            rospy.logwarn('Point %i has a low quality fix type of %i'
                            % (point_count, point['fix_type']))
        (easting, northing) = (point['easting'], point['northing'])
        polygon_points.append((float(easting), float(northing)))

    print "input poly =", polygon_points

    return polygon_points

def get_heading(start_point, end_point):

    # Calculate the angle between this waypoint and the next
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]

    # it's not worth calculating an approximation error
    if np.isclose(dy, 0.0):
        dy = 0
    if np.isclose(dx, 0.0):
        dx = 0
    heading = atan2(dy, dx)
    # print "heading calc:", dy, dx, heading

    return heading

def calculate_headings(path):
    """
    Calculates the headings between paths and adds them to the waypoints.
    """
    new_path = []
    lookback_indices = []

    for index in range(int(len(path) - 1)):

        heading = get_heading(path[index], path[index+1])

        # sometimes two consecutive points are the same but different point types
        # this case yields a yaw of zero so we'll change the yaw on the next non-zero
        if np.isclose(heading, 0.0):
            lookback_indices.append(index)
        else:
            for back_index in lookback_indices:
                new_path[back_index][2] = heading
                lookback_indices = []

        new_path.append(list(path[index])+[heading])

    heading = get_heading(path[-2], path[-1])
    new_path.append(list(path[-1])+[heading])

    return new_path

def clean_dead_end_paths(path, types):
    """
    Compares headings between each point. If the next point causes a 180 turnaround,
    removes that next point and adjusts the current point to the next [different] heading in the
    resulting path.
    """
    new_path = []
    new_types = []
    skip_next = False
    lookback_index = 0

    for index in range(int(len(path) - 1)):
        if skip_next: # the current index is skipped
            # keep skipping if the next point is still causing a turnaround
            if np.isclose(path[index][2], path[index+1][2]):
                lookback_index = lookback_index + 1
            else: # we can add the next valid point
                new_heading = get_heading(path[index-lookback_index], path[index+1])
                new_path.append(list(path[index-lookback_index][:2]) + [new_heading])
                skip_next = False
            continue

        heading_diff = path[index+1][2] - path[index][2]

        # print "index:", index, "got angle of", heading_diff * 180 / pi

        if np.isclose(heading_diff % pi, 0) and not np.isclose(heading_diff, 0):
            # print "at index:", index, "found a turnaround"
            skip_next = True
            lookback_index = 1
            # traceback all colinear points leading to the current one
            for reverse_idx in range(index - 1, 0, -1):
                if np.isclose(path[index][2], path[reverse_idx][2]):
                    lookback_index = lookback_index + 1
                    new_path.pop()
                else:
                    break
        else:
            # if next point is same or is not a turnaround, should be left as-is
            new_path.append(path[index])
            lookback_index = 0

        new_types.append(types[index])

    # print(new_path)

    return (new_path, new_types)


def clean_colinear_points_in_path(path):

    index_blacklist = []
    new_path = []

    new_path.append(path[0])

    for index in range(int(len(path) - 2)):
        # checks [index + 1] point if we need it in the path. This relocates points into the next one
        # until the check detects that the two next points don't line up with each other
        if path[index] == path[index + 1]:
            print index, " == ", index + 1, ":", path[index]
            s = set(index_blacklist)
            if index in s:
                index_blacklist.append(index + 1)
                print index + 1, "also added for rewriting"
            else:
                new_path.append(path[index+1])
            continue
        if point_is_within_previous_pathline(path[index+2], path[index], path[index+1]):
            print index + 1, "needs rewriting, point:", path[index+1]
            # new_path.append(path[index+2])
            index_blacklist.append(index + 1)
        else:
            for back_index in index_blacklist:
                new_path.append(path[index+1])
                index_blacklist = []
            new_path.append(path[index+1])

    new_path.append(path[-1])
    return new_path


def point_is_within_previous_pathline(point_to_check, point1, point2):
    # two-point form check, if the point_to_check falls on the line, then the equation is satisfied
    # (y - y1) = (dy / dx)(x - x1)
    return (point_to_check[1] - point1[1]) == ( (point2[1] - point1[1]) / (point2[0] - point1[0]) ) * (point_to_check[0] - point1[0])


def read_point_dict(point_info):
    (easting, northing) = (point_info['easting'], point_info['northing'])
    return (float(easting), float(northing))

def fetch_params():
    loaded_params = PlanParameters()
    if rospy.has_param('/boustrophedon_server/repeat_boundary'):
        loaded_params.repeat_boundary = rospy.get_param('/boustrophedon_server/repeat_boundary')
    if rospy.has_param('/boustrophedon_server/outline_clockwise'):
        loaded_params.outline_clockwise = rospy.get_param('/boustrophedon_server/outline_clockwise')
    if rospy.has_param('/boustrophedon_server/skip_outlines'):
        loaded_params.skip_outlines = rospy.get_param('/boustrophedon_server/skip_outlines')
    if rospy.has_param('/boustrophedon_server/outline_layer_count'):
        loaded_params.outline_layer_count = rospy.get_param('/boustrophedon_server/outline_layer_count')
    # note name diff: cut_spacing <-> stripe_separation
    if rospy.has_param('/boustrophedon_server/stripe_separation'):
        loaded_params.cut_spacing = rospy.get_param('/boustrophedon_server/stripe_separation')
    if rospy.has_param('/boustrophedon_server/intermediary_separation'):
        loaded_params.intermediary_separation = rospy.get_param('/boustrophedon_server/intermediary_separation')
    # note name diff: cut_angle_degrees <-> stripe_angle
    if rospy.has_param('/boustrophedon_server/stripe_angle'):
        loaded_params.cut_angle_degrees = rospy.get_param('/boustrophedon_server/stripe_angle')
    if rospy.has_param('/boustrophedon_server/enable_stripe_angle_orientation'):
        loaded_params.enable_stripe_angle_orientation = rospy.get_param('/boustrophedon_server/enable_stripe_angle_orientation')
    if rospy.has_param('/boustrophedon_server/travel_along_boundary'):
        loaded_params.travel_along_boundary = rospy.get_param('/boustrophedon_server/travel_along_boundary')
    if rospy.has_param('/boustrophedon_server/allow_points_outside_boundary'):
        loaded_params.allow_points_outside_boundary = rospy.get_param('/boustrophedon_server/allow_points_outside_boundary')
    if rospy.has_param('/boustrophedon_server/stripes_before_outlines'):
        loaded_params.stripes_before_outlines = rospy.get_param('/boustrophedon_server/stripes_before_outlines')
    if rospy.has_param('/boustrophedon_server/points_per_turn'):
        loaded_params.points_per_turn = rospy.get_param('/boustrophedon_server/points_per_turn')
    if rospy.has_param('/boustrophedon_server/turn_start_offset'):
        loaded_params.turn_start_offset = rospy.get_param('/boustrophedon_server/turn_start_offset')

    # selecting the right turn type based on params:
    if rospy.has_param('/boustrophedon_server/enable_full_u_turns'):
        u_turn_enabled = rospy.get_param('/boustrophedon_server/enable_full_u_turns')
    if rospy.has_param('/boustrophedon_server/enable_half_y_turns'):
        half_y_enabled = rospy.get_param('/boustrophedon_server/enable_half_y_turns')

    if u_turn_enabled:
        loaded_params.turn_type = PlanParameters.TURN_FULL_U
    elif half_y_enabled:
        loaded_params.turn_type = PlanParameters.TURN_HALF_Y
    else:
        loaded_params.turn_type = PlanParameters.TURN_BOUNDARY

    return loaded_params


def populate_plan_path_input(polygon):

    polygon_pub = PolygonStamped()
    for point in polygon:
        polygon_pub.polygon.points.append(Point32(x=point[0], y=point[1], z=0.0))
    pub_node = PlanMowingPathActionGoal()

    # EQ: fix the start to first point of poly
    robot_pose = PoseStamped()
    robot_pose.pose.position.x = polygon[0][0]
    robot_pose.pose.position.y = polygon[0][1]
    robot_pose.pose.orientation.w = 1.0

    pub_node.goal.property.polygon = polygon_pub.polygon
    pub_node.goal.property.header.frame_id = "map"
    pub_node.goal.property.header.stamp = rospy.Time.now()
    pub_node.goal.robot_position.pose = robot_pose.pose
    pub_node.goal.robot_position.header.frame_id = "map"
    pub_node.goal.property.header.stamp = rospy.Time.now()

    params = fetch_params()
    # pub_node.goal.parameters = params
    param_pub.publish(params)
    rospy.loginfo("Published angle: " + str(params.cut_angle_degrees * 180 / pi ))

    return pub_node

def send_polygon(polygon, robot_pose):
    pub_node = populate_plan_path_input(polygon)
    pub.publish(pub_node)
    print "sent to coverage planner"


def getStraightPath(start_x, start_y, goal_x, goal_y, start_or_z, start_or_w, goal_or_z, goal_or_w, dist):
    dt = 0.1
    x = start_x
    y = start_y
    waypoints = []
    yaw = getYaw(start_or_z, start_or_w)
    yaw = yaw
    waypoints.append((x, y, yaw))
    while(distance(x,y, start_x,start_y) <= dist):
        x = x + (dt/dist)*(goal_x - start_x)
        y = y + (dt/dist)*(goal_y - start_y)
        waypoints.append((x, y, yaw))
    waypoints.pop()
    return waypoints


def distance(start_x, start_y, end_x, end_y):
    return sqrt((pow((end_x - start_x), 2) + pow((end_y - start_y), 2)))


def add_intermediaries(points, types, small_dist):
    if points:
        x = points[0]
        y = points[1]
        yaw = points[2]
        new_points = []
        new_types = []
        for index in range(int(len(points) - 1)):
            start_x = points[index][0]
            start_y = points[index][1]
            yaw = points[index][2]
            end_x = points[index+1][0]
            end_y = points[index+1][1]
            inter_x = start_x
            inter_y = start_y

            new_points.append((start_x, start_y, yaw))
            new_types.append(types[index])

            segment_len = distance(start_x, start_y, end_x, end_y)
            if segment_len < small_dist:
                continue

            dist = distance(inter_x, inter_y, start_x, start_y)
            while dist <= (segment_len - small_dist):
                inter_x = inter_x + (small_dist / segment_len) * (end_x - start_x)
                inter_y = inter_y + (small_dist / segment_len) * (end_y - start_y)
                dist = distance(inter_x, inter_y, start_x, start_y)

                new_points.append((inter_x, inter_y, yaw))
                new_types.append(3)

    return (new_points, new_types)


def parse_striping_points(striping_points):
    """
    Checks the path produced and also does some cleaning and adding waypoint headings.
    """
    points = []
    types = []
    for i in striping_points:
            points.append((i.point.x, i.point.y))
            types.append(i.type)

    # print len(points), len(types)
    # for idx, i in enumerate(points):
    #     print i[0], i[1], types[idx]

    # points = clean_colinear_points_in_path(points)
    points = calculate_headings(points)
    (points, types) = clean_dead_end_paths(points, types)

    # Enable this to make intermediate points along the length of the paths
    # (points, types) = add_intermediaries(points, types, 0.3)

    # for idx, i in enumerate(points):
    #     print i[0], i[1], (types[idx] * 180 / pi), i[2]
    save_to_csv(points, types)

    # for idx, i in enumerate(points):
    #     print i[0], i[1], types[idx], i[2]

    # print len(points), len(types)

    #
    # Check output
    #

    return (points, types)


def save_to_csv(points, types=[]):
    if not len(types) == len(points):
        types = [0] * len(points)
    if not os.path.isdir(file_dir + '/waypoints/'):
        os.makedirs(file_dir + '/waypoints/')
    file = open(file_full_path, 'w')
    for index, point in enumerate(points):
        # print point[0], point[1], point[2], types[index]
        zero_point_approach = False
        if types[index] == 2 or types[index] == 0:
            zero_point_approach = True
        file.write('%f, %f, %f, %f\n' % (point[0], point[1], point[2], zero_point_approach))
    file.close()


def send_polygon_blocking(polygon, robot_pose):
    pub_node = populate_plan_path_input(polygon)

    boustro_planner = SimpleActionClient('plan_path', PlanMowingPathAction)
    boustro_planner.wait_for_server()
    boustro_planner.send_goal(pub_node.goal)
    boustro_planner.wait_for_result()

    result = boustro_planner.get_result()
    while (len(result.plan.points) == 0):
        result = boustro_planner.get_result()

    return parse_striping_points(result.plan.points)


def result_callback(data):
    global count, points, types
    if count == 0:

        points, types = parse_striping_points(data.result.plan.points)
        count += 1

        print("Done!!! Press 'ctrl+c'")


def visualize_polygon(polygon):
    plt.subplot(1, 2, 1)
    line_poly = plt.Polygon([(x[1], x[0]) for x in polygon], fill=None, edgecolor='r')
    plt.gca().add_line(line_poly)
    plt.xlim(min([x[1] for x in polygon]), max([x[1] for x in polygon]))
    plt.ylim(min([x[0] for x in polygon]), max([x[0] for x in polygon]))
    plt.gca().invert_xaxis()
    # plt.show(block=False)


def visually_trace_waypoint(points, types, slow=False):
    x_plt = []
    y_plt = []
    type_plt = []
    plt.clf()
    visualize_polygon(polygon)

    plt.subplot(1, 2, 2)
    plt.style.use('seaborn-darkgrid')
    plt.gca().invert_xaxis()
    line_trace = plt.Polygon([(x[1], x[0]) for x in polygon], fill=None, edgecolor='g')
    plt.gca().add_line(line_trace)

    if (slow):
        plt.show(block=False)

    for index, point in enumerate(points):
        x_plt.append(point[0])
        y_plt.append(point[1])
        type_plt.append(types[index])

        # print "plotting:", point[0], point[1], types[index]

        line_plt = plt.Line2D(y_plt[-2:], x_plt[-2:])

        if types[index] == 2 or types[index] == 0:
            plt.scatter(point[1], point[0], s=(4-types[index])*100, c=types[index])

        # draws the goal pose at the end of each point. only visible when animated
            plt.quiver(point[1], point[0], cos(point[2] + pi / 2.0), sin(point[2] + pi / 2.0))
        plt.gca().add_line(line_plt)
        if slow and not types[index] == 3:
            plt.pause(0.4)

        # toggle to debug a particular point
        # if index == 97: break

    if (not slow):
        # plt.scatter(y_plt, x_plt, s=[(4-x)*100 for x in type_plt], c=type_plt, cmap='rainbow')
        plt.title("direction: " + str(cut_direction) + " deg")
        plt.show(block=False)
        plt.pause(0.5)

def visualize_path_as_marker(path, path_status):
    """
    Publishes visualization Markers to represent the planned path.

    Publishes the path as a series of spheres connected by lines.
    The color of the spheres is set by the path_status parameter,
    which is a list of strings of which the possible values are in
    ['not_visited', 'visiting', 'visited'].
    """
    # Get the time
    now = rospy.Time.now()

    path_markers = MarkerArray()
    line_strip_points = []
    # Create the waypoint markers
    for index, waypoint in enumerate(path):
        waypoint_marker = Marker()
        waypoint_marker.header.stamp = now
        waypoint_marker.header.frame_id = "map"
        waypoint_marker.ns = "waypoints"
        waypoint_marker.id = index
        waypoint_marker.type = Marker.ARROW
        if index == 0:
            waypoint_marker.type = Marker.CUBE
        waypoint_marker.action = Marker.MODIFY
        waypoint_marker.scale.x = 1
        waypoint_marker.scale.y = 1
        waypoint_marker.scale.z = 0.25
        point = Point(waypoint[0], waypoint[1], 0)
        waypoint_marker.pose.position = point
        # Store the point for the line_strip marker
        line_strip_points.append(point)
        # Set the heading of the ARROW
        quat = qfe(0, 0, waypoint[2])
        waypoint_marker.pose.orientation.x = quat[0]
        waypoint_marker.pose.orientation.y = quat[1]
        waypoint_marker.pose.orientation.z = quat[2]
        waypoint_marker.pose.orientation.w = quat[3]
        # Color is based on path_status
        status = path_status[index]
        if status == 'not_visited':
            waypoint_marker.color = ColorRGBA(1,0,0,0.5)
        elif status == 'visiting':
            waypoint_marker.color = ColorRGBA(0,1,0,0.5)
        elif status == 'visited':
            waypoint_marker.color = ColorRGBA(0,0,1,0.5)
        else:
            rospy.err("Invalid path status.")
            waypoint_marker.color = ColorRGBA(1,1,1,0.5)
        # Put this waypoint Marker in the MarkerArray
        path_markers.markers.append(waypoint_marker)
    # Create the line_strip Marker which connects the waypoints
    line_strip = Marker()
    line_strip.header.stamp = now
    line_strip.header.frame_id = "map"
    line_strip.ns = "lines"
    line_strip.id = 0
    line_strip.type = Marker.LINE_STRIP
    line_strip.action = Marker.ADD
    line_strip.scale.x = 0.1
    line_strip.color = ColorRGBA(0,0,1,0.5)
    line_strip.points = line_strip_points
    path_markers.markers.append(line_strip)
    # Publish the marker array
    path_pub.publish(path_markers)

if __name__ == '__main__':
    rospy.init_node('waypoint_extractor')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher("/plan_path/goal", PlanMowingPathActionGoal, queue_size=1, latch=True)
    param_pub = rospy.Publisher("/boustrophedon_server/params", PlanParameters, queue_size=1)
    while (param_pub.get_num_connections() < 1):
        rospy.loginfo_throttle_identical(1.0, "Waiting for 1 connection...")
    rospy.Subscriber("/plan_path/result", PlanMowingPathActionResult, result_callback)
    r = rospy.Rate(10)

    if rospy.has_param('/field_polygon'):
        field_polygon = rospy.get_param("/field_polygon")
    polygon = read_field_file(field_polygon)
    # visualize_polygon(polygon)
    # plt.show()

    # cut_direction = rospy.get_param("~cut_direction", 0.0)
    cut_direction = 16
    rospy.set_param("/boustrophedon_server/stripe_angle", cut_direction * pi / 180.0)

    print "successfully read map. sending polygon.."
    robot_pose = get_robot_pose()
    (points, types) = send_polygon_blocking(polygon, robot_pose)
    # Add True param to trace the path one by one
    visually_trace_waypoint(points, types)

    while not rospy.is_shutdown():
        path_status = ['not_visited']*(len(points))
        visualize_path_as_marker(points, path_status)

        cut_direction = cut_direction + 1.0
        if cut_direction > 179.0:
            cut_direction = 0.
        rospy.set_param("/boustrophedon_server/stripe_angle", cut_direction * pi / 180.0)

        print "successfully read map. sending polygon.."
        robot_pose = get_robot_pose()
        (points, types) = send_polygon_blocking(polygon, robot_pose)
        visually_trace_waypoint(points, types)

        r.sleep()

    # polygon = read_field_file(load_yaml_file())
    # points = read_points_from_file(file_dir + '/waypoints/' + 'backup.csv')
    # points = [(float(x[0]), float(x[1]), float(x[2])) for x in points]
    # print(points)
    # types = [1]*len(points)
    # visually_trace_waypoint(points, types, slow=False)
