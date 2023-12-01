#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from numpy import array, vstack, delete 
from copy import copy
from sklearn.cluster import MeanShift
from rrt_explorer.msg import PointArray
from helper import gridValue, informationGain


mapData = OccupancyGrid()
frontiers = []
globalmaps = []

# Callbacks
def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x

def mapCallBack(data):
    global mapData
    mapData = data

def globalMap(data):
    global globalmaps, litraIndx, namespace_init_count, n_robots
    if n_robots > 1:
        indx = int(data._connection_header['topic'][litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data


# Nodes
def node():
    global frontiers, mapData, globalmaps, litraIndx, n_robots, namespace_init_count
    rospy.init_node('filter', anonymous=False)

    map_topic = rospy.get_param('~map_topic', 'map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    info_radius = rospy.get_param('~info_radius', 1.0)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    rateHz = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param('~global_costmap_topic', '/move_base/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)


    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rospy.Subscriber(namespace+str(i+namespace_init_count) + global_costmap_topic, OccupancyGrid, globalMap)
    elif len(namespace) == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)

    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)

    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rospy.loginfo('Waiting for the global costmap')
            rospy.sleep(0.1)

    global_frame = "/"+mapData.header.frame_id

    tfLisn = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tfLisn.waitForTransform(global_frame[1:], namespace+str(i+namespace_init_count)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tfLisn.waitForTransform(global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack, callback_args=[tfLisn, global_frame[1:]])
    frontier_pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    centroid_pub = rospy.Publisher('centroids', Marker, queue_size=10)
    filter_pub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

    rospy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers"
    points.id = 0
    points.type = Marker.POINTS
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.scale.x = 0.2
    points.scale.y = 0.2
    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0
    points.color.a = 1
    points.lifetime = rospy.Duration()

    pnt = Point()
    pnt.z = 0

    pointList = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "cluster_markers"
    points_clust.id = 4

    points_clust.type = Marker.POINTS
    points_clust.action = Marker.ADD
    points_clust.pose.orientation.w = 1.0
    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0
    points_clust.color.a = 1
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0
    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

    # Main Loop
    while not rospy.is_shutdown():
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_ #centoid of cluster

        if len(front) == 1:
            centroids = front
        frontiers = copy(centroids)

# Clearing frontiers
        z = 0
        while z < len(centroids):
            cond = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            for i in range(0, n_robots):
                transformedPoint = tfLisn.transformPoint(
                    globalmaps[i].header.frame_id, temppoint)
                x = array([transformedPoint.point.x, transformedPoint.point.y])
                cond = (gridValue(globalmaps[i], x) > threshold) or cond

            if (cond or (informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)) < 0.2):
                centroids = delete(centroids, (z), axis=0)
                z = z-1
            z += 1

# Publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filter_pub.publish(arraypoints)
        pointList = []
        for q in range(0, len(frontiers)):
            pnt.x = frontiers[q][0]
            pnt.y = frontiers[q][1]
            pointList.append(copy(pnt))
        points.points = pointList
        pointList = []
        for q in range(0, len(centroids)):
            pnt.x = centroids[q][0]
            pnt.y = centroids[q][1]
            pointList.append(copy(pnt))
        points_clust.points = pointList
        frontier_pub.publish(points)
        centroid_pub.publish(points_clust)
        rate.sleep()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass