#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from numpy import array, vstack, delete, floor
from copy import copy
from sklearn.cluster import MeanShift
from rrt_explorer.msg import PointArray

#Grid value function
def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100

# Callback functions.
mapData = OccupancyGrid()
frontier = []
globalmap = []

def callBack(data, args):
    global frontier, min_dist
    transPoint = args[0].transformPoint(args[1], data)
    x = [array([transPoint.point.x, transPoint.point.y])]
    if len(frontier) > 0:
        frontier = vstack((frontier, x))
    else:
        frontier = x

def mapCallBack(data):
    global mapData
    mapData = data

def globalMapCallBack(data):
    global globalmap, n_robots, namespace_init_count, litraIndx
    if n_robots > 1:
        indx = int(data._connection_header['topic'][litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmap[indx] = data

def node():
    global frontier, mapData, globalmap, n_robots, namespace_init_count, litraIndx
    rospy.init_node('filter', anonymous=False)

    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    sensor_radius = rospy.get_param('~sensor_radius', 0.1)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('~namespace_init_count', 1)
    ratefreq = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param('~global_costmap_topic', '/move_base/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    litraIndx = len(namespace)
    rate = rospy.Rate(ratefreq)

    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)

    for i in range(n_robots):
        globalmap.append(OccupancyGrid())
    
    if litraIndx > 0:
        for i in range(n_robots):
            rospy.Subscriber(namespace + str(i+namespace_init_count) + global_costmap_topic, OccupancyGrid, globalMapCallBack)
    elif litraIndx == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMapCallBack)
    
    while(len(mapData.data) < 1):
        rospy.loginfo("Waiting for map data...")
        rospy.sleep(0.1)
        
    for i in range(n_robots):
        while(len(globalmap[i].data) < 1):
            rospy.loginfo("Waiting for global map data...")
            rospy.sleep(0.1)

    global_frame = "/"+mapData.header.frame_id

    tfListener = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(n_robots):
            tfListener.waitForTransform(global_frame[1:], namespace + str(i+namespace_init_count) + robot_frame, rospy.Time(), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tfListener.waitForTransform(global_frame[1:], robot_frame, rospy.Time(), rospy.Duration(10.0))
    
    rospy.Subscriber(goals_topic, PointStamped, callback=callBack, callback_args=[tfListener, global_frame[1:]])
    frontier_pub = rospy.Publisher('/frontiers', Marker, queue_size=10)
    cetroid_pub = rospy.Publisher('/centroids', Marker, queue_size=10)
    filter_pub = rospy.Publisher('/filtered_frontiers', PointStamped, queue_size=20)

    rospy.loginfo("Starting frontier filter node...")

    while len(frontier) < 1:
        rospy.loginfo("Waiting for frontier data...")
        rospy.sleep(0.05)

    points = Marker()
    points_cluster = Marker()
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
    pnt.z = 0.0

    pointList = []

    points_cluster.header.frame_id = mapData.header.frame_id
    points_cluster.header.stamp = rospy.Time.now()

    points_cluster.ns = "cluster_markers"
    points_cluster.id = 5
    points_cluster.type = Marker.POINTS

    points_cluster.action = Marker.ADD
    points_cluster.pose.orientation.w = 1.0
    points_cluster.scale.x = 0.2
    points_cluster.scale.y = 0.2
    points_cluster.color.r = 0.0/255.0
    points_cluster.color.g = 255.0/255.0
    points_cluster.color.b = 0.0/255.0
    points_cluster.color.a = 1
    points_cluster.lifetime = rospy.Duration()

    tempPoint = PointStamped()
    tempPoint.header.frame_id = mapData.header.frame_id
    tempPoint.header.stamp = rospy.Time.now()
    tempPoint.point.z = 0.0
    arrayPoints = PointArray()
    tempPoints = Point()
    tempPoints.z = 0.0

    # Main loop.
    while not rospy.is_shutdown():
        # Cluster frontiers
        centroids = []
        frontiers = copy(frontier)
        if len(frontiers) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(frontiers)
            centroids = ms.cluster_centers_

        elif len(frontiers) == 1:
            centroids = frontiers
        
        frontiers = copy(centroids)

        # Clearing Frintiers
        z = 0
        while z < len(centroids):
            check = False
            tempPoint.point.x = centroids[z][0]
            tempPoint.point.y = centroids[z][1]

            for i in range(n_robots):
                transPoint = tfListener.transformPoint(globalmap[i].header.frame_id, tempPoint)
                x = array([transPoint.point.x, transPoint.point.y])
                check = (gridValue(globalmap[i], x) > threshold) or check
            if(check or (informationGain(mapData, [centroids[z][0], centroids[z][1]], sensor_radius) < 0.2)):
                centroids = delete(centroids, z, 0)
                z -= 1
            z+=1

        # Publish filtered frontiers
        arrayPoints.points = []
        for i in centroids:
            tempPoints.x = i[0]
            tempPoints.y = i[1]
            arrayPoints.points.append(copy(tempPoints))
        filter_pub.publish(arrayPoints)
        pointList = []
        for j in range(frontiers):
            pnt.x = frontiers[j][0]
            pnt.y = frontiers[j][1]
            pointList.append(copy(pnt))
        points_cluster.points = pointList
        frontier_pub.publish(points)
        cetroid_pub.publish(points_cluster)
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass