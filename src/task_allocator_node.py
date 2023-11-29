import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
#from rrt_explorer.msg import PointArray
import math as m
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal
map = ()
frontiers = []
size = 0
current_pos = ()

def nav_cost(occupancy_grid, current_pos, frontier_point):
    return np.linalg.norm(np.asarray(current_pos) - np.asarray(frontier_point))

def information_gain(occupancy_grid, frontier_point, sensor_radius, cell_dim):
    ig = 0

    sensor_cell_radius = int(sensor_radius//cell_dim + 1)

    for i in range(frontier_point[0] - sensor_cell_radius, frontier_point[0] + sensor_cell_radius):
        for j in range(frontier_point[1] - sensor_cell_radius, frontier_point[1] + sensor_cell_radius):
            if np.linalg.norm(np.asarray([i, j]) - np.asarray(frontier_point))*cell_dim<sensor_radius:
                if occupancy_grid[i][j] == -1:
                    ig += 1
    return ig

def mapCallback(msg):
    global map
    map = msg.data

def frontiersCallback(msg):
    global frontiers
    frontiers = np.asarray(msg.data)

def odomCallback(msg):
    global current_pos
    current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def publish_goal(frontier):
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose.position.x = frontier[0]
    goal.target_pose.pose.position.y = frontier[1]

    client.send_goal(goal)
    print("Sent Goal")
    client.wait_for_result()

def goal():
    global map, frontiers, current_pos

    sensor_radius = 0.1
    alpha = 1
    beta = 1

    rospy.init_node('task_allocator')

    rospy.Subscriber('map', OccupancyGrid, mapCallback)
    while len(map)<1:
        rospy.loginfo("Waititng for map data")
        rospy.sleep(0.1)

    #rospy.Subscriber('frontier', PointArray, frontiersCallback)

    rospy.Subscriber('odom', Odometry, odomCallback)

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal)


    while not rospy.is_shutdown():

        #Get the Occupancy Grid and pre-proccess
        size = int(m.sqrt(len(map)))
        grid = np.resize(np.asarray(map), (size, size))

        optimal_frontier = ()
        least_revenue = m.inf
        for frontier in frontiers:
            nav_cost = nav_cost(grid, current_pos, frontier)
            ig = information_gain(grid, frontier, sensor_radius)

            revenue = alpha*nav_cost + beta*ig

            if revenue < least_revenue:
                least_revenue = revenue
                optimal_frontier = frontier


        frontier = (100, 100)
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = 'map'
        goal.goal.target_pose.header.stamp = rospy.Time.now() 
        goal.goal.target_pose.pose.position.x = frontier[0]
        goal.goal.target_pose.pose.position.y = frontier[1]
        pub.publish(goal)
        rospy.loginfo("Published goal")



if __name__ == '__main__':
    try:
        goal()
    except rospy.ROSInterruptException:
        pass