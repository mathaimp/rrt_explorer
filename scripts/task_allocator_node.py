#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rrt_explorer.msg import PointArray
from helper import robot,informationGain
import math as m
import actionlib
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
GOAL_TIMEOUT = 30.0
BLACKLIST_RADIUS = 0.5
failed_goals = []
map = ()
map_msg = None
frontiers = []
size = 0
current_pos = ()
goal_status = GoalStatusArray()

def movebase_client(frontier):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = frontier[0]
    goal.target_pose.pose.position.y = frontier[1]
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.logerr("Sent Goal (%.2f, %.2f)" % (frontier[0], frontier[1]))
    finished = client.wait_for_result(rospy.Duration(GOAL_TIMEOUT))
    if not finished:
        client.cancel_goal()
        rospy.logwarn("Goal timed out after %.0fs, cancelled" % GOAL_TIMEOUT)
        return False
    return client.get_state() == GoalStatus.SUCCEEDED

def nav_cost(occupancy_grid, current_pos, frontier_point):
    return np.linalg.norm(np.asarray(current_pos) - np.asarray(frontier_point))




# def information_gain(occupancy_grid, frontier_point, sensor_radius, cell_dim):
#     ig = 0
# 
#     sensor_cell_radius = int(sensor_radius//cell_dim + 1)
# 
#     for i in range(frontier_point[0] - sensor_cell_radius, frontier_point[0] + sensor_cell_radius):
#         for j in range(frontier_point[1] - sensor_cell_radius, frontier_point[1] + sensor_cell_radius):
#             if np.linalg.norm(np.asarray([i, j]) - np.asarray(frontier_point))*cell_dim<sensor_radius:
#                 if occupancy_grid[i][j] == -1:
#                     ig += 1
#     return ig

def mapCallback(msg):
    global map, map_msg
    map = msg.data
    map_msg = msg

def frontiersCallback(msg):
    global frontiers
    frontiers = [(p.x, p.y) for p in msg.points]

def odomCallback(msg):
    global current_pos
    current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def goalCallback(msg):
    global goal_status
    goal_status = msg

def goal():
    global map, frontiers, current_pos, goal_status

    sensor_radius = 0.1
    alpha = 1
    beta = 1

    rospy.init_node('task_allocator')

    rospy.Subscriber('map', OccupancyGrid, mapCallback)
    while len(map)<1:
        rospy.loginfo("Waititng for map data")
        rospy.sleep(0.1)

    rospy.Subscriber('filtered_points', PointArray, frontiersCallback)

    rospy.Subscriber('odom', Odometry, odomCallback)

    rospy.Subscriber('move_base/status', GoalStatusArray, goalCallback)


    while not rospy.is_shutdown():

        goal_id = 0

        if len(frontiers) == 0 or len(current_pos) == 0:
            rospy.sleep(0.5)
            continue

        #Get the Occupancy Grid and pre-proccess
        size = int(m.sqrt(len(map)))
        grid = np.resize(np.asarray(map), (size, size))

        optimal_frontier = ()
        least_revenue = m.inf
        for frontier in frontiers:
            if any(m.dist(frontier, f) < BLACKLIST_RADIUS for f in failed_goals):
                continue
            cost = nav_cost(grid, current_pos, frontier)
            ig = informationGain(map_msg, frontier, sensor_radius)

            revenue = alpha*cost + beta*ig

            if revenue < least_revenue:
                least_revenue = revenue
                optimal_frontier = frontier

        if len(optimal_frontier) == 0:
            rospy.sleep(0.5)
            continue

        if not movebase_client(optimal_frontier):
            failed_goals.append(tuple(optimal_frontier))


if __name__ == '__main__':
    try:
        goal()
    except rospy.ROSInterruptException:
        pass