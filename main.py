import numpy as np
from maputils import *
from nodeclass import *
from planner import Planner

scale = 50
map_size = (10, 10)
obstacles =  [
                Circle(center = (2, 2), radius = 1),
                Circle(center = (2, 8), radius = 1),
                Polygon(verts = np.array([[0.25, 5.75], [1.75, 5.75], [1.75, 4.25], [0.25, 4.25]],)),
                Polygon(verts = np.array([[3.75, 5.75], [6.25, 5.75], [6.25, 4.25], [3.75, 4.25]],)),
                Polygon(verts = np.array([[7.25, 4], [8.75, 4], [8.75, 2], [7.25, 2]],)),
             ]

print('\nStart position coordinates:')
x_s, y_s, theta_s = input('x,y,theta(in degrees) : ').split(',')
start_pos = int(float(x_s)*scale), int(float(y_s)*scale), int(theta_s)

print('\nGoal position coordinates:')
x_g, y_g = input('x,y : ').split(',')
goal_pos = int(float(x_g)*scale), int(float(y_g)*scale), 0

print('\nWheel RPMS:')
rpm1, rpm2 = input('rpm1, rpm2 : ').split(',')
rpm1, rpm2 = float(rpm1), float(rpm2)
clearance = float(input('\nClearance from obstacle: '))

robot = Robot(rpm1, rpm2, scale = scale)
map = Map(map_size, obstacles, clearance + robot.radius, scale)

print('\nMap Created')

H = 2

if map.is_free(start_pos) and map.is_free(goal_pos):
    # euclidean distance as heuristic
    hv = lambda pos: H*dist(pos, goal_pos)
    #start and goal nodes
    start_node = Node(start_pos, None, (0, 0), 0, hv(start_pos))
    goal_node  = Node( goal_pos, None, None, 0, 0)
    # initialise dijsktra search with map and a start position
    A_star = Planner(robot,map, start_node, hv)
    # search for a path to goal position
    path = A_star.search(goal_node)
    # visualize and save explored nodes and path
    print('\nsaving exploration video')
    A_star.visualize(path)
else:
    print('Start or Goal is inside obstacle')
