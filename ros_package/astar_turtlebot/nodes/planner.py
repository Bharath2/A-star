import numpy as np
import heapq as hq
from nodeclass import *
from maputils import *
from time import time

class Planner:
    def __init__(self, robot, map, start_node, hv):
        ''' Initialise with environment map and start poseition
        '''
        self.robot = robot
        self.map = map   # environment map
        self.hv = hv
        self.start_node =  start_node   # create start node
        self.explored = {start_node.pose: start_node}  # initialise the explored nodes dict
        # variable to store recently explored path
        self.recent_path = None
        
    def search(self, goal_node):
        ''' Search for a path from start to given goal poseition
        '''
        # initialise openset with start node
        openset = [self.start_node]
        # Run Dijkstra Algorithm
        start_t = time()
        while openset:
            curr_node = hq.heappop(openset)  # pop the next node in openset
            self.map.close(curr_node.pose) # close the poped node
            
            # if the curr_node is goal return path
            if self.check_goal(curr_node, goal_node):
                end_t = time()
                print(len(self.explored))
                print('\nPath found in', np.round(end_t - start_t, 3), 'seconds\n')
                print('cost from start to goal: ', np.round(curr_node.c2c, 3)/self.map.scale)
                return self.backtrack(curr_node)
            
            # get all children of the  current node
            for action in self.robot.actions:
                new_pose, cost, _ = self.robot.next(curr_node.pose, action)
                new_pose = self.map.roundoff(new_pose)
                new_cost = curr_node.c2c + cost
                # if the new position is not open, skip the below steps
                if self.map.is_free(new_pose):
                    # update the openset with new c2c
                    if new_pose not in self.explored: 
                        child_node = Node(new_pose, curr_node, action, new_cost, self.hv(new_pose))
                        self.explored[new_pose] = child_node
                        hq.heappush(openset, child_node)
                    else: 
                        child_node = self.explored[new_pose]
                        if new_cost < child_node.c2c:
                            child_node.c2c = new_cost 
                            child_node.parent = curr_node
                            child_node.action = action
                            hq.heapify(openset)  
                    
        # if the heap is exhausted, path is not found. 
        print('Goal is not reachable')
        return None
    
    def check_goal(self, curr_node, goal_node):
        dt = dist(curr_node.pose, goal_node.pose)
        return dt < 15
        
    def backtrack(self, node):
        ''' Backtrack the path from given node to start node
        '''
        print(node.pose)
        path = []
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        self.recent_path = path
        return path
    
    
    def visualize(self, path, name = 'exploration'):
        ''' Visualise the exploration and the recently found path
        '''
        img = self.map.get_image()
        h, w, _ = img.shape
        # open video writer
        out = cv2.VideoWriter(f'{name}.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (w, h))
        # visualize exploration
        for pose, node in self.explored.items():
            if node.parent is None: continue
            parent_pose = node.parent.pose
            _, _, (xs, ys) = self.robot.next(parent_pose, node.action)
            cv2.polylines(img, [np.int0(np.c_[xs, w - ys -1])], False, [0, 80 ,0], 1)
            out.write(img)
            cv2.imshow('Exploration', img)
            if cv2.waitKey(1) == ord('q'):
                break
        # visualise path
        if self.recent_path is not None:
            for node in self.recent_path[1:]:
                parent_pose = node.parent.pose
                _, _, (xs, ys) = self.robot.next(parent_pose, node.action)
                cv2.polylines(img, [np.int0(np.c_[xs, w - ys - 1])], False, [0, 0 , 255], 2)
            for i in range(100): out.write(img)
        out.release()
        #show final image and save video
        cv2.imshow('Exploration', img)
        cv2.imwrite('final.jpg', img)
        print('\npress any key to exit')
        cv2.waitKey(0)
        print(f'Video saved as {name}.mp4')
            