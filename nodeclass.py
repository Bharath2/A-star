import numpy as np

class Node:
    ''' Container class for a Node
    ''' 
    def __init__(self, pose, parent, action, c2c, c2g):
        ''' Initialise a Node class object
        '''
        self.pose = pose       # Node co-ordinates (tuple: (x, y, theta))
        self.parent = parent   # Reference to parent of this node
        self.c2c = c2c         # cost to come from start node
        self.c2g = c2g         # estimate of cost to go to goal node
        self.action = action
    
    def __lt__(self, other):
        ''' Compare two nodes based on cost (for min heap)
        '''
        return self.c2c + self.c2g < other.c2c + other.c2g


class Robot:

    def __init__(self, rpm1, rpm2, dt = 0.1, T = 1, scale = 50):
        
        self.r = 0.038*scale
        self.L = 0.354*scale

        self.dt = dt
        self.steps = int(T/self.dt)

        rpms = np.array([[0, rpm1], [rpm1, rpm1], [rpm1, 0],
                         [0, rpm2], [rpm2, rpm2], [rpm2, 0],
                         [rpm1, rpm2], [rpm2, rpm1]])

        self.actions = self.r*rpms*0.10472

        self.radius = 0.2

    def next(self, pose, action):
        ''' Returns new poseition and cost after taking an action 
        '''
        UL, UR = action
        x, y, theta = pose
        theta = theta*np.pi/180

        dL = 0.5*self.r*(UL + UR)*self.dt
        dtheta = (self.r/self.L)*(UR - UL)*self.dt

        thetas = theta + np.cumsum([dtheta]*self.steps)
        xs = x + np.cumsum(dL*np.cos(thetas))
        ys = y + np.cumsum(dL*np.sin(thetas))
        cost = dL*self.steps
        
        new_pose = xs[-1], ys[-1], (thetas[-1]*180/np.pi)%360
        return new_pose, cost, (xs, ys)
    

def dist(pos, goal_pos):
    xp, yp, tp = pos
    xg, yg, tg = goal_pos
    return np.sqrt((xp - xg)**2 + (yp - yg)**2)

    
if __name__ == 'main':

    st = Node((0, 0, 30), None, 0, 0)