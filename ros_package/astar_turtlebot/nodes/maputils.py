import cv2
import numpy as np
from scipy.signal import convolve2d

class Map:
    ''' A class for environment map
    '''
    def __init__(self, size, obstacles, clearance, scale = 1):
        ''' Creates the required map
        '''
        # size of the map
        self.size = size
        self.scale = scale
        self.new_size = new(size, scale)
        self.grid = self.create_map(obstacles, clearance)
        self.cspace = np.zeros((*self.new_size, 36))
    
    def create_map(self, obstacles, clearance):
        ''' Create a binary occupancy grid given obstacles list
        '''
        # create an empty grid
        grid = np.zeros(self.new_size, np.int8)
        # fill the grid with the given obstacles
        xs, ys = np.indices(self.new_size)
        for obs in obstacles:
            grid |= obs.get(xs, ys, self.scale)
        # boundaries of the map
        grid[(-1, 0), :] = 1; grid[:, (-1, 0)] = 1

        # Add circular clearance to the obstacles
        cl = int(clearance*self.scale)
        filter = np.zeros((2*cl + 1, 2*cl + 1))
        cv2.circle(filter, (cl, cl), cl, 1, -1)
        # dilate with above filter 
        mp = convolve2d(grid, filter, mode = 'same')
        mp = np.where(mp > 0, 1, 0)
        return mp + grid

    def is_free(self, pos):
        ''' Check if the node is in bounds and free of obstacle
        '''
        # inside the map size
        if pos[0] < 0 or pos[1] < 0: 
            return False 
        if pos[0] >= self.new_size[0] or pos[1] >= self.new_size[1]: 
            return False 

        x, y, theta = pos
        j = int(theta/10)

        return self.grid[x, y] == 0 and self.cspace[x, y, j] == 0
    
    def close(self, pos):
        x, y, theta = pos
        j = int(theta/10)
        self.cspace[x, y, j] = 1
    
    def get_image(self):
        # To visualise the environment map
        mx, my = self.new_size
        img = np.full((my, mx, 3), 255, np.uint8)
        xs, ys = np.where(self.grid == 1)
        img[my - ys - 1, xs] = 60
        xs, ys = np.where(self.grid == 2)
        img[my - ys - 1, xs] = 120
        return img
    
    def roundoff(self, pos):
        x, y, theta = pos
        return int(x), int(y), int(theta/10)*10


class Polygon:
    ''' Polygon shaped obstacle class
    '''
    def __init__(self, verts):
        self.N = len(verts)
        self.center = np.mean(verts, axis = 0)
        self.verts = np.vstack((verts, verts[0]))
    
    def get(self, xs, ys, scale):
        cx, cy = new(self.center, scale)
        verts = new(self.verts, scale)
        grid = 1
        # for all polygon edges
        for i in range(self.N):
            x1, y1 = verts[i]
            x2, y2 = verts[i+1]
            s1 = (xs - x1) * (y2 - y1) - (ys - y1) * (x2 - x1)
            s2 = (cx - x1) * (y2 - y1) - (cy - y1) * (x2 - x1)
            #points on the same side of the edge, as polygon's center
            grid &= (s1*s2 >= 0)
        return grid
        

class Circle:
    ''' Circle shaped obstacle class
    '''
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    
    def get(self, xs, ys, scale):
        cx, cy = new(self.center, scale)
        radius = int(self.radius*scale)
        grid = (xs - cx)**2 + (ys - cy)**2 <= radius**2
        return grid


def new(points, scale):
    return np.int0(np.array(points)*scale)

if __name__ == '__main__':
    
    obs = [   
           Circle(center = (2, 2), radius = 1),
           Circle(center = (2, 8), radius = 1),
           Polygon(verts = np.array([[0.25, 5.75], [1.75, 5.75], [1.75, 4.25], [0.25, 4.25]], np.int0)),
           Polygon(verts = np.array([[4.25, 5.75], [5.75, 5.75], [5.75, 4.25], [4.25, 4.25]], np.int0)),
           Polygon(verts = np.array([[7.25, 3.75], [9.75, 3.75], [9.75, 2.25], [8.25, 2.25]], np.int0)),
             ]

    map = Map((10, 10), obs, 0.1, scale = 50)
    img = map.get_image()

    # print(new([0.25, 0.75], 0.1))

    # print(map.is_open((2,2,0)))

    # print((1.14/0.02))
    # print((1.12/0.02))
    # print(58 *2)

    print(int(0.25*50))

    # print()
    cv2.imshow('test_map', img)
    cv2.waitKey(0)
    


    
