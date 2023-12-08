import numpy as np
import math

def lineseg_dists(p, a, b):
    '''
    Computes the shortest euclidean distance between a point 'p' and a line segment
    defined by endpoints 'a' and 'b'.
    
    Source:
    https://stackoverflow.com/questions/54442057/calculate-the-euclidian-distance-between-an-array-of-points-to-a-line-segment-in
    '''
    
    
    # Handle case where p is a single point, i.e. 1d array.
    p = np.atleast_2d(p)

    # TODO for you: consider implementing @Eskapp's suggestions
    if np.all(a == b):
        return np.linalg.norm(p - a, axis=1)

    # normalized tangent vector
    d = np.divide(b - a, np.linalg.norm(b - a))

    # signed parallel distance components
    s = np.dot(a - p, d)
    t = np.dot(p - b, d)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(p))])

    # perpendicular distance component, as before
    # note that for the 3D case these will be vectors
    c = np.cross(p - a, d)

    # use hypot for Pythagoras to improve accuracy
    return np.hypot(h, c)

class PathFinding:
    
    def __init__(self, map, destination, algorithm="dijkstra", outOfBounds=0.2, bound=40):
        self.algorithm = algorithm
        self.closestMap = map
        boundMap = np.zeros(shape=self.closestMap.shape)
        occupiable = (self.closestMap <= outOfBounds)
        boundMap[occupiable] = -1
        boundMap = boundMap + 1
        self.map = boundMap
        self.shortest_dists = None
        self.current_pose = None
        self.destination = destination
        self.bound = bound
        
        
        self.path = None

    def reduce_path(self, epsilon):
        '''
        Reduce self.path under Douglas-Peuker with a maximum tolerance of epsilon.
        i.e. if a point exceeds more than epsilon from surrounding line segment,
        it must be kept. Otherwise, delete the point.
        '''
        
        assert (epsilon > 0)
        
        def lp(epsilon, path, idxs):
            ''' The internal helper that performs the main recursive logic of Douglas-Peuker. '''
            
            
            # assumes that self.path is a 2D numpy array [[x1, y1], [x2, y2], ...]
            path_len = len(path)
            assert(path_len >= 2)

            # path with only two points cannot be reduced, fewer is faulty input
            if path_len == 2:
                return

            # determine reference line between points
            p1, p2 = path[0], path[-1]

            # get farthest point from line
            dists = lineseg_dists(path, p1, p2)
            idx = np.argmax(dists * idxs)
            p = path[idx]

            # determine whether to keep that point or reduce the path
            if dists[idx] > epsilon:
                # keep
                lp(epsilon, path[: idx + 1], idxs[: idx + 1])
                lp(epsilon, path[idx:], idxs[idx:])
            else:
                # reduce segment
                idxs[1:-1] = False
        
        path = self.path
        idxs = np.ones(len(path), dtype=bool)
        
        lp(epsilon, path, idxs)

        self.path = self.path[idxs]
        return self.path
        

    #sets the postition of the robot
    def update_pose(self, pose):
        self.current_pose = pose

    def naive_path_finder(self, epsilon):
        next_node = self.get_next_node()
        nodes = []
        distances = []
        
        while(all(distances)):
            nodes.append(next_node)
            distances.append(True)
            next_node = self.get_next_node(next_node)
            if self.shortest_dists[next_node[0]][next_node[1]] < self.bound:
                break
            elif (next_node[0] - self.current_pose[0] == 0):
                for i in range(len(nodes)):
                    distances[i] = abs(nodes[i][0] - next_node[0]) < epsilon
            else:
                slope = (next_node[1] - self.current_pose[1])/(next_node[0] - self.current_pose[0])
                offset = (next_node[1] - (slope * next_node[0]))
                for i in range(len(nodes)):
                    d = lambda x : math.hypot(nodes[i][0]-x,nodes[i][1] - (slope * x) - offset)
                    z = (nodes[i][0] + (slope * (nodes[i][1] - offset)))/(1 + pow(slope,2))
                    distances[i] = d(z) < epsilon
        return self.get_translation_vector(nodes[-1]), nodes[-1]

    def find_nearest_non_negative(self, node=None):
        if node is None:
            node = self.current_pose

        checked = [node]
        unchecked = self.get_adjacent(node)
        
        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.shortest_dists[j[0]][j[1]] == -1 and not (j in checked or j in tempUnchecked or j in unchecked):                       
                        tempUnchecked.append(j)
                    elif self.shortest_dists[j[0]][j[1]] != -1:
                        return j
                checked.append(i)
            unchecked = tempUnchecked
        return (0,0)
        

    def get_next_node(self, node=None):
        if node is None:
            node = self.current_pose
    
        if self.algorithm == "dijkstra":
            adjacent = self.get_adjacent(node)
            min = 0
            if(self.shortest_dists[node[0]][node[1]] == -1):
                return self.find_nearest_non_negative(node)
            else:
                for i in range(len(adjacent)):
                    minx = adjacent[min][0]
                    miny = adjacent[min][1]
                    
                    curx = adjacent[i][0]
                    cury = adjacent[i][1]
                    
                    if (self.shortest_dists[minx][miny] == -1):
                        min = i
                        continue
                    
                    node_valid = self.shortest_dists[curx][cury] != -1
                    path_len_equal = self.shortest_dists[curx][cury] == self.shortest_dists[minx][miny]
                    farther_from_wall = self.closestMap[curx][cury] > self.closestMap[minx][miny]
                    closer_to_dest = self.shortest_dists[curx][cury] < self.shortest_dists[minx][miny]
                    
                    
                    if (node_valid and ((path_len_equal and farther_from_wall) or closer_to_dest)):
                        min = i
                        
                return adjacent[min]
        elif self.algorithm == "a_star":
            return None
        else:
            return None       

    

    #returns the vector that coresponds to the direction the robot should move in world coordinates.
    def get_translation_vector(self, node=None):
        next_node = node
        if node is None:
            next_node = self.get_next_node()

        if self.shortest_dists[self.current_pose[0]][self.current_pose[1]] == -1 and self.shortest_dists[next_node[0]][next_node[1]] == -1:
            next_node = self.find_nearest_non_negative(self.current_pose) 

        translation = (-self.current_pose[0] + next_node[0], - self.current_pose[1] + next_node[1])
        if translation[0] == 0 and translation[1] == 0:
            return (0,0)
        else:
            hy = math.hypot(translation[0],translation[1])
            return (translation[0]/hy,translation[1]/hy)
    
    def get_adjacent(self, node):
        # borders without diagonals
        # borders = [(-1,0),(0,1),(1,0),(0,-1)]
        
        borders = [(-1,0),(0,1),(1,0),(0,-1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        adjacent_nodes = []
        for i in borders:
            if (node[0] + i[0] > -1) and (node[0] + i[0] < len(self.map)) and (node[1] + i[1] > -1) and (node[1] + i[1] < len(self.map[0])):
                adjacent_nodes.append((node[0] + i[0],node[1] + i[1]))
        return adjacent_nodes


    def compute_path_finding(self):
        if self.algorithm == "dijkstra":
            self.compute_dijkstra()
        elif self.algorithm == "a_star":
            self.compute_astar()
        else:
            return

    def get_path(self):
        if self.path is None:
            self.compute_path()    
        return self.path
    
    def compute_path(self):
        temp_path = []
        current_pose = self.current_pose
        destination = self.destination    

        while current_pose != destination:
            temp_path.append(current_pose)
            current_pose = self.get_next_node(current_pose)
        
        self.path = np.array(temp_path)
        self.dist = len(self.path)

    def compute_astar(self):
        self.compute_dijkstra()
        path = self.get_path()
        self.algorithm = "a_star"
        self.shortest_dists = np.zeros(shape=self.map.shape) - 1
        for i in range(len(path)):
            p = path[i]
            self.shortest_dists[p[0]][p[1]] = i

    def compute_dijkstra(self):
        self.algorithm = "dijkstra"
        checked = [self.destination]
        unchecked = self.get_adjacent(self.destination)
        self.shortest_dists = np.zeros(shape=self.map.shape) - 1
        self.shortest_dists[self.destination[0], self.destination[1]] = 0
        
        
        for i in unchecked:
             self.shortest_dists[i[0]][i[1]] = 1
        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.shortest_dists[j[0]][j[1]] == -1 and self.map[j[0]][j[1]] == 1:
                        tempUnchecked.append(j)
                        # distance update without diagonals
                        self.shortest_dists[j[0]][j[1]] = self.shortest_dists[i[0]][i[1]] + 1
                        # distance update with diagonals
                        #self.shortest_dists[j[0]][j[1]] = self.shortest_dists[i[0]][i[1]] + np.hypot(i[0]-j[0],i[1]-j[1])
                checked.append(i)
            unchecked = tempUnchecked
        # assert(False)
    
    def at_destination(self):
        return self.shortest_dists[self.current_pose[0]][self.current_pose[1]] < self.bound and self.shortest_dists[self.current_pose[0]][self.current_pose[1]] != -1