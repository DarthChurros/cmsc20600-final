import numpy as np
import math

class PathFinding:
    
    def __init__(self, map, start, destination, algorithm="dijkstra", bound=4):
        self.algorithm = algorithm
        self.map = map
        self.shortest_dists = None
        self.current_pose = start
        self.destination = destination
        self.bound = bound
        
        self.path = None

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
        return self.get_translation_vector(nodes[-1])

    def find_nearest_non_negative(self, node=None):
        if node is None:
            node = self.current_pose

        checked = [node]
        unchecked = self.get_adjacent(node)

        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.shortest_dists[j[0]][j[1]] == -1 and not (j in checked or j in tempUnchecked):
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
            for i in range(len(adjacent)):
                if (self.shortest_dists[adjacent[min][0]][adjacent[min][1]] == -1) or ((self.shortest_dists[adjacent[i][0]][adjacent[i][1]] != -1) and (self.shortest_dists[adjacent[i][0]][adjacent[i][1]] < self.shortest_dists[adjacent[min][0]][adjacent[min][1]])):
                    min = i

            return adjacent[min]
        elif self.algorithm == "a_star":
            return None
        else:
            return None       

    def compute_shortest_dists(self):
        if self.algorithm == "dijkstra":
            self.compute_shortest_dists_dijkstra()
        elif self.algorithm == "a_star":
            assert(False)
            # TODO!!!
        else:
            return

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
        borders = [(-1,0),(0,1),(1,0),(0,-1)]
        adjacent_nodes = []
        for i in borders:
            if (node[0] + i[0] > -1) and (node[0] + i[0] < len(self.map)) and (node[1] + i[1] > -1) and (node[1] + i[1] < len(self.map[0])):
                adjacent_nodes.append((node[0] + i[0],node[1] + i[1]))
        return adjacent_nodes

    def compute_path(self):
        '''
        Must call compute_shortest_dists before calling this.
        '''
        
        temp_path = []
        current_pose = self.current_pose
        destination = self.destination

        while current_pose != destination:
            temp_path.append(current_pose)
            current_pose = self.get_next_node(current_pose)

        self.path = np.array(temp_path)

    

    def compute_shortest_dists_dijkstra(self):
        self.algorithm = "dijkstra"
        checked = [self.destination]
        unchecked = self.get_adjacent(self.destination)
        self.shortest_dists = np.zeros(shape=self.map.shape) - 1
        self.shortest_dists[self.destination[0], self.destination[1]] = 0
        
        # self.path[:20, :40] = 0
        
        # import matplotlib.pyplot as plt
        # print("combinati")
        # plt.imshow(self.path, cmap='hot', interpolation='nearest', origin="lower")
        # plt.plot(self.destination[0], self.destination[1], "go")
        # plt.show()
        
        for i in unchecked:
             self.shortest_dists[i[0]][i[1]] = 1
        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.shortest_dists[j[0]][j[1]] == -1 and self.map[j[0]][j[1]] == 1:
                        tempUnchecked.append(j)
                        self.shortest_dists[j[0]][j[1]] = self.shortest_dists[i[0]][i[1]] + 1
                checked.append(i)
            unchecked = tempUnchecked
    
    def at_destination(self):
        return self.shortest_dists[self.current_pose[0]][self.current_pose[1]] < self.bound and self.shortest_dists[self.current_pose[0]][self.current_pose[1]] != -1