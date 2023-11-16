# !/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt

def lineseg_dists(p, a, b):
    # https://stackoverflow.com/questions/54442057/calculate-the-euclidian-distance-between-an-array-of-points-to-a-line-segment-in
    
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
    def __init__(self, map, start, destination):
        self.algorithm = "dijkstra"
        self.map = map

        # for testing only
        # self.path = np.dstack((range(5), [0, 3, 2, -1, 0]))[0]
        # self.path = np.dstack((np.random.uniform(5,10,100), np.random.uniform(5,10,100)))[0]
        self.path = np.dstack([np.arange(0, 1000, 0.1), 16*np.sin(np.arange(0, 1000, 0.1))])[0]
        # self.path = np.dstack([np.arange(0, 5, 1), [0,1,2,1,0]])[0]
        
        print(self.path)
        # self.path = []

        self.current_pose = start
        self.destination = destination

    # sets the postition of the robot
    def update_pose(self, pose):
        self.current_pose = pose

    def reduce_path(self, epsilon):
        assert (epsilon > 0)
        
        def lp(epsilon, path, idxs):
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

    def get_next_node(self):
        if self.algorithm == "dijkstra":
            adjacent = self.get_adjacent(self.current_pose)
            min = 0
            for i in range(len(adjacent)):
                if (
                    (self.path[adjacent[min][0]][adjacent[min][1]] == -1)
                    or (self.path[adjacent[i][0]][adjacent[i][1]] != -1)
                    and (
                        self.path[adjacent[i][0]][adjacent[i][1]]
                        < self.path[adjacent[min][0]][adjacent[min][1]]
                    )
                ):
                    min = i
            return adjacent[min]
        elif self.algorithm == "a_star":
            return None
        else:
            return None

    def compute_path(self):
        if self.algorithm == "dijkstra":
            self.compute_path_dijkstra()
        elif self.algorithm == "a_star":
            self.compute_path_a_star()
        else:
            return

    # returns the vector that coresponds to the direction the robot should move in world coordinates.
    def get_translation_vector(self):
        next_node = self.get_next_node()
        if (
            self.path[self.current_pose[0]][self.current_pose[1]] != -1
            and self.path[next_node[0]][next_node[1]] != -1
        ):
            translation = (
                -self.current_pose[0] + next_node[0],
                -self.current_pose[1] + next_node[1],
            )
            return translation
        else:
            return (0, 0)

    def get_adjacent(self, node):
        borders = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        adjacent_nodes = []
        for i in borders:
            if (
                (node[0] + i[0] > -1)
                and (node[0] + i[0] < len(self.map))
                and (node[1] + i[1] > -1)
                and (node[1] + i[1] < len(self.map[0]))
            ):
                adjacent_nodes.append((node[0] + i[0], node[1] + i[1]))
        return adjacent_nodes

    def compute_path_a_star(self):
        self.algorithm = "a_star"
        return

    def compute_path_dijkstra(self):
        self.algorithm = "dijkstra"
        checked = [self.destination]
        unchecked = self.get_adjacent(self.destination)
        self.path = [[-1] * len(self.map[0]) for i in range(len(self.map))]
        for i in unchecked:
            self.path[i[0]][i[1]] = 1
        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.path[j[0]][j[1]] == -1 and self.map[j[0]][j[1]] == 1:
                        tempUnchecked.append(j)
                        self.path[j[0]][j[1]] = self.path[i[0]][i[1]] + 1
                checked.append(i)
            unchecked = tempUnchecked


if __name__ == "__main__":
    fig, ax = plt.subplots()
    p = PathFinding(None, None, None)
    ax.plot(p.path[:, 0], p.path[:, 1], 'r-')
    start=time.time()
    print(p.reduce_path(8))
    print("reduce", time.time()-start)
    ax.plot(p.path[:, 0], p.path[:, 1], 'g-')
    plt.show()
