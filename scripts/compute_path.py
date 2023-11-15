# !/usr/bin/env python3

import numpy as np


class PathFinding:
    def __init__(self, map, start, destination):
        self.algorithm = "dijkstra"
        self.map = map

        # for testing only
        # self.path = np.dstack((range(5), [0, 3, 2, -1, 0]))[0]
        self.path = []

        self.current_pose = start
        self.destination = destination

    # sets the postition of the robot
    def update_pose(self, pose):
        self.current_pose = pose

    def reduce_path(self, epsilon, path=None, idxs=None):
        # assumes that self.path is a 2D numpy array [[x1, y1], [x2, y2]]
        first_call = False

        if path is None:
            path = self.path
            first_call = True

        if idxs is None:
            idxs = np.ones(len(path), dtype=bool)

        # path with only two points cannot be reduced, fewer is faulty input
        if len(path) == 2:
            return

        # determine reference line between points
        p1, p2 = path[0], path[-1]
        line = p2 - p1
        line_len = np.linalg.norm(line)

        # get farthest point from line
        dists = np.abs(np.cross(line, path - p1) / line_len)
        idx = np.argmax(dists * idxs)
        p = path[idx]

        # determine whether to keep that point or reduce the path
        if dists[idx] > epsilon:
            # keep
            self.reduce_path(epsilon, path[: idx + 1], idxs[: idx + 1])
            self.reduce_path(epsilon, path[idx:], idxs[idx:])
        else:
            # reduce segment
            idxs[1:-1] = False

        if first_call:
            self.path = path[idxs]
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


# if __name__ == "__main__":
#     p = PathFinding(None, None, None)
#     print(p.reduce_path(2))
