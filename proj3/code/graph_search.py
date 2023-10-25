from heapq import heappush, heappop  # Recommended.
import numpy as np
import queue
import heapq as hq
from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.


def h_condition(v, v_goal):
    # return ((v[0] - v_goal[0]) * 2 + (v[1] - v_goal[1]) * 2 + (v[2] - v_goal[2]) * 2) * .5
    # return np.linalg.norm(v-v_goal)
    # print("v: ", v)
    # print("v_goal: ", v_goal)
    # print(np.sum(np.asarray(v)-np.asarray(v_goal)))
    return np.sum(np.asarray(v)-np.asarray(v_goal))


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    map_ = occ_map.map
    trav = [-1, 0, 1]
    neighbours = []
    for i in trav:
        for j in trav:
            for k in trav:
                neighbours.append([i, j, k])
    neighbours = np.array(neighbours)
    neighbours = np.delete(neighbours, 13, axis=0)

    if (astar == True):
        Q = queue.PriorityQueue()
        Q.put((h_condition(start_index, goal_index), start_index, 0, [start_index]))
        expanded = set()
        while not Q.empty():
            node = Q.get()
            path_cost = node[2]
            path_idx = node[3]
            path_meters = occ_map.index_to_metric_center(node[3])
            if (node[1]) in expanded:
                continue
            expanded.add((node[1]))
            if goal_index == (node[1]):
                path_meters[0] = start
                path_meters[-1] = goal
                return np.array(path_meters), len(expanded)

            for n in neighbours:
                c_v = np.linalg.norm(np.array(n))
                it_x, it_y, it_z = node[1] + n

                if 0 <= it_x < map_.shape[0] and 0 <= it_y < map_.shape[1] and 0 <= it_z < map_.shape[2]:
                    if map_[it_x, it_y, it_z] == False:
                        Q.put((h_condition((it_x, it_y, it_z), goal_index)+path_cost + c_v, (it_x, it_y, it_z),
                               path_cost + c_v, path_idx + [(it_x, it_y, it_z)]))
        return None, 0

    else:
        expanded = set()
        Q = queue.PriorityQueue()
        Q.put((0, start_index, [start_index]))
        while not Q.empty():
            node = Q.get()
            path_cost = node[0]
            path_idx = node[2]
            path_meters = occ_map.index_to_metric_center(node[2])
            if (node[1]) in expanded:
                continue
            expanded.add(node[1])
            if goal_index == (node[1]):
                path_meters[0] = start
                path_meters[-1] = goal
                return np.array(path_meters), len(expanded)

            for n in neighbours:
                c_v = np.linalg.norm(np.array(n))
                it_x, it_y, it_z = node[1] + n
                if 0 <= it_x < map_.shape[0] and 0 <= it_y < map_.shape[1] and 0 <= it_z < map_.shape[2]:
                    if map_[it_x, it_y, it_z] == False:
                        Q.put((path_cost + c_v, (it_x, it_y, it_z), path_idx + [(it_x, it_y, it_z)]))
        return None, 0

    return None, 0

