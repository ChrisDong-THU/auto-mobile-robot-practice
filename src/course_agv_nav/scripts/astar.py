from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class PRM(object):
    def __init__(self, N_SAMPLE=500, KNN=20, MAX_EDGE_LEN=2):
        self.N_SAMPLE = N_SAMPLE
        self.KNN = KNN
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.minx = -10
        self.maxx = 10
        self.miny = -10
        self.maxy = 10
        self.robot_size = 0.2
        self.avoid_dist = 0.2

    def plan(self,  start_x, start_y, goal_x, goal_y ,ox , oy):
        # Obstacles
        start=time.time()
        obstacle_x = ox
        obstacle_y = oy
        #print("start_x")
        #print(start_x)
        #print(obstacle_x)
        # Obstacle KD Tree
        # print(np.vstack((obstacle_x, obstacle_y)).T)
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        # Sampling
        sample_x, sample_y = self.sampling(start_x, start_y, goal_x, goal_y, obstree)
        # Generate Roadmap
        road_map = self.generate_roadmap(sample_x, sample_y, obstree)
        #print(road_map)
        # Search Path
        path_x, path_y = self.astar(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)
        end=time.time()
        print("TIME:")
        print(end-start)
        print("length:")
        print(len(path_x))
        return path_x, path_y

    def sampling(self, start_x, start_y, goal_x, goal_y, obstree):
        sample_x, sample_y = [], []

        while len(sample_x) < self.N_SAMPLE:
            tx = (random.random() * (self.maxx - self.minx)) + self.minx
            ty = (random.random() * (self.maxy - self.miny)) + self.miny

            distance, index = obstree.query(np.array([tx, ty]))

            if distance >= self.robot_size + self.avoid_dist:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(start_x)
        sample_y.append(start_y)
        sample_x.append(goal_x)
        sample_y.append(goal_y)

        return sample_x, sample_y

    def generate_roadmap(self, sample_x, sample_y, obstree):
        road_map = []
        nsample = len(sample_x)
        sampletree = KDTree(np.vstack((sample_x, sample_y)).T)

        for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):
            distance, index = sampletree.query(np.array([ix, iy]), k=nsample)
            edges = []
            # print(len(index))

            for ii in range(1, len(index)):
                nx = sample_x[index[ii]]
                ny = sample_y[index[ii]]

                # check collision
                if not self.check_obs(ix, iy, nx, ny, obstree):
                    edges.append(index[ii])

                if len(edges) >= self.KNN:
                    break

            road_map.append(edges)

        return road_map

    def check_obs(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:
            return True

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False

    def calc_heuristic(self,n1,n2):
        w= 1.0 
        d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
        return d

    def astar(self, start_x, start_y, goal_x, goal_y, road_map,
        sample_x, sample_y):
        path_x, path_y = [], []
        #print(road_map)
        #print(start_x)
        start = Node(start_x, start_y, 0.0, -1)
        goal = Node(goal_x, goal_y, 0.0, -1)

        openset, closeset = dict(), dict()
        #print(len(road_map))
        openset[len(road_map)-2] = start
        #print(start)

        path_found = True
        while True:
            #print("7")
            if not openset:
                print("Cannot find path")
                path_found = False
                break
            #print("5")
            c_id = min(openset, key=lambda o: openset[o].cost+self.calc_heuristic(goal, openset[o]))
            current = openset[c_id]
            #print(c_id)
            if c_id == (len(road_map) - 1):
                print("Goal is found!")
                goal.cost = current.cost
                goal.parent = current.parent
                break

            del openset[c_id]
            closeset[c_id] = current

            # expand
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                #print(n_id)
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                    current.cost + d, c_id)
                if n_id in closeset:
                    continue
                if n_id in openset:
                    if openset[n_id].cost > node.cost:
                        openset[n_id].cost = node.cost
                        openset[n_id].parent = c_id
                else:
                    openset[n_id] = node

        if path_found:
            path_x.append(goal.x)
            path_y.append(goal.y)
            parent = goal.parent
            while parent != -1:
                path_x.append(closeset[parent].x)
                path_y.append(closeset[parent].y)
                parent = closeset[parent].parent

        return path_x, path_y
