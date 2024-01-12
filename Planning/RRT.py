import pybullet as p
from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
import copy
import math
import random
import time
from Helper.Bounds import Bounds

random.seed(0)

class RRT:

    def __init__(self, bounds=Bounds([[5, 5], [5, 5], [5, 5]]),
                 expandDis=1.0, goalSampleRate=10, maxIter=200, droneID=0):

        self.start = None
        self.goal = None
        self.Bounds = bounds

        self.expand_dis = expandDis
        self.goal_sample_rate = goalSampleRate
        self.max_iter = maxIter
        self.node_sets = np.zeros((maxIter, 5))  # contain x y z and parent and cost
        self.node_sets_pointer = 0  # point at the next empty one

        self.droneID = droneID

    def rrt_planning(self, start, goal, render=True):
        self.start = start
        self.goal = goal
        self.node_sets[0] = np.array(start + [-1, 0])  #no parent and 0 cost
        self.node_sets_pointer += 1
        if render:
            p.addUserDebugPoints([goal], [[1, 0, 0]], 10)

        while True:
            rnd = self.sample()
            parent_id, nearest_node, new_node = self.get_nearestNode_newNode(rnd)
            no_collision = self.check_segment_collision(nearest_node, new_node)

            if no_collision:
                cost = self.expand_dis + self.node_sets[parent_id, 4]
                self.node_sets[self.node_sets_pointer] = np.array(new_node.tolist()+[parent_id, cost])
                self.node_sets_pointer += 1
                if render:
                    p.addUserDebugPoints([new_node.tolist()], [[0, 0, 1]], 5)
                    p.addUserDebugLine(nearest_node.tolist(), new_node.tolist(), [0, 1, 0], 3)

                if self.is_near_goal(new_node):
                    if self.check_segment_collision(new_node, np.array(self.goal)):
                        if render:
                            p.addUserDebugLine(new_node.tolist(), self.goal, [0, 1, 0], 3)
                        path, path_distance = self.get_final_course()

                        return path, path_distance

                time.sleep(0.02)

    def sample(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = np.array([random.uniform(self.Bounds.xMin, self.Bounds.xMax) + self.Bounds.center[0],
                            random.uniform(self.Bounds.yMin, self.Bounds.yMax) + self.Bounds.center[1],
                            random.uniform(self.Bounds.zMin, self.Bounds.zMax) + self.Bounds.center[2]])
        else:  # goal point sampling
            rnd = np.array(self.goal)
        return rnd

    def get_nearestNode_newNode(self, rnd):
        distance_all_square = np.sum((self.node_sets[:self.node_sets_pointer, :3]-rnd)**2, axis=1)
        minIndex = np.argmin(distance_all_square)
        min_distance_square = distance_all_square[minIndex]
        nearest_node = self.node_sets[minIndex, :3]

        # steer-line
        new_node = (rnd-nearest_node)*self.expand_dis/np.sqrt(min_distance_square) + nearest_node
        # print(np.sqrt(np.sum((nearest_node-new_node)**2)))

        return minIndex, nearest_node, new_node

    def check_segment_collision(self, nearest_node, new_node):
        result = p.rayTest(nearest_node.tolist(), new_node.tolist())
        if result[0][0] == -1:  # be careful, there is maybe bug!
            return True
        return False

    def is_near_goal(self, node):
        d = np.sqrt(np.sum((self.goal-node)**2))
        if d < self.expand_dis:
            return True
        return False

    def get_final_course(self):
        path = [self.goal]
        pointer = self.node_sets_pointer - 1
        distance = np.sqrt(np.sum((self.goal-self.node_sets[pointer, :3])**2))+self.node_sets[pointer, 4]
        while self.node_sets[pointer, 3] != -1:
            node = self.node_sets[pointer, :3]
            path.insert(0, node.tolist())
            pointer = self.node_sets[pointer, 3].astype(int)
        path.insert(0, self.start)
        return path, distance






