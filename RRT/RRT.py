import pybullet as p
from environment.world import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
import copy
import math
import random
import time

random.seed(0)

class RRT:

    def __init__(self, x_range=(-5, 5), y_range=(-5, 5), z_range=(0, 1),
                 expandDis=1.0, goalSampleRate=10, maxIter=200, droneID=0):

        self.start = None
        self.goal = None
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.expand_dis = expandDis
        self.goal_sample_rate = goalSampleRate
        self.max_iter = maxIter
        self.node_sets = np.zeros((maxIter, 5))  # contain x y z and parent and cost
        self.node_sets_pointer = 0  # point at the next empty one
        self.parents_of_goal = []

        self.droneID = droneID

    def rrt_planning(self, start, goal, render=True):
        self.start = start
        self.goal = goal
        self.node_sets[0] = np.array(start + [-1, 0])  #no parent and 0 cost
        self.node_sets_pointer += 1
        if render:
            p.addUserDebugPoints([start], [[1, 0, 0]], 10)
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
                    point_id = p.addUserDebugPoints([new_node.tolist()], [[0, 0, 1]], 5)
                    line_id = p.addUserDebugLine(nearest_node.tolist(), new_node.tolist(), [0, 1, 0], 3)

                if self.is_near_goal(new_node):
                    if self.check_segment_collision(new_node, np.array(self.goal)):
                        if render:
                            p.addUserDebugLine(new_node.tolist(), self.goal, [0, 1, 0], 3)
                        path, path_distance = self.get_final_course()

                        return path, path_distance

                time.sleep(0.02)
            if self.node_sets_pointer >= self.max_iter:
                break

    def rrt_star_planning(self, start, goal, render=True):
        self.start = start
        self.goal = goal
        self.node_sets = np.zeros((self.max_iter, 6))  # contain x y z and parent and cost and line_id
        self.node_sets[0] = np.array(start + [-1, 0, -1])  #no parent and 0 cost
        self.node_sets_pointer += 1
        if render:
            p.addUserDebugPoints([start], [[1, 0, 0]], 10)
            p.addUserDebugPoints([goal], [[1, 0, 0]], 10)

        while True:
            if self.node_sets_pointer == 107:
                a = 555
            rnd = self.sample()
            parent_id, nearest_node, new_node = self.get_nearestNode_newNode(rnd)
            no_collision = self.check_segment_collision(nearest_node, new_node)

            if no_collision:
                near_inds, near_distance_square = self.find_near_nodes(new_node)
                if near_inds.size == 0:
                    new_parent_id = parent_id
                    new_distance = self.expand_dis
                    cost = new_distance + self.node_sets[new_parent_id, 4]
                else:
                    cost, new_parent_id, near_inds_NC, near_distance_NC = \
                         self.choose_new_parent(new_node, near_inds, near_distance_square)
                    if cost == None:
                        new_parent_id = parent_id
                        new_distance = self.expand_dis
                        cost = new_distance + self.node_sets[new_parent_id, 4]
                        near_inds = np.array([])

                if render:
                    point_id = p.addUserDebugPoints([new_node.tolist()], [[0, 0, 1]], 5)
                    line_id = p.addUserDebugLine(self.node_sets[new_parent_id, :3].tolist(), new_node.tolist(), [0, 1, 0], 3)
                else:
                    line_id = -1
                self.node_sets[self.node_sets_pointer] = np.array(new_node.tolist()+[new_parent_id, cost, line_id])
                self.node_sets_pointer += 1

                if near_inds.size != 0:
                    self.rewire(new_node, near_inds_NC, near_distance_NC, render)

                if self.is_near_goal(new_node):
                    if self.check_segment_collision(new_node, np.array(self.goal)):
                        if render:
                            p.addUserDebugLine(new_node.tolist(), self.goal, [0, 1, 0], 3)
                        self.parents_of_goal.append(self.node_sets_pointer-1)

                time.sleep(0.02)

            if self.node_sets_pointer >= self.max_iter:
                best_node_id = self.search_best_goal_node()
                path, path_distance = self.get_final_course2(best_node_id)
                print('-----------------')
                return path, path_distance

    def sample(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = np.array([random.uniform(self.x_range[0], self.x_range[1]),
                            random.uniform(self.y_range[0], self.y_range[1]),
                            random.uniform(self.z_range[0], self.z_range[1])])
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

    def get_final_course2(self, best_id):
        path = [self.goal]
        pointer = best_id
        distance = np.sqrt(np.sum((self.goal-self.node_sets[pointer, :3])**2))+self.node_sets[pointer, 4]
        while self.node_sets[pointer, 3] != -1:
            node = self.node_sets[pointer, :3]
            path.insert(0, node.tolist())
            pointer = self.node_sets[pointer, 3].astype(int)
        path.insert(0, self.start)
        return path, distance

    def find_near_nodes(self, new_node):
        r_square = (10.0 * math.sqrt(math.log(self.node_sets_pointer) / self.node_sets_pointer)) ** 2
        distance_all_square = np.sum((self.node_sets[:self.node_sets_pointer, :3]-new_node)**2, axis=1)
        near_inds = np.where(distance_all_square <= r_square)[0]
        near_distance_square = distance_all_square[near_inds]
        return near_inds, near_distance_square

    def choose_new_parent(self, newNode, nearInds, near_distance_square):
        # MAX_RAY_INTERSECTION_BATCH_SIZE = 16384  # Pay attention!
        results = p.rayTestBatch([newNode.tolist()]*len(nearInds), self.node_sets[nearInds, :3].tolist())
        non_collision_indices = [i for i, result in enumerate(results) if result[0] == -1]
        if len(non_collision_indices) == 0:
            return [None]*4

        nearInds_NoCollision = nearInds[non_collision_indices]
        near_distance_square_NoCollision = near_distance_square[non_collision_indices]

        near_distance_NoCollision = np.sqrt(near_distance_square_NoCollision)
        total_cost = self.node_sets[nearInds_NoCollision, 4] + near_distance_NoCollision
        cost = np.min(total_cost)
        new_parent_id = nearInds_NoCollision[np.argmin(total_cost)]

        return cost, new_parent_id, nearInds_NoCollision, near_distance_NoCollision

    def rewire(self, new_node, near_inds_NC, near_distance_NC, render):
        original_cost = self.node_sets[near_inds_NC, 4]
        new_cost = near_distance_NC + self.node_sets[self.node_sets_pointer-1, 4]
        inds = np.where(new_cost < original_cost)[0]
        rewire_inds = near_inds_NC[inds]
        rewire_costs = new_cost[inds]

        for ind, cost in zip(rewire_inds, rewire_costs):
            if render:
                p.removeUserDebugItem(int(self.node_sets[ind, 5]))
                line_id = p.addUserDebugLine((self.node_sets[ind, :3]).tolist(), new_node.tolist(), [0, 1, 0], 3)
            else:
                line_id = -1
            self.node_sets[ind, 3:] = np.array([self.node_sets_pointer-1, cost, line_id])
            self.propagate_cost_to_leaves(ind)

    def propagate_cost_to_leaves(self, parent_ind):
        for ind in range(self.node_sets_pointer-1):
            if self.node_sets[ind, 3] == parent_ind:
                self.node_sets[ind, 4] = self.node_sets[parent_ind, 4] + \
                                         np.sqrt(np.sum((self.node_sets[ind, :3]-self.node_sets[parent_ind, :3])**2))
                self.propagate_cost_to_leaves(ind)

    def search_best_goal_node(self):
        distance_goal_to_node = \
            np.sqrt(np.sum((self.node_sets[self.parents_of_goal, :3]-np.array(self.goal))**2, axis=1))
        costs = distance_goal_to_node + self.node_sets[self.parents_of_goal, 4]
        best_node_id = self.parents_of_goal[np.argmin(costs)]

        return  best_node_id


