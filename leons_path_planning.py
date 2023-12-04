import time

import numpy as np
import pygame
import sys


class Environment:
    # rectangular
    def __init__(self):
        self.width = 10
        self.height = 10
        self.obstacles = []
        self.start = (0, 0)
        self.goal = (0, 0)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def is_free(self, sample_point):
        nearest_obst = 0
        nearest_obst_dist = np.inf
        for obstacle in self.obstacles:
            if np.linalg.norm(obstacle.center - sample_point, ord=2) < nearest_obst_dist:
                nearest_obst = obstacle
        if nearest_obst.interior(sample_point):
            return False
        else:
            return True

    def build_(self):
        start_pos = (1, 1)
        goal_pos = (9, 9)
        self.start = np.array(start_pos)
        self.goal = np.array(goal_pos)
        return start_pos, goal_pos

    def visualize(self, V, E):
        # Initialize Pygame
        pygame.init()

        # Set up the display
        width, height = 1000, 1000
        sf_w = width / self.width #scaling factor width
        sf_h = height / self.height #scaling factor height
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Pygame Drawing")

        # Set up colors
        black = (0, 0, 0)
        white = (255, 255, 255)
        red = (255, 0, 0)
        green = (0, 255, 0)

        # Main loop
        n_vertices = len(V)
        print(n_vertices)
        lc = 0 # loop count
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Clear the screen
            screen.fill(white)
            # Draw start and goal
            pygame.draw.rect(screen, green, (self.start[0]*sf_w, self.start[1]*sf_h, 50, 50)) # (x, y, width, height)
            pygame.draw.rect(screen, red, (self.goal[0]*sf_w, self.goal[1]*sf_h, 50, 50))
            # draw obstacles
            for obs in self.obstacles:
                pygame.draw.rect(screen, black,((obs.center[0]-obs.width/2) * sf_w, (obs.center[1]-obs.height/2) * sf_h, obs.width*sf_w, obs.height*sf_h))  # (x, y, width, height)

            # draw edges and vertices
            for e in E[:min(lc+1, len(E))]:
                e_start = (int(e[0][0] * sf_w), int(e[0][1] * sf_h))
                e_end = (int(e[1][0] * sf_w), int(e[1][1] * sf_h))
                pygame.draw.line(screen, black, e_start, e_end, width=2)
            for v in V[:min(lc+1, n_vertices)]:
                pygame.draw.circle(screen, green, (int(v[0] * sf_w), int(v[1] * sf_h)), 5)

            time.sleep(0.05)
            if lc > max(len(V), len(E)):
                print("graph completed")
                time.sleep(5)
                pygame.quit()
                sys.exit()

            # Update the display and counters
            lc += 1
            pygame.display.flip()

        # Quit Pygame
        pygame.quit()
        sys.exit()


class Obstacle:
    def __init__(self, pos=(0, 0), width=1, height=1):
        self.center = np.array(pos)  # center as tuple
        self.width = width
        self.height = height

    def interior(self, sample_point):
        dist_x = sample_point[0] - self.center[0]
        dist_y = sample_point[1] - self.center[1]
        if np.linalg.norm(dist_x) < self.width / 2 or np.linalg.norm(dist_y) < self.height / 2:
            return True
        return False


class RRT:
    def __init__(self):
        self.V = [np.array([0, 0])] # this must be the goal node
        self.E = []
        self.hury = 10  # period of steps for when to check whether we can reach the goal
        self.ko = 200 # timeout, ie number of steps before termination

    def get_sample(self, env):
        res = 100
        x = np.random.choice(np.linspace(0, env.width, num=res), 1)
        y = np.random.choice(np.linspace(0, env.height, num=res), 1)
        return np.array([x, y])

    def get_nearest_neighbor(self, new_node):
        nnn = 0
        nn_dist = np.inf
        for node in self.V:
            d_ = np.linalg.norm(node - new_node, ord=2)
            if d_ < nn_dist:
                nnn = node  # new nearest neighbor
                nn_dist = d_

        return nnn

    def path_free(self, edge, env):
        res = int(10) # number of samples per edge unit length
        edge_length = int(np.sqrt((edge[0][0]-edge[1][0])**2+(edge[0][1]-edge[1][1])**2))
        x = np.random.choice(np.linspace(int(edge[0][0]), int(edge[1][0]), num=(res*edge_length)), (res*edge_length), replace=False)
        y = np.random.choice(np.linspace(int(edge[0][1]), int(edge[1][1]), num=(res*edge_length)), (res*edge_length), replace=False)
        for i in range(res*edge_length):
            if env.is_free((x[i], y[i])):
                pass
            else:
                return False
        return True

    def run(self, env, visualize=False):
        counter = 0
        while counter <= self.ko:
            if counter % 10 == 0:
                new_sample = env.goal
            else:
                new_sample = self.get_sample(env)
            if env.is_free(new_sample):
                nn_node = self.get_nearest_neighbor(new_sample) #nearest neighboring node
                edge = (nn_node, new_sample)
                if self.path_free(edge, env):
                    self.V.append(new_sample)
                    self.E.append(edge)
            counter += 1



leons_room = Environment()
leons_room.build_()
n_obstacles = 10
for i in range(n_obstacles):
    x = np.random.choice(np.linspace(0, leons_room.width, num=20), 1)
    y = np.random.choice(np.linspace(0, leons_room.height, num=20), 1)
    position = (int(x),int(y))
    box = Obstacle(pos=position)
    leons_room.add_obstacle(box)
planning_algorithm = RRT()
planning_algorithm.run(leons_room)
leons_room.visualize(planning_algorithm.V, planning_algorithm.E)