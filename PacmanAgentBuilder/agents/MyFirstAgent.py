import math
import random
from time import sleep

import pygame
from pygame import K_UP, K_DOWN, K_LEFT, K_RIGHT

from PacmanAgentBuilder.agents.Iagent import IAgent
from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.constants import *
from Pacman_Complete.vector import Vector2
import random
import heapq


class MyFirstAgent(IAgent):
    """
    for documentation see https://github.com/SpicyOverlord/PacmanAgentBuilder
    """

    def __init__(self, gameController):
        super().__init__(gameController)
        self.pacman_node = None
        self.bad_notes = []
        self.not_visited_notes = None
        self.goal = None
        self.clean_graph = False
        self.graph_nodes = None

    def make_graph_from_nodes(self, obs: Observation):
        self.clean_graph = True
        self.graph_nodes = [graph_node(node) for node in obs.nodeGroup.nodesLUT.values()]
        bad_notes = [Vector2(230, 360), Vector2(310, 360), Vector2(230, 320), Vector2(310, 320), Vector2(230, 340),
                     Vector2(270, 340), Vector2(310, 340)]
        for i in bad_notes:
            node = self.get_graph_node_from_node(obs.getNodeFromVector(i))
            self.bad_notes.append(node)
            self.graph_nodes.remove(node)

    def get_graph_node_from_node(self, node):
        for i in self.graph_nodes:
            if node == i.node:
                return i

    def find_safe_factors(self, obs: Observation):
        for i in obs.getGhosts():
            target = self.get_graph_node_from_node(i.target)
            if target is not None:
                dijkstra(obs, target, heuristic_ghost, ghost_func_on_node)

    def calculateNextMove(self, obs: Observation):
        # uncomment this to draw the graph of the current level to the screen:
        DebugHelper.drawMap(obs)

        # sleep(0.01)
        pacmanPosition = obs.getPacmanPosition()
        pacmanTarget = obs.getPacmanTargetPosition()

        # draw a purple line from pacman to pacmans target
        DebugHelper.drawLine(pacmanPosition, pacmanTarget, DebugHelper.PURPLE, 5)
        # if pacman is on a node, use pathfinding to find next direction
        if pacmanPosition == pacmanTarget:
            self.make_graph_from_nodes(obs)
            for node in self.graph_nodes:
                node.set_neighbors(self)
            self.find_safe_factors(obs)
            self.pacman_node = self.get_graph_node_from_node(obs.getNodeFromVector(pacmanTarget))
            self.goal = decision_maker(obs, self)
            dijkstra(obs, self.pacman_node, heuristic_safest, pacman_func_on_node)
            next_node = get_next_node(self.pacman_node, self.goal)
            return next_node.direction_from_previous
            # return random.choice([UP, DOWN, LEFT, RIGHT])
        if self.goal is not None:
            next_node = self.goal
            if next_node.previous is not None:
                while next_node.previous != self.pacman_node:
                    DebugHelper.drawLine(next_node.node.position, next_node.previous.node.position, DebugHelper.GREEN, )
                    next_node = next_node.previous
        # you need to return UP, DOWN, LEFT, RIGHT or STOP (where STOP means you don't change direction)
        return STOP


def get_distance_between_vectors(vector1: Vector2, vector2: Vector2) -> float:
    distance = 0

    distance += (vector1.y - vector2.y) ** 2
    distance += (vector1.x - vector2.x) ** 2

    return math.sqrt(distance)


def reverse_direction(direction):
    if direction == UP:
        return DOWN
    if direction == DOWN:
        return UP
    if direction == LEFT:
        return RIGHT
    if direction == RIGHT:
        return LEFT
    return STOP


n1 = Vector2(20, 80)
n2 = Vector2(520, 640)


def decision_maker(obs: Observation, agent):
    # safest = agent.graph_nodes[0]
    # for i in agent.graph_nodes:
    #     if i.safe_factor > safest.safe_factor:
    #         safest = i
    # return safest

    return random.choice(agent.graph_nodes)



class graph_node:
    def __init__(self, node):
        self.previous = None
        self.direction_from_previous = STOP
        self.distance = 0
        self.node = node
        self.neighbors = None
        self.safe_factor = 0

    def set_neighbors(self, agent):
        self.neighbors = {}
        for neighbor in self.node.neighbors:
            if self.node.neighbors[neighbor] is not None:
                self.neighbors[neighbor] = agent.get_graph_node_from_node(self.node.neighbors[neighbor])

    def set_previous(self, previous, direction, distance):
        self.previous = previous
        self.direction_from_previous = direction
        self.distance = distance

    def __lt__(self, other):
        return self.distance < other.distance

    def __eq__(self, other):
        return str(self.node) == str(other.node)

    def __str__(self):
        return str(self.node)

    def add_to_safe_factor(self, dist):
        self.safe_factor += dist*10


def dijkstra(obs: Observation, start_node: graph_node, heuristic_func: callable, func_on_node: callable):
    start_node.distance = 0
    seen_nodes = [start_node]
    queue = [start_node]
    func_on_node(start_node, None, 0, STOP)
    while queue:
        current_node = heapq.heappop(queue)
        for direction in current_node.neighbors:
            neighbor = current_node.neighbors[direction]
            if neighbor is not None:
                if neighbor not in seen_nodes:
                    distance = current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position)
                    if direction == 3:
                        distance = current_node.distance
                        direction = 0
                    distance += heuristic_func(obs, neighbor)
                    func_on_node(neighbor, current_node, distance, direction)
                    seen_nodes.append(neighbor)
                    queue.append(neighbor)
                elif neighbor.distance > current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position):
                    distance = current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position)
                    if direction == 3:
                        distance = current_node.distance
                        direction = 0
                    distance += heuristic_func(obs, neighbor)
                    func_on_node(neighbor, current_node, distance, direction)


def get_next_node(start_node: graph_node, goal: graph_node):
    # print("____________________________________________")
    next_node = goal
    # print(next_node)
    if next_node.previous is not None:
        while next_node.previous != start_node:
            #DebugHelper.drawLine(next_node.node.position, next_node.previous.node.position, DebugHelper.GREEN, )
            next_node = next_node.previous
            # print(next_node)
            # print(next_node.distance)
    return next_node


def a_star(pbs: Observation, goal: Vector2) -> Vector2:
    pass


def heuristic_shortest(obs: Observation, pos: Vector2) -> int:
    pass


def heuristic_safest(obs: Observation, node: graph_node) -> float:
    if node.safe_factor > 0:
        safety = (1/node.safe_factor) * 1000000
    else:
        safety = 1000000
    # print(node)
    # print(safety)
    return safety


def heuristic_ghost(obs: Observation, node: graph_node) -> float:
    return 0


def pacman_func_on_node(node: graph_node, prev: graph_node, dist, direction):
    node.set_previous(prev, direction, dist)


def ghost_func_on_node(node: graph_node, prev: graph_node, dist, direction):
    node.add_to_safe_factor(dist)
