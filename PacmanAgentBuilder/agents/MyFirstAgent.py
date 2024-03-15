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
            self.graph_nodes.remove(self.get_graph_node_from_node(obs.getNodeFromVector(i)))

    def get_graph_node_from_node(self, node):
        for i in self.graph_nodes:
            if node == i.node:
                return i

    def calculateNextMove(self, obs: Observation):
        # uncomment this to draw the graph of the current level to the screen:
        DebugHelper.drawMap(obs)
        if not self.clean_graph:
            self.make_graph_from_nodes(obs)
            self.not_visited_notes = self.graph_nodes.copy()
            self.goal = random.choice(self.not_visited_notes)
            for node in self.graph_nodes:
                node.set_neighbors(self)
        # sleep(0.01)
        pacmanPosition = obs.getPacmanPosition()
        pacmanTarget = obs.getPacmanTargetPosition()
        # some code to make pacman run to all possible nodes in a random order. Will be replaced with decision tree
        if self.get_graph_node_from_node(
                obs.getNodeFromVector(pacmanPosition)) is not None and self.get_graph_node_from_node(
                obs.getNodeFromVector(pacmanPosition)) == self.goal:
            self.not_visited_notes.remove(self.goal)
            if len(self.not_visited_notes) == 0:
                self.not_visited_notes = self.graph_nodes.copy()
            self.goal = random.choice(self.not_visited_notes)
        # draw a purple line from pacman to pacman's target
        DebugHelper.drawLine(pacmanPosition, pacmanTarget, DebugHelper.PURPLE, 5)
        # if pacman is on a node, use pathfinding to find next direction
        if pacmanPosition == pacmanTarget:
            next_node = dijkstra(obs, self.get_graph_node_from_node(obs.getNodeFromVector(pacmanTarget)), self.goal)
            return next_node.direction_from_previous
            # return random.choice([UP, DOWN, LEFT, RIGHT])

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


def decision_tree(osb: Observation) -> Vector2:
    pass


def goal_to_direction_translator(goal: Vector2, direction: Vector2) -> int:
    pass


class graph_node:
    def __init__(self, node):
        self.previous = None
        self.direction_from_previous = None
        self.distance = 1000000
        self.node = node
        self.neighbors = None

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


def dijkstra(obs: Observation, start_node: graph_node, goal: graph_node) -> graph_node:
    start_node.distance = 0
    seen_nodes = [start_node]
    queue = [start_node]
    while queue:
        current_node = heapq.heappop(queue)
        for direction in current_node.neighbors:
            neighbor = current_node.neighbors[direction]
            if neighbor is not None:
                if neighbor not in seen_nodes:
                    neighbor.set_previous(current_node, direction, current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position))
                    seen_nodes.append(neighbor)
                    queue.append(neighbor)
                elif neighbor.distance > current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position):
                    neighbor.set_previous(current_node, direction, current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position))
    next_node = goal
    while next_node.previous != start_node:
        next_node = next_node.previous
    return next_node
    # goal_node = obs.getNodeFromVector(goal)


def a_star(pbs: Observation, goal: Vector2) -> Vector2:
    pass


def heuristic_shortest(obs: Observation, pos: Vector2) -> int:
    pass


def heuristic_safest(obs: Observation, pos: Vector2) -> int:
    pass
