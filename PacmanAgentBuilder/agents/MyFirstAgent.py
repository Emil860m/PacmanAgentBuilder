import math
import random
from time import sleep
import copy

import pygame
from pygame import K_UP, K_DOWN, K_LEFT, K_RIGHT

from PacmanAgentBuilder.agents.Iagent import IAgent
from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.constants import *
from Pacman_Complete.vector import Vector2
import random
import heapq
first_game = True

class MyFirstAgent(IAgent):
    """
    for documentation see https://github.com/SpicyOverlord/PacmanAgentBuilder
    """

    def __init__(self, gameController):
        super().__init__(gameController)
        self.pacman_previous = None
        self.vertex_unknown = True
        self.graph = False
        self.direction = LEFT
        self.pacman_target = None
        self.pacman_vertex = None
        self.not_visited_vertices = []
        self.goal = None
        self.graph_nodes = []
        self.graph_vertices = []

    def make_graph_from_nodes(self, obs: Observation):
        self.graph = True
        self.graph_nodes = [graph_node(node) for node in obs.nodeGroup.nodesLUT.values()]
        bad_notes = [Vector2(230, 360), Vector2(310, 360), Vector2(230, 320), Vector2(310, 320), Vector2(230, 340),
                     Vector2(270, 340), Vector2(310, 340)]
        for i in bad_notes:
            node = self.get_graph_node_from_node(obs.getNodeFromVector(i))
            self.graph_nodes.remove(node)
        for node in self.graph_nodes:
            node.set_neighbors(self)
        for node in self.graph_nodes:
            for neighbor in node.neighbors:
                if node.neighbors[neighbor] is not None:
                    v = graph_vertex(node, node.neighbors[neighbor])
                    if v not in self.graph_vertices:
                        self.graph_vertices.append(v)
                    if v not in node.vertices.values():
                        node.vertices[neighbor] = v
                    if v not in node.neighbors[neighbor].vertices.values():
                        node.neighbors[neighbor].vertices[reverse_direction(neighbor)] = v

    def get_graph_node_from_node(self, node):
        for i in self.graph_nodes:
            if node == i.node:
                return i

    def find_pacman_target_and_vertex(self, obs: Observation, pacmanTarget, pacmanPosition):
        self.pacman_target = self.get_graph_node_from_node(obs.getNodeFromVector(pacmanTarget))
        self.pacman_vertex = get_pacman_vertex(pacmanPosition, self.pacman_target, self.pacman_vertex)
        if self.pacman_vertex is not None:
            self.pacman_previous = self.pacman_vertex.get_other(self.pacman_target)
        else:
            self.pacman_previous = None
        self.vertex_unknown = False

    def find_safe_factors(self, obs: Observation):
        for i in self.graph_nodes:
            i.closest_ghost_nodes = 100
        for i in obs.getGhosts():
            target = self.get_graph_node_from_node(i.target)
            if target is not None:
                dijkstra(obs, target, None, heuristic_ghost, ghost_func_on_node, self, i.position)

    def calculateNextMove(self, obs: Observation):
        global first_game
        if first_game:
            DebugHelper.pauseGame()
            first_game = False
        # At the start of the game, and if we beat a level, we create a graph and set a goal
        if len(obs.pelletGroup.pelletList) == 244 and not self.graph:
            self.make_graph_from_nodes(obs)
            self.goal = self.get_graph_node_from_node(obs.getNodeFromVector(Vector2(520, 640)))


        # Find the safe factors from the ghosts positions
        self.find_safe_factors(obs)

        pacmanPosition = obs.getPacmanPosition()
        pacmanTarget = obs.getPacmanTargetPosition()

        # Set the pacman position, target and previous nodes
        self.find_pacman_target_and_vertex(obs, pacmanTarget, pacmanPosition)
        if pacmanPosition == pacmanTarget:
            self.graph = False
            self.pacman_previous = None
            self.pacman_vertex = None

        # Finding out where to go (decision tree)
        self.goal = goal_decider(obs, self)
        # If flee was decided
        if isinstance(self.goal, str):
            direction = flee(obs, self)
            if not direction == 3:
                self.direction = direction
            return self.direction
        # If chasing pellets, a goal node was chosen
        dijkstra(obs, self.pacman_target, self.pacman_previous, heuristic_ghost, None, self,
                 pacmanPosition)
        # Now we found out which direction to go to get to the goal node
        next_node = get_next_node(self.goal, self.pacman_target,
                                  self.get_graph_node_from_node(obs.getNodeFromVector(pacmanPosition)))
        # Issue with portals
        if not next_node.direction_from_previous == 3:
            self.direction = next_node.direction_from_previous

        return self.direction


class graph_node:
    def __init__(self, node):
        self.previous = None
        self.direction_from_previous = STOP
        self.distance = 100000
        self.node = node
        self.neighbors = None
        self.safe_factor = 0
        self.vertices = {}
        self.closest_ghost_nodes = 100
        self.position = self.node.position

    def clean(self):
        self.distance = 100000
        self.previous = None
        self.direction_from_previous = STOP

    def add_to_vertices(self, vertex, direction):
        if self.vertices[direction]:
            TypeError()
        self.vertices[direction] = vertex

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
        self.safe_factor += dist


class graph_vertex:
    def __init__(self, node1: graph_node, node2: graph_node):
        self.node1 = node1
        self.node2 = node2
        self.distance = get_distance_between_vectors(node1.node.position, node2.node.position)

    def __eq__(self, other):
        return (self.node1 == other.node1 and self.node2 == other.node2) or (
                self.node1 == other.node2 and self.node2 == other.node1)

    def get_other(self, node):
        if node == self.node1:
            return self.node2
        if node == self.node2:
            return self.node1
        ValueError()


def flee(obs: Observation, agent: MyFirstAgent):
    dist = 300
    considered_ghosts = []
    for i in obs.getGhosts():
        if get_distance_between_vectors(i.position, obs.getPacmanPosition()) < dist and agent.get_graph_node_from_node(
                i.target) is not None:
            considered_ghosts.append(i)
    best_dir = agent.direction
    value = -1
    for i in considered_ghosts:
        vertex_nodes = [agent.pacman_target]
        if agent.pacman_previous is not None:
            vertex_nodes.append(agent.pacman_previous)
        ghost_target = agent.get_graph_node_from_node(i.target)
        if ghost_target is not None:
            if agent.get_graph_node_from_node(i.target) in vertex_nodes:
                check_if_same_vertex = find_direction_from_positions(obs.getPacmanPosition(), i.position)
                if check_if_same_vertex:
                    if obs.getPacmanPosition() == obs.getPacmanTargetPosition():
                        for j in agent.pacman_target.neighbors:
                            if not j == check_if_same_vertex:
                                if agent.pacman_target.neighbors[j].closest_ghost_nodes > value:
                                    best_dir = j
                                    value = agent.pacman_target.neighbors[j].closest_ghost_nodes
                    else:
                        best_dir = reverse_direction(check_if_same_vertex)
                else:
                    if obs.getPacmanPosition() == obs.getPacmanTargetPosition():
                        if agent.pacman_target.closest_ghost_nodes < 2:
                            for j in agent.pacman_target.neighbors:
                                if agent.pacman_target.neighbors[j].closest_ghost_nodes > value:
                                    best_dir = j
                                    value = agent.pacman_target.neighbors[j].closest_ghost_nodes
                    else:
                        if agent.pacman_previous.closest_ghost_nodes > agent.pacman_target.closest_ghost_nodes:
                            best_dir = reverse_direction(agent.direction)
            else:
                if obs.getPacmanPosition() == obs.getPacmanTargetPosition():
                    for j in agent.pacman_target.neighbors:
                        if agent.pacman_target.neighbors[j] is None:
                            continue
                        if agent.pacman_target.neighbors[j].closest_ghost_nodes > value:
                            best_dir = j
                            value = agent.pacman_target.neighbors[j].closest_ghost_nodes
                else:
                    pass
                    # if agent.pacman_previous is not None:
                    #     if agent.pacman_previous.closest_ghost_nodes > agent.pacman_target.closest_ghost_nodes:
                    #         best_dir = reverse_direction(agent.direction)
    return best_dir


def get_distance_between_vectors(vector1: Vector2, vector2: Vector2) -> float:
    v1 = [vector1.x, vector1.y]
    v2 = [vector2.x, vector2.y]
    return math.dist(v1, v2)


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


def find_closest_node(position: Vector2, nodes, exclusions=None):
    if exclusions is None:
        exclusions = []
    dist = 10000
    node = None
    for i in nodes:
        if i in exclusions:
            continue
        other_dist = get_distance_between_vectors(i.position, position)
        if dist > other_dist:
            dist = other_dist
            node = i
    return node


def find_direction_from_positions(position, target):
    if position.x == target.x and position.y == target.y:
        return 0
    if position.x == target.x:
        if position.y > target.y:
            return UP
        return DOWN
    elif position.y == target.y:
        if position.x > target.x:
            return LEFT
        return RIGHT
    return 0


pellet = None
exclusions = []


def get_pellet(obs, agent):
    global pellet, exclusions
    # Find closest pellet
    pellet = find_closest_node(obs.getPacmanPosition(), obs.pelletGroup.pelletList)
    pellet_node = find_closest_node(pellet.position, agent.graph_nodes)
    excl = []
    # If pellet is to unsafe, find another
    while pellet_node.closest_ghost_nodes < 2:
        excl.append(pellet)
        pellet = find_closest_node(obs.getPacmanPosition(), obs.pelletGroup.pelletList, excl)
        pellet_node = find_closest_node(pellet.position, agent.graph_nodes)
    # If we can reach pellet by going straight, we do that
    if find_direction_from_positions(obs.getPacmanPosition(), pellet.position):
        return agent.pacman_target
    pellet_dir = find_direction_from_positions(pellet_node.position, pellet.position)
    pellet_node = find_closest_node(pellet.position, agent.graph_nodes, exclusions)
    # This should not happen, but caused issues
    if pellet_dir == 0:
        return pellet_node
    # If pellet is not directly next to the closest node, we check 2nd closest node and continue until we find a node
    # that is directly next to it
    while pellet_dir not in pellet_node.neighbors.keys():
        exclusions.append(pellet_node)
        pellet_node = find_closest_node(pellet.position, agent.graph_nodes, exclusions)
        pellet_dir = find_direction_from_positions(pellet_node.position, pellet.position)
    if get_distance_between_vectors(pellet_node.neighbors[pellet_dir].position,
                                    pellet.position) < get_distance_between_vectors(pellet_node.position,
                                                                                    pellet.position):
        return pellet_node.neighbors[pellet_dir]
    return pellet_node


def goal_decider(obs: Observation, agent: MyFirstAgent):
    global pellet, exclusions
    # If pacman is not threatened
    if ((agent.pacman_target.closest_ghost_nodes > 2)
            or (agent.pacman_previous is not None and (agent.pacman_previous.closest_ghost_nodes > 2))):
        # Pellet is none or has been eaten
        if pellet not in obs.pelletGroup.pelletList:
            exclusions = []
            return get_pellet(obs, agent)
        # We reached the node closest to the pellet without eating it
        # We go the direction the pellet is from where we are
        # At this point we already know the node is directly connected to pellet (See get_pellet())
        if agent.goal.position == obs.getPacmanPosition():
            if pellet in obs.pelletGroup.pelletList:
                direction = find_direction_from_positions(obs.getPacmanPosition(), pellet.position)
                pellet_node = agent.pacman_target
                # Not sure what this does, but it performs serverly worse without it
                while direction not in pellet_node.neighbors.keys():
                    exclusions.append(pellet_node)
                    pellet_node = find_closest_node(pellet.position, agent.graph_nodes, exclusions)
                    direction = find_direction_from_positions(pellet_node.position, pellet.position)
                if not direction == 0:
                    return pellet_node.neighbors[direction]
                return pellet_node
            return get_pellet(obs, agent)
        return agent.goal
    # else
    else:
        pellet = None
        exclusions = None
        return "flee"


def get_pacman_vertex(pacman_position, pacman_target: graph_node, pacman_vertex) -> graph_vertex:
    for i in pacman_target.neighbors:
        if pacman_target.neighbors[i] is not None:
            if pacman_target.neighbors[i].node.position.x == pacman_position.x:
                if i == UP:
                    if pacman_target.node.position.y > pacman_position.y:
                        return pacman_target.vertices[i]
                else:
                    if pacman_target.node.position.y < pacman_position.y:
                        return pacman_target.vertices[i]
            elif pacman_target.neighbors[i].node.position.y == pacman_position.y:
                if i == LEFT:
                    if pacman_target.node.position.x > pacman_position.x:
                        return pacman_target.vertices[i]
                elif i == RIGHT:
                    if pacman_target.node.position.x < pacman_position.x:
                        return pacman_target.vertices[i]
                elif i == PORTAL:
                    pass
    return None


def dijkstra(obs: Observation, start_node: graph_node, start_node_other: graph_node, heuristic_func: callable,
             func_on_node: callable, agent, position, comparator: callable = None):
    # Cleaning nodes as we use this function and the same distance variables each time we run this
    for i in agent.graph_nodes:
        i.clean()
    # Rest of the code is standard dijkstra, but allowing to add 2 start_nodes
    # as pacman should pathfind from both pacman can reach
    # Also allowing a heuristic function, that ended up not being used
    seen_nodes = [start_node]
    queue = [start_node]
    pacman_func_on_node(start_node, None, get_distance_between_vectors(start_node.node.position, position),
                        agent.direction)
    if func_on_node is not None:
        func_on_node(start_node, None, get_distance_between_vectors(start_node.node.position, position),
                     agent.direction)
    if start_node_other is not None:
        pacman_func_on_node(start_node_other, None,
                            get_distance_between_vectors(start_node_other.node.position, position),
                            reverse_direction(agent.direction))
        if func_on_node is not None:
            func_on_node(start_node_other, None, get_distance_between_vectors(start_node_other.node.position, position),
                         reverse_direction(agent.direction))
        seen_nodes.append(start_node_other)
        heapq.heappush(queue, start_node_other)

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
                    # distance += heuristic_func(obs, neighbor)
                    pacman_func_on_node(neighbor, current_node, distance, direction)
                    if func_on_node is not None:
                        func_on_node(neighbor, current_node, distance, direction)
                    seen_nodes.append(neighbor)
                    heapq.heappush(queue, neighbor)
                elif neighbor.distance > current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position):
                    distance = current_node.distance + get_distance_between_vectors(
                        current_node.node.position, neighbor.node.position)
                    if direction == 3:
                        distance = current_node.distance
                        direction = 0
                    # distance += heuristic_func(obs, neighbor)
                    pacman_func_on_node(neighbor, current_node, distance, direction)
                    if func_on_node is not None:
                        func_on_node(neighbor, current_node, distance, direction)


def get_next_node(goal: graph_node, target_node: graph_node, pacman_node):
    next_node = goal
    while next_node.previous is not None and next_node.previous is not pacman_node:
        next_node = next_node.previous

    return next_node


def heuristic_ghost(obs: Observation, node: graph_node) -> float:
    return 0


def pacman_func_on_node(node: graph_node, prev: graph_node, dist, direction):
    node.set_previous(prev, direction, dist)


def ghost_func_on_node(node: graph_node, prev: graph_node, dist, direction):
    if prev is not None:
        if node.closest_ghost_nodes > prev.closest_ghost_nodes + 1:
            node.closest_ghost_nodes = prev.closest_ghost_nodes + 1
    else:
        node.closest_ghost_nodes = 0
    node.add_to_safe_factor(dist)
