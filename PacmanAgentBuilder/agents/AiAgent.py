import math
import random

from PacmanAgentBuilder.agents.Iagent import IAgent
from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.vector import Vector2


def round_to_nearest(number, round_to_this):
    return round(number / round_to_this) * round_to_this


def distance(vector1, vector2):
    return math.sqrt((vector1.x - vector2.x) ** 2 + (vector1.y - vector2.y) ** 2)


def closest_vector(vector_list, target_vector):
    if not vector_list:
        return None

    closest = vector_list[0]
    min_distance = distance(vector_list[0], target_vector)

    for vector in vector_list[1:]:
        dist = distance(vector, target_vector)
        if dist < min_distance:
            min_distance = dist
            closest = vector

    return closest


class SimpleAgent(IAgent):
    """
    This is a simple agent that allows an AI to play the game.
    """

    def __init__(self, gameController, qtable=None):
        super().__init__(gameController, qtable)
        self.q_table_manager = qtable
        self.score = 0
        self.maxLives = 3
        self.currentLives = 3
        self.num_moves = 0  # Track the number of moves
        self.pelletsEaten = 0
        self.last_rounded_pos = Vector2(0, 0)
        self.currentLevel = 0
        self.random_choice = 0

        self.q_table = qtable.getQTable()
        # self.q_tableNew = qtable.getNewTable()

        # Q-table initialization
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.epsilon = 0.001
        self.bad_notes = [Vector2(230.0, 320), Vector2(270.0, 280), Vector2(310.0, 320), Vector2(230.0, 340),
                          Vector2(270.0, 340), Vector2(310.0, 340), Vector2(230.0, 360), Vector2(310.0, 360)]

    def calculateDoMove(self, obs: Observation, pos):
        b = obs.getPacmanPosition() == obs.getPacmanTargetPosition() # on corners
        b = b or self.num_moves == 0 # on first move
        self.num_moves += 1
        b = b or self.num_moves % 30 == 0 # periodically
        b = b or self.pelletsEaten < self.gameController.pellets.numEaten # on pellet eaten
        b = b or self.currentLevel < self.gameController.level # on level change
        b = b or obs.getLives() < self.currentLives # on life lost
        b = b and obs.getPacmanPosition() not in self.bad_notes # but not when the game will crash
        return b

    def calculateNextMove(self, obs: Observation):
        round_to = 10
        rounded_pos = Vector2(round_to_nearest(obs.getPacmanPosition().x, round_to),
                              round_to_nearest(obs.getPacmanPosition().y, round_to))
        if self.calculateDoMove(obs, rounded_pos):
            if self.pelletsEaten < self.gameController.pellets.numEaten:
                self.epsilon = 0.001
            self.last_rounded_pos = rounded_pos
            self.currentLives = obs.getLives()
            self.state = self.encode_state(obs, rounded_pos, round_to)

            # Update Q-value for previous state-action pair
            if hasattr(self, 'prev_state') and hasattr(self, 'prev_action'):
                reward = self.calculate_reward(obs)
                self.update_q_table(self.prev_state, self.prev_action, reward, self.state, obs)

            # Exploration-exploitation trade-off
            # print(random.uniform(0, 1))
            if random.uniform(0, 1) < self.epsilon:
                action = random.choice(obs.getLegalMoves())
                self.random_choice += 1
            else:
                action = self.choose_best_action(self.state, obs)
                self.epsilon += 0.001

            # Store previous state-action pair
            self.prev_state = self.state
            self.prev_action = action
            if self.gameController.level > 3:
                self.gameController.endGame()
            return action

        return 0

    def encode_state(self, obs: Observation, rounded_pos, round_to):
        # Encode the current state based on relevant information such as Pac-Man's position, ghost positions, etc.
        # For simplicity, let's use a tuple to represent the state
        pacman_position = str(rounded_pos)
        ghost_positions = []
        for i in obs.getGhostPositions():
            if (not (230 <= i.x <= 310 and 320 <= i.y <= 360)) and distance(obs.getPacmanPosition(), i) < 200:
                ghost_positions.append(str(Vector2(round_to_nearest(i.x, round_to), round_to_nearest(i.y, round_to))))
        pellet = str(closest_vector(obs.getPelletPositions(), obs.getPacmanPosition()))
        encoded_state = (pacman_position, pellet, tuple(ghost_positions), self.gameController.level % 2)
        return encoded_state

    def calculate_reward(self, obs: Observation):
        # Calculate reward based on the change in pellet eaten and lives
        reward = self.gameController.pellets.numEaten - self.pelletsEaten
        reward *= 10
        self.pelletsEaten = self.gameController.pellets.numEaten
        if obs.getLives() < self.currentLives:
            # Large punishment if a life is lost (10x pellet)
            reward -= 100 * self.currentLives - obs.getLives()
            self.currentLives = obs.getLives()
        if self.gameController.level > self.currentLevel:
            # Large reward on level completion (5x pellet)
            reward += 50
            self.currentLevel = self.gameController.level
        return reward

    def update_q_table(self, state, action, reward, next_state, obs):
        # Update Q-value using Q-learning update rule
        if (state, action) not in self.q_table:
            self.q_table[(state, action)] = 0
        if next_state is None:
            max_q_value = 0
        else:
            max_q_value = max([self.q_table.get((next_state, a), 0) for a in obs.getLegalMoves()])
        self.q_table[(state, action)] += self.learning_rate * (
                reward + self.discount_factor * max_q_value - self.q_table[(state, action)])

    def choose_best_action(self, state, obs):
        # Choose the action with the highest Q-value for the given state
        legal_moves = obs.getLegalMoves()
        q_values = [self.q_table.get((state, a), 0) for a in legal_moves]
        max_q_value = max(q_values)
        best_actions = [legal_moves[i] for i in range(len(legal_moves)) if q_values[i] == max_q_value]
        return random.choice(best_actions)
