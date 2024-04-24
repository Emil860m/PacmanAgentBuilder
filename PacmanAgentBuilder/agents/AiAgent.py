import math
import random

from PacmanAgentBuilder.agents.Iagent import IAgent
from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.vector import Vector2


def round_to_nearest_ten(number):
    return round(number / 10) * 10


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

        self.q_table = qtable.getQTable()
        self.q_tableNew = qtable.getNewTable()

        # Q-table initialization
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.epsilon = 0.1
        self.bad_notes = [Vector2(270.0, 280), Vector2(230.0, 320), Vector2(310.0, 320), Vector2(230.0, 340),
                          Vector2(270.0, 340), Vector2(310.0, 340), Vector2(230.0, 360), Vector2(310.0, 360)]

    def calculateDoMove(self, obs: Observation):
        b = obs.getPacmanPosition() == obs.getPacmanTargetPosition()
        b = b or self.num_moves % 30 == 0
        b = b or obs.getLives() < self.currentLives
        b = b or obs.getPacmanPosition() == Vector2(263, 520)
        b = b and obs.getPacmanPosition() not in self.bad_notes
        return b


    def calculateNextMove(self, obs: Observation):
        self.num_moves += 1  # Increment the number of moves
        if self.calculateDoMove(obs):
            self.num_moves = 0
            self.currentLives = obs.getLives()
            self.state = self.encode_state(obs)

            # Update Q-value for previous state-action pair
            if hasattr(self, 'prev_state') and hasattr(self, 'prev_action'):
                reward = self.calculate_reward(obs)
                self.update_q_table(self.prev_state, self.prev_action, reward, self.state, obs)

            # Exploration-exploitation trade-off
            if random.uniform(0, 1) < self.epsilon:
                action = random.choice(obs.getLegalMoves())
            else:
                action = self.choose_best_action(self.state, obs)

            # Store previous state-action pair
            self.prev_state = self.state
            self.prev_action = action

            return action
        return 0

    def encode_state(self, obs: Observation):
        # Encode the current state based on relevant information such as Pac-Man's position, ghost positions, etc.
        # For simplicity, let's use a tuple to represent the state
        pacman_position = Vector2(round_to_nearest_ten(obs.getPacmanPosition().x), round_to_nearest_ten(obs.getPacmanPosition().y))
        ghost_positions = []
        for i in obs.getGhostPositions():
            ghost_positions.append(Vector2(round_to_nearest_ten(i.x), round_to_nearest_ten(i.y)))
        # pellet_positions = []
        # pellet_positions = obs.getPelletPositions()
        closest_pellet = closest_vector(obs.getPelletPositions(), obs.getPacmanPosition())
        encoded_state = (pacman_position, tuple(ghost_positions), closest_pellet)
        return encoded_state

    def calculate_reward(self, obs: Observation):
        # Calculate reward based on the change in score and lives
        reward = obs.getScore() - self.score
        self.score = obs.getScore()
        if obs.getLives() < self.currentLives:
            # Large punishment if a life is lost
            reward -= 1000 * self.currentLives - obs.getLives()
            self.currentLives = obs.getLives()
        reward -= self.num_moves / 10
        return reward

    def update_q_table(self, state, action, reward, next_state, obs):
        # Update Q-value using Q-learning update rule
        if (state, action) not in self.q_tableNew:
            self.q_tableNew[(state, action)] = 0
        if next_state is None:
            max_q_value = 0
        else:
            max_q_value = max([self.q_tableNew.get((next_state, a), 0) for a in obs.getLegalMoves()])
        self.q_tableNew[(state, action)] += self.learning_rate * (
                reward + self.discount_factor * max_q_value - self.q_tableNew[(state, action)])

    def choose_best_action(self, state, obs):
        # Choose the action with the highest Q-value for the given state
        legal_moves = obs.getLegalMoves()
        q_values = [self.q_table.get((state, a), 0) for a in legal_moves]
        max_q_value = max(q_values)
        best_actions = [legal_moves[i] for i in range(len(legal_moves)) if q_values[i] == max_q_value]
        return random.choice(best_actions)

class TensorFlowAgent(IAgent):
    """
    This is a simple agent implemented with tensorflow that allows an AI to play the game.
    """

    def __init__(self, gameController, qtable=None):
        super().__init__(gameController, qtable)

    def calculateNextMove(self, obs: Observation):
        return random.choice(obs.getLegalMoves())
