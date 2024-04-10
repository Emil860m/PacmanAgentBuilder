import random

import pygame
from pygame import K_UP, K_DOWN, K_LEFT, K_RIGHT

from PacmanAgentBuilder.agents.Iagent import IAgent
from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.constants import *


class AiAgent(IAgent):
    """
    This is a simple agent that allows an AI to play the game.
    """
    def __init__(self, gameController):
        super().__init__(gameController)

    def calculateNextMove(self, obs: Observation):
        print(obs.getPacmanPosition())
        DebugHelper.pauseGame()
        return random.choice(obs.getLegalMoves())
