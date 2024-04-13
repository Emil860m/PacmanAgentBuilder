from abc import abstractmethod, ABC

from PacmanAgentBuilder.utils.observation import Observation


class IAgent(ABC):
    """
    THis is the interface for the agent. All agents must implement this interface.
    """
    @abstractmethod
    def __init__(self, gameController, qtable):
        self.gameController = gameController
        self.actionsTaken = 0
        self.pelletsEatenThisLevel = 0


    @abstractmethod
    def calculateNextMove(self, obs: Observation):
        raise Exception("NotImplementedException")

    def takeStats(self):
        self.actionsTaken += 1
        self.pelletsEatenThisLevel = self.gameController.pellets.numEaten
        pass

