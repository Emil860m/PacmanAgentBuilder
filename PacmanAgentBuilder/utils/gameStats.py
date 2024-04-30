from PacmanAgentBuilder.agents.Iagent import IAgent
from Pacman_Complete.run import GameController


class GameStats(object):
    """
    A GameStats object is returned after an agent has finished a game.
    It contains information about the game played (the score, levels completed, pellets eaten, and actions taken).
    """
    def __init__(self, game: GameController, agent: IAgent):
        self.score = game.score
        self.levelsCompleted = game.level - game.startLevel
        self.totalPelletsEaten = self.levelsCompleted * 240 + agent.pelletsEatenThisLevel
        self.actionsTaken = agent.actionsTaken

    def __str__(self):
        return (f"GameStats(score={self.score}, "
                f"totalPelletsEaten={self.totalPelletsEaten}, "
                f"actionsTaken={self.actionsTaken}, "
                f"levelsCompleted={self.levelsCompleted})")

    @staticmethod
    def calculatePerformance(gameStats: list['GameStats']):
        """
        This function acts as a performance evaluator over multiple games.

        :param gameStats: A list of GameStats objects from the games.
        :return: A dictionary containing the calculated performance values.
        """
        baseScores = [game.score for game in gameStats]

        averageBaseScore = sum(baseScores) / len(baseScores)

        baseActions = [agent.actionsTaken for agent in gameStats]

        averageBaseActions = sum(baseActions) / len(baseActions)

        return {"averageScore": round(averageBaseScore, 3), "averageActions": round(averageBaseActions, 1)}

