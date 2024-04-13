import os

from PacmanAgentBuilder.agents.MyFirstAgent import MyFirstAgent
from PacmanAgentBuilder.agents.HumanAgent import HumanAgent
from PacmanAgentBuilder.agents.AiAgent import SimpleAgent
from PacmanAgentBuilder.utils.runnerFunctions import *

# stops the PyGame hello message from showing
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"


if __name__ == "__main__":
    for i in range(100):
        stats = calculatePerformanceOverXGames(
            agentClass=SimpleAgent,  # Specify the agent to be evaluated.
            gameCount=100,  # Number of games the agent will play.
            gameSpeed=5,  # Sets the speed of the game from 0.1 (slow) to 5 (fast). For higher speeds, enable lockDeltaTime.
            startLevel=0,  # Choose the starting level for the agent (0 for level one, 1 for level two, and so on).
            ghostsEnabled=True,  # Toggle ghosts on or off.
            freightEnabled=True,  # Toggle if the effect of power pellets should be ignored.
            lockDeltaTime=True,  # When enabled, the game will run at the highest possible speed.
            logging=False,  # Toggle the logging of game-related information to the console while the agent is playing.
            disableVisuals=True
        )
