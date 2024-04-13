import os.path
import subprocess

import pygame
from pygame.locals import *

from PacmanAgentBuilder.utils.debugHelper import DebugHelper
from Pacman_Complete.constants import *
from Pacman_Complete.pacman import Pacman
from Pacman_Complete.nodes import NodeGroup
from Pacman_Complete.pellets import PelletGroup
from Pacman_Complete.ghosts import GhostGroup
from Pacman_Complete.fruit import Fruit
from Pacman_Complete.pauser import Pause
from Pacman_Complete.text import TextGroup
from Pacman_Complete.sprites import LifeSprites, MazeSprites
from Pacman_Complete.mazedata import MazeData
from PacmanAgentBuilder.agents.Iagent import IAgent


class GameController(object):
    def __init__(self, gameSpeed: int,
                 startLives: int,
                 startLevel: int = 0,
                 ghostsEnabled: bool = True,
                 freightEnabled: bool = True,
                 lockDeltaTime: bool = False,
                 disableVisuals: bool = False
                 ):
        if disableVisuals:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            # Disable video subsystem completely
            pygame.display.init()

            # Disable audio subsystem if you don't need audio
            pygame.mixer.quit()
            pygame.mixer.init(frequency=0)

            # Disable font subsystem if you don't need fonts
            pygame.font.quit()
            pygame.font.init()
        else:
            pygame.init()

        self.gameSpeed = gameSpeed
        self.gameOver = False
        self.ghostsEnabled = ghostsEnabled
        self.freightEnabled = freightEnabled
        self.lockDeltaTime = lockDeltaTime
        self.startLevel = startLevel

        self.screen = pygame.display.set_mode(SCREENSIZE, 0, 32)
        DebugHelper.setScreen(self.screen)

        self.background = None
        self.background_norm = None
        self.background_flash = None
        self.clock = pygame.time.Clock()
        self.fruit = None
        self.pause = Pause(False)
        self.level = startLevel
        self.lives = startLives
        self.score = 0
        self.textgroup = TextGroup()
        self.lifesprites = LifeSprites(self.lives)
        self.flashBG = False
        self.flashTime = 0.2
        self.flashTimer = 0
        self.fruitCaptured = []
        self.fruitNode = None
        self.mazedata = MazeData()

    def setBackground(self):
        self.background_norm = pygame.surface.Surface(SCREENSIZE).convert()
        self.background_norm.fill(BLACK)
        self.background_flash = pygame.surface.Surface(SCREENSIZE).convert()
        self.background_flash.fill(BLACK)
        self.background_norm = self.mazesprites.constructBackground(self.background_norm, self.level % 5)
        self.background_flash = self.mazesprites.constructBackground(self.background_flash, 5)
        self.flashBG = False
        self.background = self.background_norm

    def startGame(self, agent: IAgent = None):
        self.agent = agent
        self.textgroup.hideText()

        self.mazedata.loadMaze(self.level)
        self.mazesprites = MazeSprites(
            os.path.join("Pacman_Complete", f"{self.mazedata.obj.name}.txt"),
            os.path.join("Pacman_Complete", f"{self.mazedata.obj.name}_rotation.txt")
        )
        self.setBackground()
        self.nodes = NodeGroup(os.path.join("Pacman_Complete", f"{self.mazedata.obj.name}.txt"))
        self.mazedata.obj.setPortalPairs(self.nodes)
        self.mazedata.obj.connectHomeNodes(self.nodes)
        self.pacman = Pacman(self.nodes.getNodeFromTiles(*self.mazedata.obj.pacmanStart), agent)
        self.pellets = PelletGroup(os.path.join("Pacman_Complete", f"{self.mazedata.obj.name}.txt"))
        self.ghosts = GhostGroup(self.nodes.getStartTempNode(), self.pacman)

        self.ghosts.pinky.setStartNode(self.nodes.getNodeFromTiles(*self.mazedata.obj.addOffset(2, 3)))
        self.ghosts.inky.setStartNode(self.nodes.getNodeFromTiles(*self.mazedata.obj.addOffset(0, 3)))
        self.ghosts.clyde.setStartNode(self.nodes.getNodeFromTiles(*self.mazedata.obj.addOffset(4, 3)))
        self.ghosts.setSpawnNode(self.nodes.getNodeFromTiles(*self.mazedata.obj.addOffset(2, 3)))
        self.ghosts.blinky.setStartNode(self.nodes.getNodeFromTiles(*self.mazedata.obj.addOffset(2, 0)))

        self.nodes.denyHomeAccess(self.pacman)
        self.nodes.denyHomeAccessList(self.ghosts)
        self.ghosts.inky.startNode.denyAccess(RIGHT, self.ghosts.inky)
        self.ghosts.clyde.startNode.denyAccess(LEFT, self.ghosts.clyde)
        self.mazedata.obj.denyGhostsAccess(self.ghosts, self.nodes)

    def update(self):
        if self.lockDeltaTime:
            # dt = 0.034
            dt = 0.05
        else:
            dt = self.clock.tick(30 * self.gameSpeed) / (1000.0 / self.gameSpeed)

        self.textgroup.update(dt)
        self.pellets.update(dt)
        if not self.pause.paused:
            if self.ghostsEnabled:
                self.ghosts.update(dt)

            if self.fruit is not None:
                self.fruit.update(dt)
            self.checkPelletEvents()
            if self.ghostsEnabled:
                self.checkGhostEvents()
            self.checkFruitEvents()

        if self.pacman.alive:
            if not self.pause.paused:
                self.pacman.update(dt)
        else:
            self.pacman.update(dt)

        if self.flashBG:
            self.flashTimer += dt
            if self.flashTimer >= self.flashTime:
                self.flashTimer = 0
                if self.background == self.background_norm:
                    self.background = self.background_flash
                else:
                    self.background = self.background_norm

        afterPauseMethod = self.pause.update(dt)
        if afterPauseMethod is not None:
            afterPauseMethod()
        self.checkEvents()

        # don't draw over the game if paused
        if self.pause.paused:
            return
        if DebugHelper.shouldPause:
            DebugHelper.shouldPause = False
            self.pause.setPause()

        self.render()

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
            elif event.type == KEYDOWN:
                if event.key == K_SPACE:
                    if self.pacman.alive:
                        self.pause.setPause()
                        if not self.pause.paused:
                            self.textgroup.hideText()
                            self.showEntities()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 1 is the left mouse button
                    # Get the mouse position
                    pos = pygame.mouse.get_pos()
                    print(f"Mouse clicked at {pos}")

    def checkPelletEvents(self):
        pellet = self.pacman.eatPellets(self.pellets.pelletList)
        if pellet:
            self.pellets.numEaten += 1
            self.updateScore(pellet.points)
            if self.pellets.numEaten == 30:
                self.ghosts.inky.startNode.allowAccess(RIGHT, self.ghosts.inky)
            if self.pellets.numEaten == 70:
                self.ghosts.clyde.startNode.allowAccess(LEFT, self.ghosts.clyde)
            self.pellets.pelletList.remove(pellet)
            if pellet.name == POWERPELLET:
                if self.freightEnabled:
                    self.ghosts.startFreight()
            if self.pellets.isEmpty():
                self.flashBG = True
                self.hideEntities()
                self.pause.setPause(0, func=self.nextLevel)

    def checkGhostEvents(self):
        for ghost in self.ghosts:
            if self.pacman.collideGhost(ghost):
                if ghost.mode.current is FREIGHT:
                    self.pacman.visible = False
                    ghost.visible = False
                    self.updateScore(ghost.points)
                    self.textgroup.addText(str(ghost.points), WHITE, ghost.position.x, ghost.position.y, 8, time=1)
                    self.ghosts.updatePoints()
                    self.pause.setPause(0, func=self.showEntities)
                    ghost.startSpawn()
                    self.nodes.allowHomeAccess(ghost)
                elif ghost.mode.current is not SPAWN:
                    if self.pacman.alive:
                        self.lives -= 1
                        self.lifesprites.removeImage()
                        self.pacman.die()
                        self.ghosts.hide()
                        if self.lives <= 0:
                            self.pause.setPause(0, func=self.endGame)
                        else:
                            self.pause.setPause(0, func=self.resetLevel)

    def checkFruitEvents(self):
        if self.pellets.numEaten == 50 or self.pellets.numEaten == 140:
            if self.fruit is None:
                self.fruit = Fruit(self.nodes.getNodeFromTiles(9, 20), self.level)
        if self.fruit is not None:
            if self.pacman.collideCheck(self.fruit):
                self.updateScore(self.fruit.points)
                self.textgroup.addText(str(self.fruit.points), WHITE, self.fruit.position.x, self.fruit.position.y, 8,
                                       time=1)
                fruitCaptured = False
                for fruit in self.fruitCaptured:
                    if fruit.get_offset() == self.fruit.image.get_offset():
                        fruitCaptured = True
                        break
                if not fruitCaptured:
                    self.fruitCaptured.append(self.fruit.image)
                self.fruit = None
            elif self.fruit.destroy:
                self.fruit = None

    def showEntities(self):
        self.pacman.visible = True
        self.ghosts.show()

    def hideEntities(self):
        self.pacman.visible = False
        self.ghosts.hide()

    def nextLevel(self):
        self.showEntities()
        self.level += 1
        self.pause.paused = False
        self.startGame(agent=self.agent)
        self.textgroup.updateLevel(self.level)

    def endGame(self):
        self.gameOver = True
        return

    def resetLevel(self):
        # self.pause.paused = True

        self.pacman.reset()
        self.ghosts.reset()
        self.fruit = None

    def updateScore(self, points):
        self.score += points
        self.textgroup.updateScore(self.score)

    def render(self):
        self.screen.blit(self.background, (0, 0))

        self.pellets.render(self.screen)
        if self.fruit is not None:
            self.fruit.render(self.screen)
        self.pacman.render(self.screen)
        if self.ghostsEnabled:
            self.ghosts.render(self.screen)
        self.textgroup.render(self.screen)

        for i in range(len(self.lifesprites.images)):
            x = self.lifesprites.images[i].get_width() * i
            y = SCREENHEIGHT - self.lifesprites.images[i].get_height()
            self.screen.blit(self.lifesprites.images[i], (x, y))

        for i in range(len(self.fruitCaptured)):
            x = SCREENWIDTH - self.fruitCaptured[i].get_width() * (i + 1)
            y = SCREENHEIGHT - self.fruitCaptured[i].get_height()
            self.screen.blit(self.fruitCaptured[i], (x, y))

        DebugHelper.drawShapes()

        pygame.display.update()


if __name__ == '__main__':
    print("\n--------------------------------------------------------------------------------------\n"
          "Hello, you are executing the wrong file, it will still work but please read the README!\n"
          "The correct file will execute now.\n"
          "--------------------------------------------------------------------------------------\n")
    # Get the absolute path to fileA.py
    # Determine the directory where fileA.py is located
    dir_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fileA_path = os.path.join(dir_path, 'runner.py')

    os.chdir(dir_path)

    # Run fileA.py using Python
    subprocess.run(['python', fileA_path])
