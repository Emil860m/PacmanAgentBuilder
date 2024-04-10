import math

import pygame
from pygame import Surface

from PacmanAgentBuilder.utils.observation import Observation
from Pacman_Complete.vector import Vector2


class DebugHelper(object):
    """
    The DebugHelper offers static methods that can help with debugging agent behavior.
    """
    _instance = None

    shouldPause = False
    _enabled = True

    _screen = None
    _shapesToDraw = {"line": [], "dashedLine": [], "dashedCircle": [], "dot": []}

    GREEN = (0, 128, 0)
    LIGHTBLUE = (155, 226, 255)
    YELLOW = (255, 218, 0)
    WHITE = (255, 255, 255)
    BLUE = (0, 128, 255)
    PURPLE = (128, 0, 128)
    RED = (255, 30, 30)

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(DebugHelper, cls).__new__(cls)
        return cls._instance

    @staticmethod
    def setScreen(screen: Surface):
        DebugHelper._screen = screen

    @staticmethod
    def pauseGame():
        """
            Pauses the game.
        """
        DebugHelper.shouldPause = True

    @staticmethod
    def disable():
        """
            Disables the DebugHelper.
        """
        DebugHelper._enabled = False

    @staticmethod
    def enable():
        """
            Enables the DebugHelper.
        """
        DebugHelper._enabled = True

    @staticmethod
    def drawLine(startVector: Vector2, endVector: Vector2, color: tuple[int, int, int], width: int = 5):
        """
            Draws a line between two vectors.
            :param startVector: The starting point of the line.
            :param endVector: The ending point of the line.
            :param color: The color of the line.
            :param width: The width of the line.
        """
        if not DebugHelper._enabled:
            return

        DebugHelper.__addDrawObject__("line",
                                      [startVector.asInt(), endVector.asInt(), color, width])

    @staticmethod
    def drawDashedLine(startVector: Vector2, endVector: Vector2, color: tuple[int, int, int], width: int = 5,
                       dashLength: int = 10):
        """
            Draws a dashed line between two vectors.
            :param startVector: The starting point of the dashed line.
            :param endVector: The ending point of the dashed line.
            :param color: The color of the dashed line.
            :param width: The width of the dashed line.
            :param dashLength: The length of the dashes in the line.
        """
        if not DebugHelper._enabled:
            return

        DebugHelper.__addDrawObject__("dashedLine",
                                      [startVector.asInt(), endVector.asInt(), color, width, dashLength])

    @staticmethod
    def drawDot(center: Vector2, radius: float, color: tuple[int, int, int], ):
        """
            Draws a dot at a vector.
            :param center: The center of the dot.
            :param radius: The radius of the dot.
            :param color: The color of the dot.
        """
        if not DebugHelper._enabled:
            return

        DebugHelper.__addDrawObject__("dot",
                                      [center.asInt(), color, radius])

    @staticmethod
    def drawDashedCircle(center: Vector2, radius: float, color: tuple[int, int, int], width=1, dash_length=10):
        """
            Draws a dashed circle around a vector.
            :param center: The center of the dashed circle.
            :param radius: The radius of the dashed circle.
            :param color: The color of the dashed circle.
            :param width: The width of the dashes.
            :param dash_length: The length of the dashes.
        """
        if not DebugHelper._enabled:
            return

        DebugHelper.__addDrawObject__("dashedCircle",
                                      [center.asInt(), radius, color, width, dash_length])

    @staticmethod
    def drawMap(obs: Observation):
        """
            Draws the map/graph of the current level that Pac-Man and the ghosts are moving on.
            :param obs: The current Observation object.
        """
        if not DebugHelper._enabled:
            return

        for node in obs.nodeGroup.nodesLUT.values():
            DebugHelper.drawDot(node.position, 4, DebugHelper.BLUE)

            for n in node.neighbors.keys():
                if node.neighbors[n] is not None:
                    line_start = node.position
                    line_end = node.neighbors[n].position
                    DebugHelper.drawLine(line_start, line_end, DebugHelper.LIGHTBLUE, 2)

    @staticmethod
    def __drawDashedLine__(startVector: Vector2, endVector: Vector2,
                           color: tuple[int, int, int], width=1, dash_length=5):
        x1, y1 = startVector
        x2, y2 = endVector
        dx, dy = x2 - x1, y2 - y1
        distance = math.sqrt(dx ** 2 + dy ** 2)
        dash_count = int(distance / dash_length)

        for i in reversed(range(dash_count)):
            start = x1 + dx * (i / dash_count), y1 + dy * (i / dash_count)
            end = x1 + dx * ((i + 0.5) / dash_count), y1 + dy * ((i + 0.5) / dash_count)
            pygame.draw.line(DebugHelper._screen, color, start, end, width + ((i * 2) % 5 - 2))

    @staticmethod
    def __drawDashedCircle__(center: Vector2, radius: float, color: tuple[int, int, int], width=1, dash_length=5):
        total_circumference = 2 * math.pi * radius
        num_dashes = int(total_circumference / dash_length)
        angle_step = 2 * math.pi / num_dashes

        for dash in range(num_dashes):
            start_angle = dash * angle_step
            end_angle = start_angle + angle_step / 2  # Adjust this for dash thickness

            # Calculate bounding rectangle for the arc
            bounding_rect = (center[0] - radius, center[1] - radius, radius * 2, radius * 2)

            # Draw arc (part of the dashed circle)
            pygame.draw.arc(DebugHelper._screen, color, bounding_rect, start_angle, end_angle, width)

    @staticmethod
    def __addDrawObject__(drawObjectType: str, drawObject: list):
        if not DebugHelper._enabled:
            return
        
        DebugHelper._shapesToDraw[drawObjectType].append(drawObject)

    @staticmethod
    def drawShapes():
        if not DebugHelper._enabled:
            return

        for drawObjectType in DebugHelper._shapesToDraw.keys():
            for drawObject in DebugHelper._shapesToDraw[drawObjectType]:
                if drawObjectType == "line":
                    pygame.draw.line(surface=DebugHelper._screen,
                                     start_pos=drawObject[0], end_pos=drawObject[1],
                                     color=drawObject[2], width=drawObject[3])
                elif drawObjectType == "dashedLine":
                    DebugHelper.__drawDashedLine__(startVector=drawObject[0], endVector=drawObject[1],
                                                   color=drawObject[2], width=drawObject[3], dash_length=drawObject[4])
                elif drawObjectType == "dashedCircle":
                    DebugHelper.__drawDashedCircle__(center=drawObject[0], radius=drawObject[1], color=drawObject[2],
                                                     width=drawObject[3], dash_length=drawObject[4])
                elif drawObjectType == "dot":
                    pygame.draw.circle(surface=DebugHelper._screen, center=drawObject[0],
                                       color=drawObject[1], radius=drawObject[2])

        DebugHelper._shapesToDraw = {"line": [], "dashedLine": [], "dashedCircle": [], "dot": []}
