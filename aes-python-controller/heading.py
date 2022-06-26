import math
import os
from enum import auto, Enum
from random import random
from typing import Optional

import pygame
from pygame.math import Vector2

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, BLACK, YELLOW

heading = 0

SIZE = (250, 250)

ARROW_START = Vector2(125, 125)

ARROW_DISTANCE = 100

IMG = pygame.image.load('spidercar.png')
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)

ARROW_END_CONSTANT = 2.0943951023931953


def arrow_end(distance, angle) -> Vector2:
    v = Vector2()
    v.from_polar((distance, angle))
    return v.cross(ARROW_START)


def arrow(screen, color, angle, trirad=10, thickness=5):
    start = ARROW_START
    end = Vector2()
    end.from_polar((ARROW_DISTANCE, -angle - 90))
    end = ARROW_START + end
    pygame.draw.line(screen, color, start, end, thickness)
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi / 2
    pygame.draw.polygon(screen, color, ((end[0] + trirad * math.sin(rotation),
                                         end[1] + trirad * math.cos(rotation)),
                                        (end[0] + trirad * math.sin(rotation - ARROW_END_CONSTANT),
                                         end[1] + trirad * math.cos(rotation - ARROW_END_CONSTANT)),
                                        (end[0] + trirad * math.sin(rotation + ARROW_END_CONSTANT),
                                         end[1] + trirad * math.cos(rotation + ARROW_END_CONSTANT))))


def draw_heading(current_heading: float,
                 goal_heading: Optional[float],
                 follow_wall_angle: Optional[float],
                 avoid_obstacle_angle: Optional[float]):
    IMG.set_colorkey((0, 0, 0))
    heading = pygame.Surface(SIZE)
    heading.fill(BLACK)
    # heading_rect = pygame.Rect(0, 0, *SIZE)
    # vehicle = pygame.draw.rect(heading, GRAY, (0, 0, 164, 194), 0, 20)
    car = pygame.transform.rotate(IMG, current_heading)
    heading.blit(car, (
        heading.get_width() / 2 - int(car.get_width() / 2), heading.get_width() / 2 - int(car.get_width() / 2)))

    if goal_heading is not None:
        arrow(heading, GREEN, goal_heading)
    if follow_wall_angle is not None and follow_wall_angle != math.inf:
        arrow(heading, YELLOW, follow_wall_angle)
    if avoid_obstacle_angle is not None and avoid_obstacle_angle != math.inf:
        arrow(heading, RED, avoid_obstacle_angle)


    # rotated = heading.get_rect(center=(-50, 50))
    # angle = FONT.render("ANGLE: 0\N{DEGREE SIGN}", False, WHITE)
    # heading.blit(angle, (200, 45))

    return heading
