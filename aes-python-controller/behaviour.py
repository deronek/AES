import os
from enum import auto, Enum
from typing import Optional

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, DARK_GRAY, YELLOW

FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 60)
FONT_LINES = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 40)


class BehaviourState(Enum):
    BEHAVIOUR_DRIVE_TO_GOAL = 0
    BEHAVIOUR_FOLLOW_THE_WALL = 1
    BEHAVIOUR_AVOID_OBSTACLE = 2


BEHAVIOUR_COLORS = {
    BehaviourState.BEHAVIOUR_DRIVE_TO_GOAL: GREEN,
    BehaviourState.BEHAVIOUR_FOLLOW_THE_WALL: YELLOW,
    BehaviourState.BEHAVIOUR_AVOID_OBSTACLE: RED
}


def draw_behaviour(surface: pygame.Surface, state: Optional[BehaviourState]):
    surface.blit(FONT.render("ALGO BEHAVIOUR", False, WHITE), (0, 0))

    for index, item in enumerate(BehaviourState):
        color = BEHAVIOUR_COLORS[item] if item is state else DARK_GRAY
        surface.blit(FONT_LINES.render(item.name, False, color), (0, 80 + 50 * index))
