import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

SIZE = (700, 300)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)


def draw_heading():
    heading = pygame.Surface(SIZE)
    # heading_rect = pygame.Rect(0, 0, *SIZE)
    vehicle = pygame.draw.rect(heading, GRAY, (0, 0, 163, 193), 0, 20)
    angle = FONT.render("ANGLE: 0\N{DEGREE SIGN}", False, WHITE)
    heading.blit(angle, (200, 45))

    return heading
