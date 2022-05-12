import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, BLACK

# SIZE = (700, 300)
SIZE = (163, 193)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)


def draw_heading(angle):
    heading = pygame.Surface(SIZE)
    heading.fill(BLACK)
    # heading_rect = pygame.Rect(0, 0, *SIZE)
    vehicle = pygame.draw.rect(heading, GRAY, (0, 0, 163, 193), 0, 20)
    heading = pygame.transform.rotate(heading, angle)
    # angle = FONT.render("ANGLE: 0\N{DEGREE SIGN}", False, WHITE)
    # heading.blit(angle, (200, 45))

    return heading
