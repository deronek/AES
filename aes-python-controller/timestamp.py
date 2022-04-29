import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

SIZE = (700, 200)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)


def draw_timestamp():
    timestamp = pygame.Surface(SIZE)
    # timestamp_rect = pygame.Rect(0, 0, *SIZE)
    text = FONT.render("TIMESTAMP: 0 ms", False, WHITE)
    timestamp.blit(text, (0, 0))

    return timestamp
