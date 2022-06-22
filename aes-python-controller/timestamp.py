import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

SIZE = (1000, 200)
# FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'NotoSansMono-Regular.ttf'), 70)
# FONT = pygame.font.SysFont('monospace', 80)

def draw_timestamp(timestamp: int):
    surface = pygame.Surface(SIZE)
    # timestamp_rect = pygame.Rect(0, 0, *SIZE)
    # text = FONT.render(f"TIMESTAMP: {timestamp:{' '}{'>'}{10}}{'ms':{' '}{'>'}{1}}", False, WHITE)
    text = FONT.render(f"TIMESTAMP: {timestamp:{' '}{'>'}{7}} ms", False, WHITE)
    surface.blit(text, (0, 0))

    return surface
