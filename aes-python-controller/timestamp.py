import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 80)
# FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'NotoSansMono-Regular.ttf'), 55)


def draw_timestamp(surface: pygame.Surface, timestamp: int):
    timestamp_text = FONT.render('TIMESTAMP:', False, WHITE)
    number_text = FONT.render(f"{timestamp:{' '}{'>'}{7}} ms", False, WHITE)
    surface.blit(timestamp_text, (0, 0))
    surface.blit(number_text, (420, 0))
