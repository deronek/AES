import os
from enum import auto, Enum

import pygame
import pygame.freetype

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, BLACK

SIZE = (250, 250)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Spidery.ttf'), 32)
TITLE = 'SPIDERCAR CONTROLLER'


def draw_title(surface: pygame.Surface):
    img = pygame.image.load(os.path.join(ASSETS_DIR, 'spidercar controller white.png'))
    # text = FONT.render(TITLE, False, RED)
    surface.blit(img, (0, 0))
