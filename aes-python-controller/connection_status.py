import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, BLACK

SIZE = (800, 100)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 80)


# class ConnectionStatus(Enum):
#     CONNECTED = auto()
#     NOT_CONNECTED = auto()
#
#
# CONNECTION_STATUS = ConnectionStatus.CONNECTED


def draw_connection_status(surface: pygame.Surface, connected: bool):
    surface.fill(BLACK)
    if connected:
        status = FONT.render("CONNECTED", False, GREEN)
    else:
        status = FONT.render("NOT CONNECTED", False, RED)
    surface.blit(status, (0, 0))
