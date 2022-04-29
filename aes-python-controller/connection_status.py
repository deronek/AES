import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR

SIZE = (800, 100)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 80)


class ConnectionStatus(Enum):
    CONNECTED = auto()
    NOT_CONNECTED = auto()


CONNECTION_STATUS = ConnectionStatus.CONNECTED


def draw_connection_status():
    con_status = pygame.Surface(SIZE)
    con_status_rect = pygame.Rect(0, 0, *SIZE)

    if CONNECTION_STATUS == ConnectionStatus.CONNECTED:
        status = FONT.render("CONNECTED", False, GREEN)
    else:
        status = FONT.render("NOT CONNECTED", False, RED)
    con_status.blit(status, (0, 0))

    return con_status
