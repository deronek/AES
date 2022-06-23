import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, DARK_GRAY

SIZE = (600, 400)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 60)
FONT_LINES = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 40)


class AppManagerState(Enum):
    APP_MANAGER_INIT = 0
    APP_MANAGER_READY = 1
    APP_MANAGER_CALIBRATING = 2
    APP_MANAGER_DRIVING = 3
    APP_MANAGER_FINISHED = 4
    APP_MANAGER_STOPPED = 5


# APP_MANAGER = AppManagerState.APP_MANAGER_READY


def draw_app_manager(state: AppManagerState):
    app_manager = pygame.Surface(SIZE)
    app_manager_rect = pygame.Rect(0, 0, *SIZE)

    app_manager.blit(FONT.render("APP MANAGER STATUS", False, WHITE), (0, 0))

    for index, item in enumerate(AppManagerState):
        app_manager.blit(FONT_LINES.render(item.name, False, WHITE if item is state else DARK_GRAY), (0, 80 + 50 * index))

    return app_manager
