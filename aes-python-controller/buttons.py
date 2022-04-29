import os
from enum import auto, Enum

import pygame
import pygame_widgets
from pygame_widgets.button import Button

import buttons
from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

SIZE = (700, 400)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)

BUTTONS = pygame.Surface(SIZE)
START_BUTTON: Button
STOP_BUTTON: Button


def init_buttons():
    global START_BUTTON, STOP_BUTTON
    buttons = pygame.Surface(SIZE)
    START_BUTTON = Button(
        BUTTONS,
        0, 0,
        350,
        120,

        text='START',
        textColour=WHITE,
        font=FONT,
        inactiveColour=GREEN,
        hoverColour=(0, 180, 0),
        pressedColour=(0, 100, 0),
        onClick=lambda: print('Click')
    )

    STOP_BUTTON = Button(
        BUTTONS,
        0, 150,
        350,
        120,

        text='STOP',
        textColour=WHITE,
        font=FONT,
        inactiveColour=RED,
        hoverColour=(0, 180, 0),
        pressedColour=(0, 100, 0),
        onClick=lambda: print('Click')
    )


def draw_buttons():
    START_BUTTON.draw()
    STOP_BUTTON.draw()
    # START_BUTTON.update()
    return BUTTONS
