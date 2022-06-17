import os
from enum import auto, Enum

import pygame
import pygame_widgets
from pygame_widgets.button import Button

import buttons
from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE

# SIZE = (700, 400)
SIZE = (1920, 1080)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)

BUTTONS = pygame.Surface(SIZE)
START_BUTTON: Button
STOP_BUTTON: Button

class Buttons:
    def __init__(self, surface, start_callback, stop_callback):
        self.surface = surface
        self.start_button = Button(
            surface,
            0, 0,
            350,
            120,

            text='START',
            textColour=WHITE,
            font=FONT,
            inactiveColour=GREEN,
            hoverColour=(0, 180, 0),
            pressedColour=(0, 100, 0),
            onClick=start_callback
        )

        self.stop_button = Button(
            surface,
            0, 150,
            350,
            120,

            text='STOP',
            textColour=WHITE,
            font=FONT,
            inactiveColour=RED,
            hoverColour=(180, 0, 0),
            pressedColour=(100, 0, 0),
            onClick=stop_callback
        )

    def draw(self):
        self.start_button.draw()
        self.stop_button.draw()
