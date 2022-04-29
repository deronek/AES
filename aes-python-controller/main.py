import pygame

pygame.font.init()
pygame.mixer.init()
import pygame_widgets

from buttons import draw_buttons, init_buttons
from constants import BLACK

from app_manager import draw_app_manager

from timestamp import draw_timestamp

import math

from heading import draw_heading

import os

from connection_status import draw_connection_status
from radar import draw_radar

WIDTH, HEIGHT = 1920, 1080
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("AES Controller")

BORDER = pygame.Rect(WIDTH // 2 - 5, 0, 10, HEIGHT)

# BULLET_HIT_SOUND = pygame.mixer.Sound('Assets/Grenade+1.mp3')
# BULLET_FIRE_SOUND = pygame.mixer.Sound('Assets/Gun+Silencer.mp3')


FPS = 30


def draw_window(events):
    # WIN.fill(BLACK)
    WIN.blit(draw_radar(), (37, 137))
    WIN.blit(draw_connection_status(), (35, 20))
    WIN.blit(draw_heading(), (200, 800))
    WIN.blit(draw_timestamp(), (1170, 50))
    WIN.blit(draw_app_manager(), (1300, 200))
    WIN.blit(draw_buttons(), (1500, 750))

    # pygame_widgets.update(events)
    pygame.display.update()


def main():
    clock = pygame.time.Clock()
    run = True
    init_buttons()
    while run:
        clock.tick(FPS)
        events = pygame.event.get()
        for event in events:
            match event.type:
                case pygame.QUIT:
                    run = False
                    pygame.quit()

        draw_window(events)

    quit()


if __name__ == "__main__":
    main()
