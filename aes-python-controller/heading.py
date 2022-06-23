import os
from enum import auto, Enum

import pygame

from constants import GREEN, RED, ASSETS_DIR, GRAY, WHITE, BLACK


SIZE = (250, 250)
FONT = pygame.font.Font(os.path.join(ASSETS_DIR, 'Iceland-Regular.ttf'), 100)


def draw_heading(angle):
    img = pygame.image.load('spidercar.png')
    # img.set_colorkey((0, 0, 0))
    heading = pygame.Surface(SIZE)
    heading.fill(BLACK)
    # heading_rect = pygame.Rect(0, 0, *SIZE)
    #vehicle = pygame.draw.rect(heading, GRAY, (0, 0, 164, 194), 0, 20)
    car = pygame.transform.rotate(img, angle)
    heading.blit(car, (heading.get_width()/2 - int(car.get_width()/2), heading.get_width()/2 - int(car.get_width()/2)))


    #rotated = heading.get_rect(center=(-50, 50))
    # angle = FONT.render("ANGLE: 0\N{DEGREE SIGN}", False, WHITE)
    # heading.blit(angle, (200, 45))

    return heading
