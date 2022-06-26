import math
from random import random, randint

import pygame
import pygame.gfxdraw
from pygame import Surface

from constants import RAD0, RAD180, WHITE, RED, BLACK

LINE_THICKNESS = 2
SIZE = 2000, 2000

RADAR_RADIUS = 500

START_POINT_LINES = pygame.Vector2(RADAR_RADIUS, RADAR_RADIUS)
END_POINT_VECTOR = pygame.Vector2(0, -RADAR_RADIUS)

ANGLES = [-90, -67.5, -45, -22.5, 0, 22.5, 45, 67.5, 90]
ANGLES_MARKERS = [22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180]
ALPHA_TICK_DECREASE = 10


# distance_HcSr04 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1 w środku okręgu

# do radarmarker potrzebujemy od 0 do 180 za to


class RadarMarker:
    def __init__(self, surface, rect, start_angle, stop_angle):
        self.alpha = 255
        self.surface = surface
        self.rect = rect
        self.start_angle = start_angle
        self.stop_angle = stop_angle

    def draw(self):
        # print(self.alpha)
        color = pygame.Color(self.alpha, 0, 0)
        # color = (self.alpha, 0, 0)
        pygame.draw.arc(self.surface, color,
                        self.rect,
                        self.start_angle, self.stop_angle, 10)
        # pygame.gfxdraw.arc(self.surface, 200, 200,
        #                    50, 90, 120, color)
        # pygame.draw.circle(self.surface, color,
        #                    (self.rect[0], self.rect[1]),
        #                    radius=50)

        # OR ####
        # pygame.draw.arc(self.surface, RED,
        #                 self.rect,
        #                 self.start_angle, self.stop_angle, 10)
        # self.surface.set_alpha(self.alpha)

    def draw_black(self):
        pygame.draw.arc(self.surface, BLACK,
                        self.rect,
                        self.start_angle, self.stop_angle, 10)

    def tick_alpha(self):
        self.alpha -= ALPHA_TICK_DECREASE


class RadarMarkers:
    def __init__(self):
        self.markers_list = []

    def add(self, surface, angle, distance: float):
        # if len(self.markers_list) > 0:
        #     return
        angle_radians = math.radians(angle)
        radius = RADAR_RADIUS * distance
        marker = RadarMarker(surface, (-math.cos(angle_radians) * radius, math.sin(angle_radians) * radius, 1000, 1000),
                             math.radians(angle - (22.5 * (1 - distance))), math.radians(angle))
        self.markers_list.append(marker)

    def draw(self):
        for marker in self.markers_list:
            marker.draw()
            marker.tick_alpha()
            # if marker.alpha <= 0:
            #     marker.draw_black()
            #     self.markers_list.remove(marker)
        self.markers_list = [marker for marker in self.markers_list if marker.alpha > 0]


RADAR_MARKERS = RadarMarkers()


def draw_radar(surface: pygame.Surface, distance):
    # radar = pygame.Surface((1000, 505))
    # radar.fill(BLACK)
    pygame.draw.arc(surface, WHITE, (0, 0, 1000, 1000), RAD0, RAD180, LINE_THICKNESS)
    pygame.draw.line(surface, WHITE, (0, RADAR_RADIUS), (500, RADAR_RADIUS), LINE_THICKNESS)
    # radar = pygame.Surface((1000, 505), pygame.SRCALPHA)
    # # radar.fill(BLACK)
    # radar_rect = pygame.Rect(0, 0, *SIZE)
    # pygame.draw.arc(radar, WHITE, (0, 0, 1000, 1000), RAD0, RAD180, LINE_THICKNESS)
    # pygame.draw.line(radar, WHITE, (0, RADAR_RADIUS), (500, RADAR_RADIUS), LINE_THICKNESS)
    # pygame.draw.arc(radar, RED, (-250, 0, 1000, 1000), math.radians(0), math.radians(22.5), LINE_THICKNESS)

    # put new markers if distance data is available
    if distance:
        distance.reverse()
        for angle in range(len(ANGLES_MARKERS)):
            d = distance[angle]
            if d == 0:
                continue
            d /= 2000000
            d = max(min(d, 1), 0)
            d = 1 - d
            RADAR_MARKERS.add(radar, ANGLES_MARKERS[angle], d)
            # r = randint(0, 30)
            # if r == 5:
            #     RADAR_MARKERS.add(surface, ANGLES_MARKERS[angle], d)

    RADAR_MARKERS.draw()
    # for marker in RADAR_MARKERS.markers_list:
    #     radar.blit(marker.surface, (0, 0))

    for angle in ANGLES:
        end_point = START_POINT_LINES + END_POINT_VECTOR.rotate(angle)
        pygame.draw.line(surface, WHITE, START_POINT_LINES, end_point, LINE_THICKNESS)
    pygame.draw.line(surface, WHITE, START_POINT_LINES, (600, 600))
    # pygame.draw.arc(WIN, WHITE, [300, 300, 100, 100], RAD_NEG90, RAD90, 5)
    # surface.blit(radar, (0, 0))
    # surface.fill(BLACK)
    # surface.blit(radar, (0, 0), special_flags=pygame.BLEND_PREMULTIPLIED)
    # print(len(RADAR_MARKERS.markers_list))
