import math
from random import random, randint

import pygame
from pygame import Surface

from constants import RAD0, RAD180, WHITE, RED, BLACK

LINE_THICKNESS = 2
SIZE = 2000, 2000

RADAR_RADIUS = 500

START_POINT_LINES = pygame.Vector2(RADAR_RADIUS, RADAR_RADIUS)
END_POINT_VECTOR = pygame.Vector2(0, -RADAR_RADIUS)

ANGLES = [-90, -67.5, -45, -22.5, 0, 22.5, 45, 67.5, 90]
ANGLES_MARKERS = [22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180]
ALPHA_TICK_DECREASE = 20


# distance_HcSr04 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1 w środku okręgu

# do radarmarker potrzebujemy od 0 do 180 za to


class RadarMarker:
    def __init__(self, rect, start_angle, stop_angle):
        self.alpha = 255
        self.rect = rect
        self.start_angle = start_angle
        self.stop_angle = stop_angle

    def draw(self):
        self.surface = pygame.Surface((1000, 500), pygame.SRCALPHA)

        color = (self.alpha, 0, 0)
        pygame.draw.arc(self.surface, color,
                        self.rect,
                        self.start_angle, self.stop_angle, 10)

        # OR ####
        # pygame.draw.arc(self.surface, RED,
        #                 self.rect,
        #                 self.start_angle, self.stop_angle, 10)
        # self.surface.set_alpha(self.alpha)

    def draw_black(self):
        self.surface = pygame.Surface((1000, 500))
        pygame.draw.arc(self.surface, BLACK,
                        self.rect,
                        self.start_angle, self.stop_angle, 10)

    def tick_alpha(self):
        self.alpha -= ALPHA_TICK_DECREASE


class RadarMarkers:
    def __init__(self):
        self.markers_list = []

    def add(self, angle, distance: float):
        angle_radians = math.radians(angle)
        radius = RADAR_RADIUS * distance
        marker = RadarMarker((-math.cos(angle_radians) * radius, math.sin(angle_radians) * radius, 1000, 1000),
                             math.radians(angle - (22.5 * (1 - distance))), math.radians(angle))
        self.markers_list.append(marker)

    def draw(self):
        for marker in self.markers_list[:]:
            marker.draw()
            marker.tick_alpha()
            if marker.alpha <= 0:
                marker.draw_black()
                self.markers_list.remove(marker)


RADAR_MARKERS = RadarMarkers()


def draw_radar(distance) -> Surface:
    radar = pygame.Surface((1000, 505), pygame.SRCALPHA)
    # radar.fill(BLACK)
    radar_rect = pygame.Rect(0, 0, *SIZE)
    pygame.draw.arc(radar, WHITE, (0, 0, 1000, 1000), RAD0, RAD180, LINE_THICKNESS)
    pygame.draw.line(radar, WHITE, (0, RADAR_RADIUS), (500, RADAR_RADIUS), LINE_THICKNESS)
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
            # print(d)
            d = max(min(d, 1), 0)
            d = 1 - d
            # print(d)
            RADAR_MARKERS.add(ANGLES_MARKERS[angle], d)
            # r = randint(0, 10)
            # if r == 5:
            # RADAR_MARKERS.add(ANGLES_MARKERS[angle], distance[angle])

    RADAR_MARKERS.draw()
    for marker in RADAR_MARKERS.markers_list:
        radar.blit(marker.surface, (0, 0))

    for angle in ANGLES:
        end_point = START_POINT_LINES + END_POINT_VECTOR.rotate(angle)
        pygame.draw.line(radar, WHITE, START_POINT_LINES, end_point, LINE_THICKNESS)
    # pygame.draw.line(radar, WHITE, START_POINT_LINES, (600, 600))
    # pygame.draw.arc(WIN, WHITE, [300, 300, 100, 100], RAD_NEG90, RAD90, 5)

    return radar
