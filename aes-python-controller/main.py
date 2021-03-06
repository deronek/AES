import random
from typing import Optional

import pygame

pygame.font.init()
pygame.mixer.init()

from threading import Thread
import time
import asyncio
import os
from buttons import Buttons
from constants import BLACK
from app_manager import draw_app_manager, AppManagerState
from timestamp import draw_timestamp
import math
from heading import draw_heading
from connection_status import draw_connection_status
from title import draw_title
from radar import draw_radar
from behaviour import draw_behaviour, BehaviourState

import ctypes
import pygame_widgets
from ble import BLE

import numpy as np

# angle1 = 0

# fix scaling on Windows
if os.name == 'nt':
    ctypes.windll.user32.SetProcessDPIAware()

WIDTH, HEIGHT = 1920, 1080
# WIN = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN | pygame.DOUBLEBUF, 8)

BORDER = pygame.Rect(WIDTH // 2 - 5, 0, 10, HEIGHT)

# BULLET_HIT_SOUND = pygame.mixer.Sound('Assets/Grenade+1.mp3')
# BULLET_FIRE_SOUND = pygame.mixer.Sound('Assets/Gun+Silencer.mp3')
TITLE_RECT = pygame.Rect(40, 40, 1200, 100)
APP_MANAGER_RECT = pygame.Rect(1330, 40, 550, 400)
BEHAVIOUR_RECT = pygame.Rect(1330, 450, 550, 300)
BUTTONS_RECT = pygame.Rect(1530, 780, 390, 280)
CONNECTION_STATUS_RECT = pygame.Rect(28, 985, 757, 80)
TIMESTAMP_RECT = pygame.Rect(700, 985, 1100, 80)
RADAR_RECT = pygame.Rect(40, 160, 1000, 505)

FPS = 30


# os.environ['PYTHONASYNCIODEBUG'] = '1'


class AESController:
    buttons: Buttons
    window: pygame.Surface

    current_heading: float
    current_heading: float
    pos_x: float
    pos_y: float
    app_manager_state: Optional[AppManagerState]
    behaviour: Optional[BehaviourState]
    goal_heading: Optional[float]
    follow_wall_heading: Optional[float]
    avoid_obstacle_angle: Optional[float]
    reflectance_left: bool
    reflectance_right: bool


    def __init__(self):
        self.reset_data()

        self.ble = BLE(self.reset_data)

        pygame.init()
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=0)
        print(self.window.get_size())

        self.buttons = Buttons(self.window.subsurface(BUTTONS_RECT),
                               self.ble.send_start_drive,
                               self.ble.send_stop_drive)
        pygame.display.set_caption("AES Controller")

        self.app_manager_surface = self.window.subsurface(APP_MANAGER_RECT)
        self.behaviour_surface = self.window.subsurface(BEHAVIOUR_RECT)
        self.radar_surface = self.window.subsurface(RADAR_RECT)
        # self.radar_surface.convert_alpha()

    def reset_data(self):
        self.current_heading = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.behaviour = None
        self.app_manager_state = None
        self.goal_heading = None
        self.follow_wall_heading = None
        self.avoid_obstacle_angle = None
        self.reflectance_left = False
        self.reflectance_right = False

    async def draw_window(self):
        self.handle_events()
        # WIN.fill(BLACK)

        if self.ble.algo.available:
            algo = await self.ble.algo.get_data()
            self.current_heading = algo.current_heading
            self.behaviour = algo.behaviour
            self.goal_heading = algo.goal_heading
            self.follow_wall_heading = algo.follow_wall_heading
            self.avoid_obstacle_angle = algo.avoid_obstacle_angle
            self.pos_x = algo.pos_x
            self.pos_y = algo.pos_y
            # print(self.pos_x, self.pos_y)
        else:
            self.goal_heading = None
            self.follow_wall_heading = None
            self.avoid_obstacle_angle = None
        if self.ble.hc_sr04.available:
            hc_sr04 = await self.ble.hc_sr04.get_data()
            distance = hc_sr04.distance
            # distance = [90000, 20000, 30000, 40000, 50000, 60000, 70000, 80000]
        else:
            # distance = random.sample(range(0, 2000000), 8)
            distance = []

        if self.ble.reflectance.available:
            request_avoidance = await self.ble.reflectance.get_data()
            self.reflectance_left = request_avoidance.left
            self.reflectance_right = request_avoidance.right
            # print(self.reflectance_left, self.reflectance_right)

        if self.ble.app_manager.available:
            self.app_manager_state = await self.ble.app_manager.get_data()
        timestamp = self.ble.timestamp

        self.window.blit(draw_heading(current_heading=self.current_heading,
                                      goal_heading=self.goal_heading,
                                      follow_wall_angle=self.follow_wall_heading,
                                      avoid_obstacle_angle=self.avoid_obstacle_angle),
                         (415, 680))
        draw_radar(self.radar_surface, distance)
        draw_connection_status(self.window.subsurface(CONNECTION_STATUS_RECT), self.ble.client.is_connected)
        draw_timestamp(self.window.subsurface(TIMESTAMP_RECT), timestamp)
        draw_app_manager(self.app_manager_surface, self.app_manager_state)
        draw_behaviour(self.behaviour_surface, self.behaviour)
        draw_title(self.window.subsurface(TITLE_RECT))
        self.buttons.draw()

        # pygame_widgets.update(events)
        pygame.display.update()

    def handle_events(self):
        events = pygame.event.get()
        # print(events)
        for event in events:
            if event.type == pygame.QUIT:
                break
        pygame_widgets.update(events)

    async def pygame_loop(self):
        # self.init_window()
        current_time = 0
        while True:
            await asyncio.sleep(0)
            last_time, current_time = current_time, time.time()
            await asyncio.sleep(1 / FPS - (current_time - last_time))  # tick
            await self.draw_window()
            # await asyncio.sleep(5)
            # await self.ble.send_start_drive()

    # async def handle_events(self, event_queue):
    #     while True:
    #         await asyncio.sleep(0)
    #         event = await event_queue.get()
    #         if event.type == pygame.QUIT:
    #             break
    #         else:
    #             pass
    #             # print("event", event)
    #     asyncio.get_event_loop().stop()

    # async def pygame_event_loop(self, event_queue):
    #     while True:
    #         await asyncio.sleep(0)
    #         event = pygame.event.poll()
    #         if event.type != pygame.NOEVENT:
    #             await event_queue.put(event)

    def exception_handler(self, task):
        try:
            task.result()
        except asyncio.CancelledError:
            pass
        except Exception:
            raise

    def main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        # loop.set_exception_handler(self.exception_handler)
        event_queue = asyncio.Queue()
        # ble_thread = Thread(target=asyncio.run, args=(self.ble.main("3C:61:05:30:8B:4A"),))
        # ble_thread.start()

        # pygame_event_task = asyncio.ensure_future(
        #     self.pygame_event_loop(event_queue), loop=loop)
        pygame_main_task = asyncio.ensure_future(self.pygame_loop(), loop=loop)
        ble_tx = asyncio.ensure_future(self.ble.ble_tx(), loop=loop)
        # event_task = asyncio.ensure_future(
        #     self.handle_events(event_queue), loop=loop)
        ble_task = asyncio.ensure_future(self.ble.main(), loop=loop)

        # pygame_event_task.add_done_callback(self.exception_handler)
        pygame_main_task.add_done_callback(self.exception_handler)
        ble_tx.add_done_callback(self.exception_handler)
        # event_task.add_done_callback(self.exception_handler)
        ble_task.add_done_callback(self.exception_handler)
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            ble_task.cancel()
            pygame_main_task.cancel()
            ble_tx.cancel()
            # pygame_event_task.cancel()
            # event_task.cancel()

        pygame.quit()


if __name__ == "__main__":
    aes_controller = AESController()
    aes_controller.main()
