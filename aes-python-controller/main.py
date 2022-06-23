import random

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
from radar import draw_radar

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
BUTTONS_RECT = pygame.Rect(1500, 750, 420, 330)

FPS = 30


# os.environ['PYTHONASYNCIODEBUG'] = '1'


class AESController:
    buttons: Buttons
    window: pygame.Surface

    heading: float

    def __init__(self):
        self.heading = 0.0

    def init_window(self):
        pygame.init()
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=0)
        print(self.window.get_size())

        self.buttons = Buttons(self.window.subsurface(BUTTONS_RECT),
                               self.ble.send_start_drive,
                               self.ble.send_stop_drive)
        pygame.display.set_caption("AES Controller")
        # pygame.event.set_allowed([pygame.QUIT])
        # init_buttons()

    async def draw_window(self):
        # global angle1
        # angle1 += 1

        self.handle_events()
        # WIN.fill(BLACK)
        timestamp = 0
        app_manager_state = AppManagerState.APP_MANAGER_INIT

        if self.ble.algo.available:
            algo = await self.ble.algo.get_data()
            self.heading = algo.heading
        if self.ble.hc_sr04.available:
            hc_sr04 = await self.ble.hc_sr04.get_data()
            distance = hc_sr04.distance
            # distance = [90000, 20000, 30000, 40000, 50000, 60000, 70000, 80000]
        else:
            distance = []

        app_manager_state = await self.ble.app_manager.get_data()
        timestamp = self.ble.timestamp

        self.window.blit(draw_heading(angle=self.heading), (200, 800))
        self.window.blit(draw_radar(distance), (37, 137))
        self.window.blit(draw_connection_status(self.ble.client.is_connected), (35, 20))
        self.window.blit(draw_timestamp(timestamp), (975, 50))
        self.window.blit(draw_app_manager(app_manager_state), (1300, 200))
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
        self.init_window()
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

        self.ble = BLE()
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
