import pygame
pygame.font.init()
pygame.mixer.init()
from threading import Thread
import time
import asyncio
import os
from buttons import draw_buttons, init_buttons
from constants import BLACK
from app_manager import draw_app_manager
from timestamp import draw_timestamp
import math
from heading import draw_heading
from connection_status import draw_connection_status
from radar import draw_radar

import ctypes
import pygame_widgets
from ble import BLE



# fix scaling on Windows
if os.name == 'nt':
    ctypes.windll.user32.SetProcessDPIAware()


WIDTH, HEIGHT = 1920, 1080
# WIN = pygame.display.set_mode((WIDTH, HEIGHT))
WIN = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("AES Controller")

BORDER = pygame.Rect(WIDTH // 2 - 5, 0, 10, HEIGHT)

# BULLET_HIT_SOUND = pygame.mixer.Sound('Assets/Grenade+1.mp3')
# BULLET_FIRE_SOUND = pygame.mixer.Sound('Assets/Gun+Silencer.mp3')


FPS = 30

# os.environ['PYTHONASYNCIODEBUG'] = '1'


class AESController:
    async def draw_window(self):
        # WIN.fill(BLACK)
        #distance = [10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000]
        distance = await self.ble.hc_sr04.get_data()
        # distance = []
        WIN.blit(draw_radar(distance.distance), (37, 137))
        WIN.blit(draw_connection_status(), (35, 20))
        # WIN.blit(draw_heading(angle=self.ble.heading), (200, 800))
        WIN.blit(draw_timestamp(), (1170, 50))
        WIN.blit(draw_app_manager(), (1300, 200))
        WIN.blit(draw_buttons(), (1500, 750))

        # pygame_widgets.update(events)
        pygame.display.update()

    def pygame_task(self):
        pass

    async def pygame_loop(self):
        current_time = 0
        while True:
            await asyncio.sleep(0)
            last_time, current_time = current_time, time.time()
            await asyncio.sleep(1 / FPS - (current_time - last_time))  # tick
            await self.draw_window()

    async def handle_events(self, event_queue):
        while True:
            await asyncio.sleep(0)
            event = await event_queue.get()
            if event.type == pygame.QUIT:
                break
            else:
                pass
                # print("event", event)
        asyncio.get_event_loop().stop()

    async def pygame_event_loop(self, event_queue):
        while True:
            await asyncio.sleep(0)
            event = pygame.event.poll()
            if event.type != pygame.NOEVENT:
                await event_queue.put(event)

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

        pygame.init()
        init_buttons()

        self.ble = BLE()
        # ble_thread = Thread(target=asyncio.run, args=(self.ble.main("3C:61:05:30:8B:4A"),))
        # ble_thread.start()

        # pygame_event_task = asyncio.ensure_future(
            # self.pygame_event_loop(event_queue), loop=loop)
        pygame_main_task = asyncio.ensure_future(self.pygame_loop(), loop=loop)
        # event_task = asyncio.ensure_future(
            # self.handle_events(event_queue), loop=loop)
        ble_task = asyncio.ensure_future(self.ble.main(), loop=loop)
        

        # pygame_event_task.add_done_callback(self.exception_handler)
        pygame_main_task.add_done_callback(self.exception_handler)
        # event_task.add_done_callback(self.exception_handler)
        ble_task.add_done_callback(self.exception_handler)
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            ble_task.cancel()
            pygame_main_task.cancel()
            # pygame_event_task.cancel()
            # event_task.cancel()

        pygame.quit()


if __name__ == "__main__":
    aes_controller = AESController()
    aes_controller.main()
