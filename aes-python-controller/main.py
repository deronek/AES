import asyncio
from threading import Thread

import pygame

from ble import BLE

pygame.font.init()
pygame.mixer.init()
import pygame_widgets

import ctypes

# fix scaling
ctypes.windll.user32.SetProcessDPIAware()


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

class AESController:
    def draw_window(self, events):
        # WIN.fill(BLACK)
        WIN.blit(draw_radar(), (37, 137))
        WIN.blit(draw_connection_status(), (35, 20))
        WIN.blit(draw_heading(angle=self.ble.heading), (200, 800))
        WIN.blit(draw_timestamp(), (1170, 50))
        WIN.blit(draw_app_manager(), (1300, 200))
        WIN.blit(draw_buttons(), (1500, 750))

        # pygame_widgets.update(events)
        pygame.display.update()

    def pygame_task(self):
        pass


    def main(self):
        # loop = asyncio.get_event_loop()
        self.ble = BLE()
        # loop.run_in_executor(None, self.pygame_task)
        # loop.create_task(self.ble.main("3C:61:05:30:8B:4A"))
        ble_thread = Thread(target=asyncio.run, args=(self.ble.main("3C:61:05:30:8B:4A"),))
        ble_thread.start()
        run = True
        init_buttons()
        clock = pygame.time.Clock()
        clock.tick(FPS)
        while run:
            events = pygame.event.get()
            for event in events:
                match event.type:
                    case pygame.QUIT:
                        run = False
                        pygame.quit()

            self.draw_window(events)
            # loop.call_soon(self.pygame_task)
        # loop.run_forever()
        quit()


if __name__ == "__main__":
    aes_controller = AESController()
    aes_controller.main()
