import asyncio
import logging
import statistics
import struct
import time
from abc import ABC, abstractmethod
from asyncio import Lock
from enum import Enum

import bleak
from bleak import BleakClient, BleakError

from app_manager import AppManagerState
from behaviour import BehaviourState


class SensorID(Enum):
    TASK_ID_HC_SR04 = 0
    TASK_ID_MPU9255 = 1
    TASK_ID_ALGO = 2
    TASK_ID_BLE = 3
    TASK_ID_APP_MANAGER = 4
    TASK_ID_REFLECTANCE = 5


NUMBER_OF_HC_SR04_SENSORS = 8


class LockedData(ABC):
    lock: Lock
    data: list
    available: bool

    def __init__(self):
        self.lock = Lock()
        self.available = False

    @abstractmethod
    async def update_data(self, data: list):
        pass

    @abstractmethod
    async def get_data(self):
        pass

class HcSr04(LockedData):
    class Distance:
        distance: list

        def __init__(self, data: list):
            self.distance = []
            for i in range(NUMBER_OF_HC_SR04_SENSORS):
                self.distance.append(int.from_bytes(
                    data[i * 4: i * 4 + 4],
                    byteorder='little'))

    data: list

    def __init__(self):
        super().__init__()
        self.data = []

    async def update_data(self, data: list):
        # async with self.lock:
        self.data = data
        self.available = True

    async def get_data(self):
        # async with self.lock:
        distance = HcSr04.Distance(self.data)
        return distance


class Algo(LockedData):
    class Data:
        current_heading: float
        pos_x: float
        pos_y: float
        behaviour: BehaviourState
        goal_heading: float
        follow_wall_heading: float
        avoid_obstacle_angle: float

        def __init__(self, data: list):
            # TODO: data should be bytes in LockedData, not list
            # print(data)
            self.current_heading = struct.unpack('f', bytes(data[0:4]))[0]
            self.pos_x = struct.unpack('f', bytes(data[4:8]))[0]
            self.pos_y = struct.unpack('f', bytes(data[8:12]))[0]
            self.behaviour = BehaviourState(int.from_bytes(bytes(data[12:16]), byteorder='little'))
            self.goal_heading = struct.unpack('f', bytes(data[16:20]))[0]
            self.follow_wall_heading = struct.unpack('f', bytes(data[20:24]))[0]
            self.avoid_obstacle_angle = struct.unpack('f', bytes(data[24:28]))[0]

    def __init__(self):
        super().__init__()
        self.data = []

    async def update_data(self, data: list):
        # async with self.lock:
        self.data = data
        self.available = True

    async def get_data(self):
        # async with self.lock:
        data = Algo.Data(self.data)
        return data


class AppManager(LockedData):
    def __init__(self):
        super().__init__()
        self.state = AppManagerState.APP_MANAGER_INIT

    async def update_data(self, data: list):
        # async with self.lock:
        self.state = AppManagerState(data[0])
        self.available = True

    async def get_data(self):
        # async with self.lock:
        state = self.state
        return state


class Reflectance(LockedData):
    class RequestAvoidance:
        left: bool
        right: bool

        def __init__(self, data: list):
            self.left = bool(data[0])
            self.right = bool(data[1])

    def __init__(self):
        super().__init__()
        self.data = []

    async def update_data(self, data: list):
        # async with self.lock:
        self.available = True
        self.data = data

    async def get_data(self):
        # async with self.lock:
        request_avoidance = Reflectance.RequestAvoidance(self.data)
        return request_avoidance

# class AlgoData(LockedData):
#     def __init__(self, data: list):
#         self.data = data
#
#         self.heading = int.from_bytes(self.data, byteorder='little', signed=True) / 10


ADDRESS = "E8:31:CD:C4:76:62"
# ADDRESS = "3C:61:05:30:8B:4A"
ESP_GATT_UUID_CTRL_INDICATION = "0000abf1-0000-1000-8000-00805f9b34fb"
ESP_GATT_UUID_DATA_NOTIFICATION = "0000abf2-0000-1000-8000-00805f9b34fb"
ESP_GATT_UUID_HEARTBEAT = "0000abf5-0000-1000-8000-00805f9b34fb"

HEARTBEAT_STRING = 'AES-2022'.encode()

################################ read MAC adress ##############################################
# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         print(d)
# asyncio.run(main())

last = 0


class BleControllerRequestId(Enum):
    REQUEST_START_DRIVING = 0
    REQUEST_STOP_DRIVING = 1
    pass


class BLE:
    hc_sr04: HcSr04
    algo: Algo
    app_manager: AppManager
    reflectance: Reflectance
    logger: logging.Logger
    # TODO
    client: BleakClient
    tx_queue: asyncio.Queue
    timestamp: int

    def __init__(self, ui_reset_data_handle):
        self.hc_sr04 = HcSr04()
        self.algo = Algo()
        self.app_manager = AppManager()
        self.reflectance = Reflectance()
        self.timestamp = 0

        self.ui_reset_data_handle = ui_reset_data_handle

        logger = logging.getLogger(self.__class__.__name__)
        logging_handler = logging.FileHandler('ble.log')
        logger.addHandler(logging_handler)
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s %(message)s')
        logging_handler.setFormatter(formatter)
        self.logger = logger
        self.logger.info('aes-python-controller started')

        self.client = BleakClient(ADDRESS)
        self.tx_queue = asyncio.Queue()

        self.last_time = None
        self.times = []

    def reset_data(self):
        self.algo.available = False
        self.app_manager.available = False
        self.hc_sr04.available = False
        self.timestamp = 0
        self.ui_reset_data_handle()

    class BlePacket:
        id: SensorID
        timestamp: int
        data_len: int
        data: list
        checksum: int

        def __init__(self, data: bytearray):
            # print([hex(i) for i in data])
            self.id = SensorID(data[0])
            self.timestamp = int.from_bytes(data[1:5], byteorder='little')
            self.data_len = data[5]
            self.checksum = data[-1]
            self.data = list(data[6:-1])
            if len(self.data) != self.data_len:
                raise ValueError(f'Incorrect data length, {len(self.data)=}, {self.data_len=}')

    async def callback(self, sender: int, data: bytearray):
        # if self.last_time is None:
        #     self.last_time = time.time()
        #     return
        # current_time = time.time()
        # self.times.append(current_time - self.last_time)
        # self.last_time = current_time
        # print(f'mean: {statistics.mean(self.times)}, '
        #       f'stdev: {statistics.stdev(self.times)}, '
        #       f'median: {statistics.median(self.times)}')
        try:
            # print('packet')
            packet = BLE.BlePacket(data)
            self.logger.info(str([hex(i) for i in list(data)]))
            await self.save_packet(packet)
            self.timestamp = packet.timestamp
        except KeyError:
            print(f'Unrecognized packet ID, got {data[0]}')

    async def save_packet(self, packet: BlePacket):
        match packet.id:
            case SensorID.TASK_ID_HC_SR04:
                await self.hc_sr04.update_data(packet.data)
            case SensorID.TASK_ID_ALGO:
                await self.algo.update_data(packet.data)
            case SensorID.TASK_ID_APP_MANAGER:
                await self.app_manager.update_data(packet.data)
            case SensorID.TASK_ID_REFLECTANCE:
                await self.reflectance.update_data(packet.data)
            case _:
                raise KeyError(f'SensorID {packet.id} not implemented')

    async def main(self, address=ADDRESS):
        # TODO: refactor client as class member
        """
        - handle connection
        - handle reconnecting
        - when connected:
            - start notify
            - send heartbeat (in loop)
        """
        while True:
            try:
                # create new instance of the client, otherwise sometimes
                # it will not connect again
                self.client = BleakClient(ADDRESS)
                # powr??t gdy stracimy po??aczneie
                await self.client.connect()
                await self.client.start_notify(ESP_GATT_UUID_DATA_NOTIFICATION, self.callback)

                while True:
                    await self.client.write_gatt_char(ESP_GATT_UUID_HEARTBEAT, HEARTBEAT_STRING)
                    await asyncio.sleep(1)

            except Exception as e:
                print(f"Exception in BLE.main(): {str(e)}")
                # try again
                await self.client.disconnect()
                self.reset_data()
                await asyncio.sleep(1)

    async def ble_tx(self):
        while True:
            try:
                if self.tx_queue.qsize() > 10:
                    print("The queue is full")

                data = await self.tx_queue.get()
                packet = [0x10, data.value]
                print(f'Sending BLE TX {packet}')
                await self.client.write_gatt_char(ESP_GATT_UUID_CTRL_INDICATION, bytes(packet), response=True)
            except Exception as e:
                print(f"Exception in BLE.ble_tx(): {str(e)}")

    def send_start_drive(self):
        if self.client.is_connected:
            self.tx_queue.put_nowait(BleControllerRequestId.REQUEST_START_DRIVING)

    def send_stop_drive(self):
        if self.client.is_connected:
            self.tx_queue.put_nowait(BleControllerRequestId.REQUEST_STOP_DRIVING)

# ble = BLE()
# # asyncio.run(ble.main(address))
#
# #
# # async def main(address):
# #     async with BleakClient(address) as client:
# #         while True:
# #             model_number = await client.read_gatt_char(ESP_GATT_UUID_SPP_DATA_RECEIVE)
# #             print(model_number)
# # asyncio.run(main(address))

# ############################################## read UUID adress ##################################
# async def main(address: str):
#      async with BleakClient(address) as client:
#          svcs = await client.get_services()
#          print("Services:")
#          for service in svcs:
#              print(service)
# asyncio.run(main(address))

# if __name__ == "__main__":
#     main(address)
