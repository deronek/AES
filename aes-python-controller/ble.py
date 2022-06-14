import asyncio
import logging
from abc import ABC, abstractmethod
from asyncio import Lock
from enum import Enum

import bleak
from bleak import BleakClient


class SensorID(Enum):
    TASK_ID_HC_SR04 = 0
    TASK_ID_MPU9255 = 1
    TASK_ID_ALGO = 2


NUMBER_OF_HC_SR04_SENSORS = 8


class LockedData(ABC):
    lock: Lock
    data: list

    def __init__(self):
        self.lock = Lock()

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

    async def get_data(self):
        # async with self.lock:
        distance = HcSr04.Distance(self.data)
        return distance


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
    logger: logging.Logger
    # TODO
    client: BleakClient
    tx_queue: asyncio.Queue

    def __init__(self):
        self.hc_sr04 = HcSr04()
        self.heading = 0

        logger = logging.getLogger(self.__class__.__name__)
        logging_handler = logging.FileHandler('ble.log')
        logger.addHandler(logging_handler)
        self.logger = logger

        self.client = BleakClient(ADDRESS)
        self.tx_queue = asyncio.Queue()

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
        try:
            # print('packet')
            packet = BLE.BlePacket(data)
            await self.save_packet(packet)
        except KeyError:
            print(f'Unrecognized packet ID, got {data[0]}')

    async def save_packet(self, packet: BlePacket):
        match packet.id:
            case SensorID.TASK_ID_HC_SR04:
                await self.hc_sr04.update_data(packet.data)

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
            # powrót gdy stracimy połaczneie
                await self.client.connect()
                await self.client.start_notify(ESP_GATT_UUID_DATA_NOTIFICATION, self.callback)

                while True:
                    await self.client.write_gatt_char(ESP_GATT_UUID_HEARTBEAT, HEARTBEAT_STRING)
                    await asyncio.sleep(1)

            except bleak.exc.BleakError as e:
                print(f"Exception in BLE.main(): {str(e)}")
                await asyncio.sleep(1)



            # send heartbeat every 1 second
            #     time.sleep(1)
            # await asyncio.Event().wait()
            # while True:
            #     await client.start_notify(ESP_GATT_UUID_SPP_DATA_NOTIFY, callback)


    async def ble_tx(self):
        while True:
            if self.tx_queue.qsize() > 10:
                print("The queue is full")

            data = await self.tx_queue.get()
            # await self.client.write_gatt_char(ESP_GATT_UUID_CTRL_INDICATION, None)
            packet = [0x10, data.value]
            await self.client.write_gatt_char(ESP_GATT_UUID_CTRL_INDICATION, bytes(packet), response=True)


    async def send_start_drive(self):
        if self.client.is_connected:
            await self.tx_queue.put(BleControllerRequestId.REQUEST_START_DRIVING)



    async def send_stop_drive(self):
        if self.client.is_connected:
            await self.tx_queue.put(BleControllerRequestId.REQUEST_STOP_DRIVING)

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
