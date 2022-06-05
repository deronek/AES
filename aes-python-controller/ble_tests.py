import asyncio
import time
from enum import Enum

from bleak import BleakScanner
from bleak import BleakClient


class SensorID(Enum):
    TASK_ID_HC_SR04 = 0
    TASK_ID_MPU9255 = 1
    TASK_ID_ALGO = 2


class SensorData:
    def __init__(self, data: bytearray):
        # print([hex(i) for i in data])
        self.id = SensorID(data[0])
        self.timestamp = int.from_bytes(data[1:5], byteorder='little')
        self.data_len = data[5]
        self.checksum = data[-1]
        self.data = data[6:-1]
        if len(self.data) != self.data_len:
            raise ValueError(f'Incorrect data length, {len(self.data)=}, {self.data_len=}')


NUMBER_OF_HC_SR04_SENSORS = 2


class HC_SR04(SensorData):
    def __init__(self, data: bytearray):
        super().__init__(data)

        self.distance = []
        for i in range(NUMBER_OF_HC_SR04_SENSORS):
            self.distance.append(int.from_bytes(data[i * 4, i * 4 + 4]))


class AlgoData(SensorData):
    def __init__(self, data: bytearray):
        super().__init__(data)

        self.heading = int.from_bytes(self.data, byteorder='little', signed=True) / 10


address = "E8:31:CD:C4:76:62"
# address = "3C:61:05:30:8B:4A"
ESP_GATT_UUID_SPP_DATA_NOTIFY = "0000abf2-0000-1000-8000-00805f9b34fb"
ESP_GATT_UUID_SPP_DATA_RECEIVE = "0000abf1-0000-1000-8000-00805f9b34fb"

############################### read MAC adress ##############################################
# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         print(d)
# asyncio.run(main())

last = 0


class BLE():
    def __init__(self):
        self.heading = 0

    def callback(self, sender: int, data: bytearray):
        print(list(data))
        # algo_data = AlgoData(data)
        # print(algo_data.heading)
        # self.heading = algo_data.heading
        # global last
        # now = time.time()
        # print(last - now)
        # last = now
        # print(f"{sender}: {data}")

    async def main(self, address):
        async with BleakClient(address) as client:
            await client.start_notify(ESP_GATT_UUID_SPP_DATA_NOTIFY, self.callback)
            # while True:
            #     await asyncio.sleep(1)
            # while True:
            #     a = input()
            #     await client.write_gatt_char(ESP_GATT_UUID_SPP_DATA_RECEIVE,
            #                            bytes(a.encode()))
            await asyncio.Event().wait()
            # while True:
            #     await client.start_notify(ESP_GATT_UUID_SPP_DATA_NOTIFY, callback)

# ble = BLE()
# asyncio.run(ble.main(address))

#
# async def main(address):
#     async with BleakClient(address) as client:
#         while True:
#             model_number = await client.read_gatt_char(ESP_GATT_UUID_SPP_DATA_RECEIVE)
#             print(model_number)
# asyncio.run(main(address))

# ############################################## read UUID adress ##################################
async def main(address: str):
     async with BleakClient(address) as client:
         svcs = await client.get_services()
         print("Services:")
         for service in svcs:
             print(service)
asyncio.run(main(address))

# if __name__ == "__main__":
#     ble = BLE()
#     asyncio.run(ble.main(address))
