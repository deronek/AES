import asyncio
import time

from bleak import BleakClient


address = "E8:31:CD:C4:76:62"
MODEL_NBR_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"

################################ read MAC adress##############################################
# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         print(d)
#
# asyncio.run(main())

def callback(sender: int, data: bytearray):
    print(f"{sender}: {data}")


async def main(address):
    async with BleakClient(address) as client:
        while True:
            await client.start_notify(MODEL_NBR_UUID, callback)


asyncio.run(main(address))






################################################ read UUID adres ##################################
# async def main(address: str):
#     async with BleakClient(address) as client:
#         svcs = await client.get_services()
#         print("Services:")
#         for service in svcs:
#             print(service)
#
# asyncio.run(main(address))
