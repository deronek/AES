import asyncio
from bleak import BleakScanner
from bleak import BleakClient



class SensorData:
    def __init__(self, data: bytearray):
        self.id = data[0]
        self.checksum = data[-1]
        self.data = []
        for b in data[1:-1]:
            data.append(b)




address = "E8:31:CD:C4:76:62"
MODEL_NBR_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"

################################ read MAC adress ##############################################
# async def main():
#     devices = await BleakScanner.discover()
#     for d in devices:
#         print(d)
# asyncio.run(main())

def callback(sender: int, data: bytearray):
   print(f"{sender}: {data}")

async def main(address):
   async with BleakClient(address) as client:
       while True:
           await client.start_notify(MODEL_NBR_UUID, callback)
asyncio.run(main(address))






################################################ read UUID adress ##################################
# async def main(address: str):
#      async with BleakClient(address) as client:
#          svcs = await client.get_services()
#          print("Services:")
#          for service in svcs:
#              print(service)
# asyncio.run(main(address))

if __name__ == "__main__":
    main()