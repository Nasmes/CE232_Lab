# import struct

# x, y = struct.unpack('2f', b'\xdb\x0fI@\x0b\x01I4')
# print(""",\n  {"lat": %3.05f, "long": %3.05f}"""%(x, y))


###############################
# import asyncio
# import websockets

# async def echo(websocket):
#   async for message in websocket:
#     print ("Received message!")
#     # await websocket.send(message)
#     if message[0] == ord('i'):
#       await websocket.send('i')

# async def main():
#   async with websockets.serve(echo, "0.0.0.0", 8765):
#     await asyncio.Future()

# try:
#   asyncio.run(main())
# except:
#   pass


########################################
# import asyncio
# import websockets

# async def main():
#   async with websockets.connect("ws://localhost:8765") as ws:
#     await ws.send("Hello")
#     while True:
#       message = await ws.recv()
#       print(message)
#       await asyncio.sleep(0.1)

# asyncio.run(main())

import sqlite3

db = sqlite3.connect("../Database/database.db")

# db.execute("DROP TABLE devices")

# db.execute("""
#            CREATE TABLE devices(
#               id INTEGER PRIMARY KEY,
#               name TEXT,
#               user_id INTEGER,
#               status BOOLEAN NOT NULL DEFAULT 0,
#               FOREIGN KEY (user_id) REFERENCES Users(id)
#            )
#            """)

# db.execute("""
#            INSERT INTO devices (id, user_id)
#            VALUES (1, 20), (2, 14), (5, 56)
#            """)
# db.commit()

result = db.execute("SELECT * FROM devices")

for row in result:
  print(row)

db.close()