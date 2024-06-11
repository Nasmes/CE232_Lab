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

# import sqlite3

# db = sqlite3.connect("../Database/database.db")

# # db.execute("DROP TABLE devices")

# # db.execute("""
# #            CREATE TABLE devices(
# #               id INTEGER PRIMARY KEY,
# #               name TEXT,
# #               user_id INTEGER,
# #               status BOOLEAN NOT NULL DEFAULT 0,
# #               FOREIGN KEY (user_id) REFERENCES Users(id)
# #            )
# #            """)

# # db.execute("""
# #            INSERT INTO devices (id, user_id)
# #            VALUES (1, 20), (2, 14), (5, 56)
# #            """)
# # db.commit()

# result = db.execute("SELECT * FROM devices")

# for row in result:
#   print(row)

# db.close()

# import cv2

# invideo = cv2.VideoCapture("video1.mp4")

# def gst_camera_pipeline(filename= "test.mp4"):
#   return(
#     "appsrc "
#     "! video/x-raw, format=(string)BGR "
#     "! videoconvert "
#     "! video/x-raw, format=(string)BGRx "
#     "! nvvidconv "
#     "! x264enc pass=5 qp-max=25 speed-preset=1"
#     "! h264parse "
#     "! qtmux "
#     "! filesink location=%s "
#     % (
#       filename
#     )
#   )

# video = cv2.VideoWriter(gst_camera_pipeline("test.mp4"), cv2.CAP_GSTREAMER, 25.0, (int(invideo.get(3)), int(invideo.get(4))))
# print(invideo.isOpened(), video.isOpened())

# count = 0
# ret, frame = invideo.read()
# while ret:
#   count += 1
#   video.write(frame)
#   ret, frame = invideo.read()

# print(count)

# video.release()
# invideo.release()

import asyncio
from pyngrok import ngrok

ngrok.set_auth_token("2ftUo8P4VomY5RxX9VJ4KuawHyQ_5u8QjT3zd36K8m38ZBT5b")
print(ngrok.connect(8765, "tcp"))
print(ngrok.connect(3000, "http"))

async def main():
  await asyncio.Future()

try:
  asyncio.run(main())
except KeyboardInterrupt:
  ngrok.kill()
  pass