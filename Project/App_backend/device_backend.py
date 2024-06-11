import os
import time
import asyncio
import websockets
import cv2
import numpy as np
import sqlite3
import struct
from io import BytesIO 
from PIL import Image


##### Params ##################################################################
PROJECT_PATH = "/home/nasmes/CE232_Project"
SERVER_PORT = 6699

VIDEO_FPS = 5.0
VIDEO_SIZE = (640,480)

MUTEX_WAIT_TIME = 50 #ms

DEBUG = True


"""
key: (int) device id
value: [(int) user_id,
        (json) timeline_handler, (str) timeline_file_name
        (cv2.videowriter) video_handler, (str) video_file_name, (byte) video_frame, (int) video_mutex, (bool) is_vid_closed?]
"""
device_handlers = {}

INDEX_USER_ID     = 0
INDEX_USER_CHANGE = 1
INDEX_TL_HDL      = 2
INDEX_TL_NAME     = 3
INDEX_VID_HDL     = 4
INDEX_VID_NAME    = 5
INDEX_VID_FRAME   = 6
INDEX_VID_MUTEX   = 7
INDEX_VID_CLOSE   = 8


##### Database ################################################################
class database():
  def __init__(self, db_path = f"{PROJECT_PATH}/Database/database.db"):
    if os.path.isfile(db_path):
      self.con = sqlite3.connect(db_path)

  def get_device_info(self, device_id):
    if device_id:
      try:
        result = self.con.execute(""" SELECT name, user_id FROM devices WHERE id = ? """, (device_id,))
        return result.fetchone()
      except:
        return None

  def update_device_status(self, device_id, status):
    if device_id:
      try:
        self.con.execute(""" UPDATE devices SET status = ? where id = ? """, (status, device_id))
        self.con.commit()
      except:
        print("Database Error when update device's status!")

  def add_video_record(self, user_id, file_name):
    if user_id:
      try:
        created_time = time.strftime("%H:%M - %d/%m/%Y")
        self.con.execute(""" INSERT INTO videos (filename, user_id, created_at) VALUES (?, ?, ?)""",
                         (file_name, user_id, created_time))
        self.con.commit()
      except:
        print("Database Error when insert video record!")

  def add_timeline_record(self, user_id, file_name):
    if user_id:
      try:
        created_time = time.strftime("%H:%M - %d/%m/%Y")
        self.con.execute(""" INSERT INTO timelines (filename, user_id, created_at) VALUES (?, ?, ?)""",
                         (file_name, user_id, created_time))
        self.con.commit()
      except:
        print("Database Error when insert timelines record!")

db = database()

##### Misc functions ##########################################################
def jpeg2cvmat(jpeg):
  frame = BytesIO(jpeg)
  frame = Image.open(frame)
  frame = np.array(frame)
  cvmat = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

  return cvmat

def video_write_gst_pipeline(filename= "test.mp4"):
  return(
    "appsrc "
    "! video/x-raw, format=(string)BGR "
    "! videoconvert "
    "! video/x-raw, format=(string)BGRx "
    "! nvvidconv "
    "! x264enc pass=5 qp-max=25 speed-preset=1 "
    "! h264parse "
    "! qtmux "
    "! filesink location=%s "
    % (
      filename
    )
  )

##### Core functions ##########################################################
def release_device_handler(device_id):
  global device_handlers
  global db

  if device_id in device_handlers:

    if device_handlers[device_id][INDEX_TL_HDL]:
      # Save file
      device_handlers[device_id][INDEX_TL_HDL].write("\n]")
      device_handlers[device_id][INDEX_TL_HDL].close()
      device_handlers[device_id][INDEX_TL_HDL] = None
      
      # Create record
      user_id = device_handlers[device_id][INDEX_USER_ID]
      file_name = device_handlers[device_id][INDEX_TL_NAME]
      db.add_timeline_record(user_id, file_name)

    if device_handlers[device_id][INDEX_VID_HDL]:
      # Signal the task to close
      device_handlers[device_id][INDEX_VID_CLOSE] = 1


async def video_write_task(device_id):
  global device_handlers
  global db
  
  while True:
    try:
      start_time = time.time()
      # If this device doesn't have video writer, exit
      if (device_handlers[device_id][INDEX_VID_HDL] is None
          or device_handlers[device_id][INDEX_VID_FRAME] is None):
        break

      # Wait mutex
      while device_handlers[device_id][INDEX_VID_MUTEX] == 0:
        await asyncio.sleep(0.0001)
      # Take mutex
      device_handlers[device_id][INDEX_VID_MUTEX] = 0

      # If system requested to close video
      if device_handlers[device_id][INDEX_VID_CLOSE]:
        # Release the video
        device_handlers[device_id][INDEX_VID_HDL].release()
        device_handlers[device_id][INDEX_VID_HDL] = None
        device_handlers[device_id][INDEX_VID_FRAME] = None

        # Add file record
        user_id = device_handlers[device_id][INDEX_USER_ID]
        file_name = device_handlers[device_id][INDEX_VID_NAME]
        db.add_video_record(user_id, file_name)

        # Break and close the task
        break

      else:
        # Write the next frame
        frame = device_handlers[device_id][INDEX_VID_FRAME]
        device_handlers[device_id][INDEX_VID_HDL].write(frame)

      # Give mutex
      device_handlers[device_id][INDEX_VID_MUTEX] = 1

      await asyncio.sleep(0.2 - (time.time() - start_time))

    except:
      # If a video is writing
      if device_handlers[device_id][INDEX_VID_HDL] is not None:
        # Release the video
        device_handlers[device_id][INDEX_VID_HDL].release()
        device_handlers[device_id][INDEX_VID_HDL] = None
        device_handlers[device_id][INDEX_VID_FRAME] = None

        # Add file record
        user_id = device_handlers[device_id][INDEX_USER_ID]
        file_name = device_handlers[device_id][INDEX_VID_NAME]
        db.add_video_record(user_id, file_name)
      break

  device_handlers[device_id][INDEX_VID_CLOSE] = 1


async def request_gate(websocket):
  global device_handlers
  global db
  device_id = 0
  device_name = None

  video_task = None

  while True:
    try:
      message = await websocket.recv()
      # If device_id is mismatched between server and message, abort message
      if device_id != 0 and device_id != int(message[1]):
        continue
      
      # Make sure device is still assigned to the owner
      if device_id in device_handlers:
        if device_handlers[device_id][INDEX_USER_CHANGE]:
          print(f"Device's owner changed! Logging device out!\n ID: {device_id}")

          # Inform device
          await websocket.send('c')

          # Release device's handlers
          release_device_handler(device_id)
          if video_task:
            await video_task
            video_task = None
          device_handlers.pop(device_id)

          # Update device status on database
          db.update_device_status(device_id, 0)

          # Next message
          continue

      # Video frame message
      if (message[0] == ord('v')):
        # Check if this device logged in properly
        if device_id in device_handlers:
          # Convert message to cv2 MAT object
          frame = jpeg2cvmat(message[2:])

          # Wait mutex
          for _ in range(MUTEX_WAIT_TIME):
            if device_handlers[device_id][INDEX_VID_MUTEX] == 0:
              await asyncio.sleep(0.001)
            else:
              # Take mutex
              device_handlers[device_id][INDEX_VID_MUTEX] = 0

              # Update the frame content
              device_handlers[device_id][INDEX_VID_FRAME] = frame

              # Check if video handler already exist, create one if not
              if device_handlers[device_id][INDEX_VID_HDL] is None:
                print(f"New video from device!\n ID: {device_id}")

                # Try to create folder path
                videos_folder_path = f"{PROJECT_PATH}/device-management/public/videos"
                os.makedirs(videos_folder_path, 755, True)

                # Create video writer
                video_file_name = f"{device_name}_{int(time.time())}".replace(' ', '_')
                video_writer = cv2.VideoWriter(video_write_gst_pipeline(f"{videos_folder_path}/{video_file_name}.mp4"),
                                               cv2.CAP_GSTREAMER, VIDEO_FPS, VIDEO_SIZE)
                
                # Update video handler
                device_handlers[device_id][INDEX_VID_NAME] = video_file_name
                device_handlers[device_id][INDEX_VID_HDL] = video_writer
                device_handlers[device_id][INDEX_VID_CLOSE] = 0

                # Create video write task
                video_task = asyncio.create_task(video_write_task(device_id))
              
              # Give Mutex
              device_handlers[device_id][INDEX_VID_MUTEX] = 1
              break

      # Video close message
      elif (message[0] == ord("c")):
        # Check if this device logged in properly
        if device_id in device_handlers:
          print(f"Device finished video.\n ID: {device_id}")
          # If video task is present
          if video_task:
            # Signal to close the video task
            device_handlers[device_id][INDEX_VID_CLOSE] = 1
            await video_task
            video_task = None

      # Timeline Data message
      elif (message[0] == ord('t')):
        # Check if this device logged in properly
        if device_id in device_handlers:

          # Get lattitude and longitude data
          lat, long = struct.unpack("2f", message[2:])

          # Check if timeline handler already exist, create one if not
          if device_handlers[device_id][INDEX_TL_HDL] is None:
            print(f"New timeline from device!\n ID: {device_id}")

            # Try to create folder path
            timelines_folder_path = f"{PROJECT_PATH}/device-management/public/timelines/"
            os.makedirs(timelines_folder_path, 755, True)

            # Create file writer
            timeline_file_name = f"{device_name}_{int(time.time())}"
            file = open(f"{timelines_folder_path}/{timeline_file_name}.json", 'w')

            # Update handler info
            device_handlers[device_id][INDEX_TL_NAME] = timeline_file_name
            device_handlers[device_id][INDEX_TL_HDL] = file
            
            # Write the first possition
            device_handlers[device_id][INDEX_TL_HDL].write("""[\n  {"lat": %3.05f, "lng": %3.05f}"""%(lat, long))

          else:
            # Append the position
            device_handlers[device_id][INDEX_TL_HDL].write(""",\n  {"lat": %3.05f, "lng": %3.05f}"""%(lat, long))

      # Device Login message
      elif (message[0] == ord('i')):
        # Make sure that device is not logged in
        if device_id == 0 and int(message[1]) not in device_handlers:
          # Default user_id value
          user_id = 0
          # Assign device id
          device_id = int(message[1])

          # Get device's name & user_id from device's record on database
          device_record = db.get_device_info(device_id)
          if device_record is not None:
            device_name = device_record[0]
            user_id = device_record[1] if device_record[1] is not None else 0
            

          if user_id > 0 :
            # If device is registered, create a handler
            device_handlers[device_id] = [user_id, 0,
                                          None, None,
                                          None, None, None, 1, 1]

            # Update device status on database
            db.update_device_status(device_id, 1)

            # Respond back to device
            await websocket.send('i')

            # Successfully logged in, continue to next message
            print(f"Device log in: success\n ID: {device_id} - UserID: {user_id}")
            continue
          
        # If login failed, clear device_id and inform device
        print(f"Device log in: unregistered\n ID: {device_id}")
        device_id = 0
        await websocket.send('u')

      # Device Logout message
      elif (message[0] == ord('o')):
        # If device has logged in
        if device_id in device_handlers:
          print(f"Device log out!\n ID: {device_id}")

          # Inform device
          await websocket.send('o')

          # Release device's handlers
          release_device_handler(device_id)
          if video_task:
            await video_task
            video_task = None
          device_handlers.pop(device_id)

          # Update device status on database
          db.update_device_status(device_id, 0)

    except websockets.ConnectionClosed:
      # If device hasn't logged out properly
      if device_id in device_handlers:
        print(f"Lost connection, logging device out!\n ID: {device_id}")

        # Release device's handlers
        release_device_handler(device_id)
        if video_task:
          await video_task
          video_task = None
        device_handlers.pop(device_id)

        # Update device status on database
        db.update_device_status(device_id, 0)
    
      break

    except KeyboardInterrupt:
      # If device has logged in
      if device_id in device_handlers:
        print(f"Logging device out!\n ID: {device_id}")

        # Inform the device
        await websocket.send('o')

        # Release device's handlers
        release_device_handler(device_id)
        if video_task:
          await video_task
          video_task = None
        device_handlers.pop(device_id)

        # Update device status on database
        db.update_device_status(device_id, 0)

      break


async def main():
  async with websockets.serve(request_gate, "", SERVER_PORT):
    await asyncio.Future()  # run forever

# ngrok.set_auth_token("2ftUo8P4VomY5RxX9VJ4KuawHyQ_5u8QjT3zd36K8m38ZBT5b")
# url = ngrok.connect(SERVER_PORT, "tcp")
# print(url)

try:
  print(f"Starting webserver at port: {SERVER_PORT}")
  asyncio.run(main())
except KeyboardInterrupt:
  # ngrok.kill()
  pass