import sqlite3


def main():
  db = sqlite3.connect("database.db")
  select_mode = 'Nothing in here'
  print("""
        Please select an action:
          1. Create Database
          2. Add device records
          3. Update device user id
          4. Delete device records
          5. Custom command
          0. Quit program
        """)
  try:
    while True:
      select_mode = input()
      if select_mode == '1':
        script_file = open("scripts/database_create.sql", "r")
        script = script_file.read()
        script_file.close()

        db.executescript(script)
        print("Database created (if not exists)")


      elif select_mode == '2':
        ids = list(map(int, input("List of device id to add (sperate by space):\n  ").split()))
        
        try:
          for id in ids:
            db.execute("""
                      INSERT OR IGNORE
                      INTO devices(id)
                      VALUES (?)""", (id,))
          db.commit()
        except:
          print("Failed, please try again!")


      elif select_mode == '3':
        params = list(map(int, input("Device id and new user_id (id, user_id):\n  ").split()))
        
        try:
          db.execute("""
                    UPDATE devices
                    SET user_id = ?
                    WHERE id = ?""", (params[1], params[0]))
          db.commit()
        except:
          print("Failed, please try again!")


      elif select_mode == '4':
        ids = list(map(int, input("List of device id to delete (sperate by space):\n  ").split()))
        
        try:
          for id in ids:
            db.execute("""
                      DELETE
                      FROM devices
                      WHERE id = ?""", (id,))
          db.commit()
        except:
          print("Failed, please try again!")


      elif select_mode == '5':
        command = input("Manual command:\n  ")
        
        try:
          result = db.execute(f"{command}")
          db.commit()
          for row in result:
            print(row)
        except:
          print("Failed, please try again!")

      elif select_mode == '0':
        break
  
  except KeyboardInterrupt:
    pass

  db.close()


if __name__ == "__main__":
  main()