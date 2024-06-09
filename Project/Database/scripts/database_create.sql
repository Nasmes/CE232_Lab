BEGIN;

CREATE TABLE IF NOT EXISTS
users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    username TEXT NOT NULL,
    password TEXT NOT NULL,
    dv_list_ver INTEGER DEFAULT 0,
    vid_list_ver INTEGER DEFAULT 0,
    tl_list_ver INTEGER DEFAULT 0,
    created_at TEXT
);

CREATE TABLE IF NOT EXISTS
devices (
    id INTEGER PRIMARY KEY,
    name TEXT NULL,
    user_id INTEGER,
    status BOOLEAN NOT NULL DEFAULT 0,
    FOREIGN KEY (user_id) REFERENCES users(id)
);

CREATE TABLE IF NOT EXISTS
videos (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    filename TEXT NOT NULL,
    user_id INTEGER,
    created_at TEXT,
    FOREIGN KEY (user_id) REFERENCES users(id)
);

CREATE TABLE IF NOT EXISTS
timelines (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    filename TEXT NOT NULL,
    user_id INTEGER,
    created_at TEXT,
    FOREIGN KEY (user_id) REFERENCES users(id)
);

COMMIT;