import sqlite3
import bcrypt
import os

# Connect to a database
# WAL mode allows app.py and testDB.py to access the DB concurrently without locking errors
connection = sqlite3.connect('testDB.db', timeout=30)
connection.execute('PRAGMA journal_mode=WAL')
# connection.execute('DROP TABLE IF EXISTS users')
# connection.execute('DROP TABLE IF EXISTS inventory')
# connection.execute('DROP TABLE IF EXISTS activity_logs')


# create cursor to execute and fetch SQL queries
cursor = connection.cursor()

# FIXME - add date and time of last edit to all tables
# create users table, only admin_perm should have access
cursor.execute('''CREATE TABLE IF NOT EXISTS users(
               username TEXT UNIQUE NOT NULL, 
               password TEXT NOT NULL, 
               refill_perm INTEGER, 
               admin_perm INTEGER NOT NULL)''') # 0 = false, 1 = true

# create inventory table, only refill_perm or admin_perm should have access
cursor.execute('''CREATE TABLE IF NOT EXISTS inventory(
               bin_id INTEGER UNIQUE NOT NULL, 
               component TEXT NOT NULL, 
               quantity INTEGER NOT NULL,
               datetime TEXT)''') # 0 = low, 1 = high

# create api key table, only admin_perm should have access
cursor.execute('''CREATE TABLE IF NOT EXISTS api_keys(
               service TEXT NOT NULL, 
               api_key TEXT NOT NULL)''')

# create user activity logs table
cursor.execute('''CREATE TABLE IF NOT EXISTS activity_logs(
               username TEXT, 
               access_req TEXT, 
               datetime TEXT)''')

# create a default admin user in users table
admin_password = os.environ.get("ADMIN_PASSWORD")
if not admin_password:
    raise ValueError("ADMIN_PASSWORD environment variable is not set")

hashed = bcrypt.hashpw(admin_password.encode(), bcrypt.gensalt()).decode()

cursor.execute('''INSERT INTO users(username, password, refill_perm, admin_perm)
                VALUES ('superuser', ?, 1, 1)
                ON CONFLICT(username) DO NOTHING''', (hashed,))




connection.commit()

# close database connection
connection.close()
