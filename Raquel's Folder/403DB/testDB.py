import sqlite3
import bcrypt
import os

# Connect to a database
connection = sqlite3.connect('testDB.db')
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


# Sync inventory from CSV log
import csv
from datetime import datetime

CSV_PATH = "/home/am1/CAPSTONE/inventory_log.csv"

connection = sqlite3.connect('testDB.db')
cursor = connection.cursor()

# Read CSV and find the most recent row per bin
latest_by_bin = {}  # bin_id -> (timestamp_str, result_str)

with open(CSV_PATH, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        bin_id = int(row['bin'])
        timestamp_str = row['timestamp']
        result_str = row['result'].strip().upper()

        if bin_id not in latest_by_bin:
            latest_by_bin[bin_id] = (timestamp_str, result_str)
        else:
            existing_ts = datetime.strptime(latest_by_bin[bin_id][0], "%Y-%m-%d %H:%M:%S")
            new_ts = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
            if new_ts > existing_ts:
                latest_by_bin[bin_id] = (timestamp_str, result_str)

# For each bin with a recent CSV entry, update inventory if needed
for bin_id, (csv_ts_str, csv_result_str) in latest_by_bin.items():
    csv_quantity = 0 if csv_result_str == 'LO' else 1
    csv_ts = datetime.strptime(csv_ts_str, "%Y-%m-%d %H:%M:%S")

    row = cursor.execute(
        'SELECT quantity, datetime FROM inventory WHERE bin_id = ?', (bin_id,)
    ).fetchone()

    if row is None:
        cursor.execute(
            'INSERT INTO inventory(bin_id, component, quantity, datetime) VALUES (?, ?, ?, ?)',
            (bin_id, '', csv_quantity, csv_ts_str)
        )
        print(f"Inserted bin_id {bin_id}: quantity={csv_quantity}, datetime={csv_ts_str}")
    else:
        db_quantity, db_datetime_str = row
        db_ts = datetime.strptime(db_datetime_str, "%Y-%m-%d %H:%M:%S") if db_datetime_str else None

        if db_ts is None or (csv_ts > db_ts and csv_quantity != db_quantity):
            cursor.execute(
                'UPDATE inventory SET quantity = ?, datetime = ? WHERE bin_id = ?',
                (csv_quantity, csv_ts_str, bin_id)
            )
            print(f"Updated bin_id {bin_id}: quantity={csv_quantity}, datetime={csv_ts_str}")

connection.commit()
connection.close()
