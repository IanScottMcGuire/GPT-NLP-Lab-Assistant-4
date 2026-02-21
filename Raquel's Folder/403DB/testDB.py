import sqlite3

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
               quantity INTEGER NOT NULL)''') # 0 = low, 1 = high

# create api key table, only admin_perm should have access
cursor.execute('''CREATE TABLE IF NOT EXISTS api_keys(
               service TEXT NOT NULL, 
               api_key TEXT NOT NULL)''')

# create user activity logs table
cursor.execute('''CREATE TABLE IF NOT EXISTS activity_logs(
               username TEXT, 
               access_req TEXT, 
               datetime TEXT)''')

# create 1 test superuser in users table
cursor.execute('''INSERT INTO users(username, password, refill_perm, admin_perm)
               VALUES ('superuser', 'testPW', 1, 1)
               ON CONFLICT(username) DO NOTHING''')

# add 4 bins and test data to inventory table
cursor.execute('''INSERT INTO inventory(bin_id, component, quantity)
               VALUES ('1', 'Resistor - 1k', 1),
               ('2', 'Capacitor - 10uF', 1),
               ('3', 'Op Amp', 1),
               ('4', 'LED - Red', 1)
               ON CONFLICT(bin_id) DO NOTHING''')

connection.commit()

# -----START OF CODE-----

# take user input from console to request access to view/edit a table

# check user permissions, block request if user doesn't have correct perms

# take user input to edit table

# -----END OF CODE-----


# -----START OF CODE-----
# FIXME - add table requested and actions performed to activity_logs table
# take user input from console to request access to view/edit a table
username = input("Enter username: ")
password = input("Enter password: ")
table_name = input("Enter table name to access (users/inventory/api_keys/activity_logs): ")

# FIXME: move permission check AFTER valid user check
# check user permissions, block request if user doesn't have correct perms
cursor.execute('SELECT refill_perm, admin_perm FROM users WHERE username = ? AND password = ?',
               (username, password))
user = cursor.fetchone()

if user is None:
    print("Invalid username or password")
    cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime")))',
                   (username, f"Failed login attempt for {table_name}"))
    connection.commit()
else:
    # FIXME: add activity logging for successful login
    cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime")))',
                   (username, f"Logged in"))
    connection.commit()

    refill_perm, admin_perm = user

    # Check permissions based on table
    access_granted = False
    if table_name == 'users' or table_name == 'api_keys':
        access_granted = (admin_perm == 1)
    elif table_name == 'inventory':
        access_granted = (refill_perm == 1 or admin_perm == 1)
    elif table_name == 'activity_logs':
        access_granted = True


    if not access_granted:
        # Log the access request
        cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime"))',
                       (username, f"Access denied for {table_name}"))
        connection.commit()
        print(f"Access denied. You don't have permission to access {table_name}")
    else:
        print(f"Access granted to {table_name}")

        # take user input to edit table
        action = input("Enter action (view/insert/update/delete): ")

        if action == 'view':
            cursor.execute(f'SELECT * FROM {table_name}')
            rows = cursor.fetchall()
            for row in rows:
                print(row)
            # Log the view request
            cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime"))',
                           (username, f"Viewed {table_name}"))
            connection.commit()

        elif action == 'insert':
            if table_name == 'users':
                u = input("Username: ")
                p = input("Password: ")
                r = int(input("Refill permission (0/1): "))
                a = int(input("Admin permission (0/1): "))
                cursor.execute('INSERT INTO users VALUES (?, ?, ?, ?)', (u, p, r, a))
            elif table_name == 'inventory':
                b = int(input("Bin ID: "))
                c = input("Component: ")
                q = int(input("Quantity (Low - 0/High - 1): "))
                cursor.execute('INSERT INTO inventory VALUES (?, ?, ?)', (b, c, q))
            elif table_name == 'api_keys':
                s = input("Service: ")
                k = input("API Key: ")
                cursor.execute('INSERT INTO api_keys VALUES (?, ?)', (s, k))
            connection.commit()
            # Log the insert request
            cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime"))',
                           (username, f"Data inserted in {table_name}"))
            connection.commit()
            print("Record inserted")

        elif action == 'update':
            print("Specify your update query")
            # Basic example for inventory
            if table_name == 'inventory':
                bin_id = int(input("Enter bin_id to update: "))
                new_qty = int(input("Enter new quantity: "))
                cursor.execute('UPDATE inventory SET quantity = ? WHERE bin_id = ?',
                               (new_qty, bin_id))
                connection.commit()
                print("Inventory updated")

            if table_name == 'api_keys':
                service = input("Enter service to update: ")
                new_key = input("Enter new API key: ")
                cursor.execute('UPDATE api_keys SET api_key = ? WHERE service = ?',
                               (new_key, service))
                connection.commit()
                print("API key updated")

            # Log the update request
            cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime"))',
                           (username, f"Data updated in {table_name}"))
            connection.commit()

        # ensure the console displays the row to be deleted to the user and gets their confirmation for deletion
        elif action == 'delete':
            print("Specify information to delete")

            if table_name == 'inventory':
                bin_id = int(input("Enter bin_id to delete: "))
                cursor.execute(f'SELECT * FROM {table_name} WHERE bin_id = ?', (bin_id,))
                rows = cursor.fetchall()
                for row in rows:
                    print(row)
                confirm_delete = input("Are you sure you want to delete this row? (y/n)")
                if confirm_delete == 'y':
                    cursor.execute('DELETE FROM inventory WHERE bin_id = ?', (bin_id,))
                    connection.commit()
                    print("Deleted row")
                else:
                    print("Exiting")

            if table_name == 'api_keys':
                print("API keys: ")
                cursor.execute(f'SELECT * FROM {table_name}')
                rows = cursor.fetchall()
                for row in rows:
                    print(row)

                key = input("Enter API key to delete: ")
                cursor.execute(f'SELECT * FROM {table_name} WHERE api_key = ?', (key,))
                rows = cursor.fetchall()
                for row in rows:
                    print(row)
                confirm_delete = input("Are you sure you want to delete this row? (y/n)")
                if confirm_delete == 'y':
                    cursor.execute('DELETE FROM {table_name} WHERE api_key = ?', (key,))
                    connection.commit()
                    print("Deleted row")
                else:
                    print("Exiting")

            if table_name == 'users':
                username = input("Enter username to delete: ")
                cursor.execute(f'SELECT * FROM {table_name} WHERE username = ?', (username,))
                rows = cursor.fetchall()
                for row in rows:
                    print(row)
                confirm_delete = input("Are you sure you want to delete this row? (y/n)")
                if (confirm_delete == 'y') and (username != "superuser"):
                    cursor.execute('DELETE FROM users WHERE username = ?', (username,))
                    connection.commit()
                    print("Deleted row")
                elif username == "superuser":
                    print("Cannot delete superuser")
                else:
                    print("Exiting")

            # Log the delete request
            cursor.execute('INSERT INTO activity_logs VALUES (?, ?, datetime("now", "localtime"))',
                           (username, f"Row deleted from {table_name}"))
            connection.commit()

# -----END OF CODE-----


# close database connection
connection.close()
