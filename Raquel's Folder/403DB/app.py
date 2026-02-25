from flask import Flask, render_template, request, redirect, url_for, session, jsonify
import sqlite3
import os
import signal
from datetime import timedelta, datetime
from functools import wraps

app = Flask(__name__)
app.secret_key = 'your-secret-key-change-in-production'
app.permanent_session_lifetime = timedelta(minutes=15)


def get_db_connection():
    conn = sqlite3.connect('testDB.db')
    conn.row_factory = sqlite3.Row
    return conn


def login_required(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if 'username' not in session:
            return redirect(url_for('login'))
        return f(*args, **kwargs)

    return decorated_function


def log_activity(username, description):
    """Log user activity to the activity_logs table with Central Time timestamp."""
    conn = get_db_connection()
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    conn.execute('INSERT INTO activity_logs (username, access_req, datetime) VALUES (?, ?, ?)',
                 (username, description, timestamp))
    conn.commit()
    conn.close()


@app.route('/')
def index():
    if 'username' in session:
        return redirect(url_for('dashboard'))
    return redirect(url_for('login'))


@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']

        conn = get_db_connection()
        user = conn.execute('SELECT * FROM users WHERE username = ?',
                            (username,)).fetchone()
        conn.close()

        if user and user['password'] == password:
            session.permanent = True
            session['username'] = user['username']
            session['is_admin'] = user['admin_perm']
            session['refill_perm'] = user['refill_perm']
            return redirect(url_for('dashboard'))

        return render_template('login.html', error='Invalid credentials')

    return render_template('login.html')


@app.route('/create_account', methods=['GET', 'POST'])
def create_account():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        confirm_password = request.form['confirm_password']

        if password != confirm_password:
            return render_template('create_account.html', error='Passwords do not match')

        conn = get_db_connection()
        existing = conn.execute('SELECT * FROM users WHERE username = ?', (username,)).fetchone()
        if existing:
            conn.close()
            return render_template('create_account.html', error='Username already exists')

        try:
            conn.execute('INSERT INTO users (username, password, refill_perm, admin_perm) VALUES (?, ?, ?, ?)',
                         (username, password, 1, 0))
            conn.commit()
            conn.close()
            return redirect(url_for('login'))
        except Exception as e:
            conn.close()
            return render_template('create_account.html', error=f'Error creating account: {str(e)}')

    return render_template('create_account.html')


@app.route('/shutdown', methods=['POST'])
def shutdown():
    os.kill(os.getpid(), signal.SIGINT)
    return 'Server shutting down...'


@app.route('/logout')
def logout():
    if 'username' in session:
        log_activity(session['username'], "Logged out")
    session.clear()
    return redirect(url_for('login'))


@app.route('/dashboard')
@login_required
def dashboard():
    available_tables = []

    if session.get('refill_perm') == 1 or session.get('is_admin') == 1:
        available_tables.append('inventory')

    if session.get('is_admin') == 1:
        available_tables.extend(['users', 'api_keys', 'activity_logs'])  # Moved activity_logs here

    return render_template('dashboard.html',
                           username=session['username'],
                           tables=available_tables)

@app.route('/view/<table_name>')
@login_required
def view_table(table_name):
    allowed_tables = ['users', 'inventory', 'activity_logs', 'api_keys']
    if table_name not in allowed_tables:
        return "Access denied", 403

    log_activity(session['username'], f"Viewed table {table_name}")

    conn = get_db_connection()

    cursor = conn.execute(f'PRAGMA table_info({table_name})')
    columns = [col['name'] for col in cursor.fetchall()]

    if table_name == 'activity_logs':
        rows = conn.execute(f'SELECT * FROM {table_name} ORDER BY datetime DESC').fetchall()
    elif table_name == 'inventory':
        rows = conn.execute(f'SELECT * FROM {table_name} ORDER BY bin_id ASC').fetchall()
    else:
        rows = conn.execute(f'SELECT * FROM {table_name}').fetchall()
    conn.close()

    data = []
    for row in rows:
        row_dict = dict(row)
        if table_name == 'users' and 'password' in row_dict:
            row_dict['password'] = '********'
        data.append(row_dict)

    can_edit = session.get('is_admin') == 1 and table_name not in ['users', 'activity_logs']

    return render_template('view_table.html',
                           table_name=table_name,
                           columns=columns,
                           data=data,
                           can_edit=can_edit)


@app.route('/edit_user_permissions/<username>', methods=['GET', 'POST'])
@login_required
def edit_user_permissions(username):
    if session.get('is_admin') != 1:
        return "Access denied", 403

    if username == 'superuser':
        return "Cannot edit superuser permissions", 403

    conn = get_db_connection()

    if request.method == 'POST':
        refill_perm = 1 if request.form.get('refill_perm') else 0
        admin_perm = 1 if request.form.get('admin_perm') else 0

        conn.execute('UPDATE users SET refill_perm = ?, admin_perm = ? WHERE username = ?',
                     (refill_perm, admin_perm, username))
        conn.commit()
        conn.close()
        log_activity(session['username'], f"Edited permissions for user {username} in users")
        return redirect(url_for('view_table', table_name='users'))

    user = conn.execute('SELECT * FROM users WHERE username = ?', (username,)).fetchone()
    conn.close()

    if not user:
        return "User not found", 404

    return render_template('edit_permissions.html', user=dict(user))

@app.route('/api/insert/<table_name>', methods=['POST'])
@login_required
def insert_row(table_name):
    if session.get('is_admin') != 1 or table_name == 'users':
        return jsonify({'success': False, 'error': 'Permission denied'}), 403

    allowed_tables = ['inventory', 'activity_logs', 'api_keys']
    if table_name not in allowed_tables:
        return jsonify({'success': False, 'error': 'Invalid table'}), 400

    data = request.json
    conn = get_db_connection()

    try:
        row_id = None
        if table_name == 'inventory':
            conn.execute('INSERT INTO inventory (bin_id, component, quantity) VALUES (?, ?, ?)',
                         (data['bin_id'], data['component'], data['quantity']))
            row_id = data['bin_id']
        elif table_name == 'api_keys':
            conn.execute('INSERT INTO api_keys (service, api_key) VALUES (?, ?)',
                         (data['service'], data['api_key']))
            row_id = data['service']
        elif table_name == 'activity_logs':
            conn.execute('INSERT INTO activity_logs (username, access_req, datetime) VALUES (?, ?, ?)',
                         (data['username'], data['access_req'], data['datetime']))
            row_id = data['username']

        conn.commit()
        conn.close()
        log_activity(session['username'], f"Added row ({row_id}) to {table_name}")
        return jsonify({'success': True})
    except Exception as e:
        conn.close()
        return jsonify({'success': False, 'error': str(e)}), 400


@app.route('/api/update/<table_name>', methods=['POST'])
@login_required
def update_row(table_name):
    if session.get('is_admin') != 1 or table_name == 'users':
        return jsonify({'success': False, 'error': 'Permission denied'}), 403

    allowed_tables = ['inventory', 'activity_logs', 'api_keys']
    if table_name not in allowed_tables:
        return jsonify({'success': False, 'error': 'Invalid table'}), 400

    data = request.json
    conn = get_db_connection()

    try:
        row_id = None
        if table_name == 'inventory':
            conn.execute('UPDATE inventory SET component = ?, quantity = ? WHERE bin_id = ?',
                         (data['component'], data['quantity'], data['bin_id']))
            row_id = data['bin_id']
        elif table_name == 'api_keys':
            conn.execute('UPDATE api_keys SET api_key = ? WHERE service = ?',
                         (data['api_key'], data['service']))
            row_id = data['service']

        conn.commit()
        conn.close()
        log_activity(session['username'], f"Edited row ({row_id}) in {table_name}")
        return jsonify({'success': True})
    except Exception as e:
        conn.close()
        return jsonify({'success': False, 'error': str(e)}), 400


@app.route('/api/delete/<table_name>', methods=['POST'])
@login_required
def delete_row(table_name):
    if session.get('is_admin') != 1 or table_name == 'users':
        return jsonify({'success': False, 'error': 'Permission denied'}), 403

    allowed_tables = ['inventory', 'activity_logs', 'api_keys']
    if table_name not in allowed_tables:
        return jsonify({'success': False, 'error': 'Invalid table'}), 400

    data = request.json
    conn = get_db_connection()

    try:
        row_id = None
        if table_name == 'inventory':
            conn.execute('DELETE FROM inventory WHERE bin_id = ?', (data['bin_id'],))
            row_id = data['bin_id']
        elif table_name == 'api_keys':
            conn.execute('DELETE FROM api_keys WHERE service = ?', (data['service'],))
            row_id = data['service']

        conn.commit()
        conn.close()
        log_activity(session['username'], f"Deleted row ({row_id}) from {table_name}")
        return jsonify({'success': True})
    except Exception as e:
        conn.close()
        return jsonify({'success': False, 'error': str(e)}), 400


if __name__ == '__main__':
    app.run(debug=True, host='127.0.0.1', port=5000)