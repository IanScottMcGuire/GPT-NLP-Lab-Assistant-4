import sqlite3

DB_NAME = "faq.db"

def init_db():
    """Create the database and FAQ table if not exists."""
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS faqs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            course_code TEXT NOT NULL,
            question TEXT NOT NULL,
            answer TEXT NOT NULL
        )
    ''')
    conn.commit()
    conn.close()
    print("Database initialized.")

def add_faq(course_code, question, answer):
    """Insert a new FAQ entry."""
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute("INSERT INTO faqs (course_code, question, answer) VALUES (?, ?, ?)",
              (course_code, question, answer))
    conn.commit()
    conn.close()
    print(f"FAQ added for {course_code}: {question}")

def get_all_faqs():
    """Fetch all FAQ entries."""
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute("SELECT * FROM faqs")
    data = c.fetchall()
    conn.close()
    return data

def search_faqs(keyword):
    """Search FAQ entries by keyword."""
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute("SELECT * FROM faqs WHERE question LIKE ? OR answer LIKE ?", 
              (f"%{keyword}%", f"%{keyword}%"))
    data = c.fetchall()
    conn.close()
    return data
