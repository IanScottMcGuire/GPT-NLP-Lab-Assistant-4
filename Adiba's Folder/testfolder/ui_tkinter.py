import tkinter as tk
import requests
import os

# === API Setup ===
API_ENDPOINT = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT", "https://chat-api.tamu.ai")
API_KEY = os.getenv("TAMUS_AI_CHAT_API_KEY")
MODEL = "protected.llama3.2"

def run_tamu_api(user_input: str, model="protected.llama3.2") -> str:
    """Send a message to TAMU AI API and return the assistant's reply."""
    messages = [
        {"role": "system", "content": "You are a helpful assistant for ECEN labs."},
        {"role": "user", "content": user_input},
    ]
    body = {"model": model, "stream": False, "messages": messages}
    headers = {"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"}
    url = f"{API_ENDPOINT}/api/chat/completions"

    response = requests.post(url, headers=headers, json=body)
    response.raise_for_status()
    data = response.json()
    return data["choices"][0]["message"]["content"]

# === Chat bubble system ===
def add_message(text, sender="ai", typing_effect=False, delay=30):
    """Add chat bubble to window."""
    bubble = tk.Label(
        chat_frame,
        text="",
        wraplength=400,
        justify="left" if sender == "ai" else "right",
        bg="#E1F5FE" if sender == "ai" else "#C8E6C9",
        fg="black",
        padx=10,
        pady=5,
        font=("Arial", 12),
        anchor="w" if sender == "ai" else "e"
    )
    bubble.pack(anchor="w" if sender == "ai" else "e", pady=5, padx=10, fill="x")

    if typing_effect:
        def type_char(i=0):
            if i < len(text):
                bubble.config(text=text[:i+1])
                chat_canvas.update_idletasks()
                chat_canvas.yview_moveto(1.0)
                root.after(delay, type_char, i+1)
            else:
                bubble.config(text=text)
        type_char()
    else:
        bubble.config(text=text)

def send_message(event=None):
    """Triggered when user presses Enter or clicks Send button."""
    user_input = entry.get().strip()
    if not user_input:
        return
    entry.delete(0, tk.END)

    # User bubble
    add_message(user_input, sender="user")

    try:
        answer = run_tamu_api(user_input)
    except Exception as e:
        answer = f"❌ Error: {e}"

    # AI bubble with typing animation
    add_message(answer, sender="ai", typing_effect=True, delay=30)

# === Tkinter Setup ===
root = tk.Tk()
root.title("ECEN 403 Chatbot")
root.geometry("600x600")

# Canvas + frame for scrolling bubbles
chat_canvas = tk.Canvas(root, bg="white")
scrollbar = tk.Scrollbar(root, command=chat_canvas.yview)
chat_canvas.configure(yscrollcommand=scrollbar.set)

scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
chat_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

chat_frame = tk.Frame(chat_canvas, bg="white")
chat_window = chat_canvas.create_window((0, 0), window=chat_frame, anchor="nw")

def on_frame_configure(event):
    chat_canvas.configure(scrollregion=chat_canvas.bbox("all"))
chat_frame.bind("<Configure>", on_frame_configure)

# Entry bar
entry_frame = tk.Frame(root, bg="#f1f1f1")
entry_frame.pack(fill=tk.X, side=tk.BOTTOM)

entry = tk.Entry(entry_frame, font=("Arial", 12))
entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(10, 5), pady=10)

send_button = tk.Button(entry_frame, text="Send", command=send_message)
send_button.pack(side=tk.RIGHT, padx=(0, 10), pady=10)

entry.bind("<Return>", send_message)
entry.focus()

# === Initial Greeting ===
add_message("Hello! How can I help you today?", sender="ai", typing_effect=True, delay=40)

root.mainloop()


