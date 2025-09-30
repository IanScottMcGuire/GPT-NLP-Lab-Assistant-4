# ui_basic.py
from tamu_api import ask_tamu_api

print("=== ECEN 403 Chatbot UI ===")
print("Type 'exit' to quit.")

while True:
    q = input("You: ")
    if q.lower() == "exit":
        break

    try:
        answer = ask_tamu_api(q)
        print("AI:", answer, "\n")
    except Exception as e:
        print("❌ Error:", e)
