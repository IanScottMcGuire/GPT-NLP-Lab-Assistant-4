# tamu_api.py
import os
import requests

API_ENDPOINT = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT", "https://chat-api.tamu.ai")
API_KEY = os.getenv("TAMUS_AI_CHAT_API_KEY")

def ask_tamu_api(user_input: str, model="protected.llama3.2") -> str:
    """Send a message to TAMU AI API and return the assistant's reply."""
    messages = [
        {"role": "system", "content": "You are a helpful assistant for ECEN labs."},
        {"role": "user", "content": user_input},
    ]

    body = {
        "model": model,
        "stream": False,
        "messages": messages,
    }

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json",
    }

    #FIX: TAMU endpoint uses `/api/chat/completions`, not `/v1/...`
    url = f"{API_ENDPOINT}/api/chat/completions"

    response = requests.post(url, headers=headers, json=body)
    response.raise_for_status()
    data = response.json()

    return data["choices"][0]["message"]["content"]
