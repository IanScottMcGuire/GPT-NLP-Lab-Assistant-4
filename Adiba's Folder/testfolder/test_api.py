import os
import requests
import json

# Set these in your environment (or replace here directly for testing)
api_key = os.getenv("TAMUS_AI_CHAT_API_KEY")  # e.g., "<your API key>"
api_endpoint = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT", "https://chat-api.tamu.ai")

# 1. List available models
models_url = f"{api_endpoint}/api/models"
headers = {"Authorization": f"Bearer {api_key}"}

models_response = requests.get(models_url, headers=headers)
print("Available models:")
print(json.dumps(models_response.json(), indent=2))

# 2. Test a chat completion
chat_url = f"{api_endpoint}/api/chat/completions"
chat_body = {
    "model": "protected.llama3.2",
    "stream": False,
    "messages": [
        {"role": "user", "content": "Why is the sky blue?"}
    ]
}

chat_response = requests.post(chat_url, headers={**headers, "Content-Type": "application/json"}, json=chat_body)
print("\nChat response:")
print(json.dumps(chat_response.json(), indent=2))

