import os
import faiss
import numpy as np
import requests
from sentence_transformers import SentenceTransformer
from dotenv import load_dotenv

# Load env variables
load_dotenv()
API_KEY = os.getenv("TAMUS_AI_CHAT_API_KEY")
API_ENDPOINT = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT", "https://chat-api.tamu.ai")
MODEL = "protected.llama3.2"

# Load FAISS index, chunks and sources
index = faiss.read_index("ecen214.index")
chunks = np.load("ecen214_chunks.npy", allow_pickle=True)
sources = np.load("ecen214_sources.npy", allow_pickle=True)

# Embedding model
model = SentenceTransformer("all-MiniLM-L6-v2")

def retrieve_context(question, k=3):
    """Retrieve top-k context passages from FAISS index."""
    q_emb = model.encode([question])
    D, I = index.search(np.array(q_emb), k)
    return [(chunks[i], sources[i]) for i in I[0]]

def ask_llm(question, context_chunks):
    """Send the user question + retrieved context to the TAMU LLM API."""
    context_text = "\n".join([f"From {src}: {txt}" for txt, src in context_chunks])

    messages = [
        {"role": "system", "content": "You are a helpful assistant for ECEN 214 labs."},
        {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {question}"}
    ]

    body = {
        "model": MODEL,
        "stream": False,
        "messages": messages
    }

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }

    response = requests.post(f"{API_ENDPOINT}/api/chat/completions",
                             headers=headers, json=body)

    return response.json()

def ask_question(prompt):
    """Main function: retrieve context and return LLM's answer (string)."""
    context = retrieve_context(prompt, k=3)
    answer_json = ask_llm(prompt, context)
    return answer_json.get("choices", [{}])[0].get("message", {}).get("content", "No answer")

# Optional: still allow CLI usage
if __name__ == "__main__":
    print("Enter your question: (type 'exit' to quit)\n")
    while True:
        q = input("Howdy! How can I assist you today? ")
        if q.lower() == "exit":
            break
        print("LLM Answer:")
        print(ask_question(q))
        print("\n")
