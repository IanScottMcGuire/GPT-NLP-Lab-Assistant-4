# rag_chat.py
import os
import pickle
import faiss
import requests
from sentence_transformers import SentenceTransformer

# Load env vars like in ask_questions.py
API_KEY = os.getenv("TAMUS_AI_CHAT_API_KEY")
API_URL = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT")

INDEX_PATH = "index/vector_index.pkl"
MODEL_NAME = "all-MiniLM-L6-v2"

def load_index():
    with open(INDEX_PATH, "rb") as f:
        index, texts = pickle.load(f)
    return index, texts

def retrieve_context(query, index, texts, embed_model, k=3):
    query_vec = embed_model.encode([query])
    distances, indices = index.search(query_vec, k)
    return "\n".join([texts[i] for i in indices[0]])

def query_tamuai(prompt):
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": "protected.llama3.2",
        "temperature": 0.0,
        "top_p": 1.0,
        "stream": False,
        "messages": [{"role": "user", "content": prompt}],
}


    response = requests.post(f"{API_URL}/api/chat/completions", headers=headers, json=payload)
    response.raise_for_status()
    return response.json()["choices"][0]["message"]["content"]

def rag_chat():
    print("Howdy! How can I assist you today? Type 'exit' or 'quit' to leave.\n")

    embed_model = SentenceTransformer(MODEL_NAME)
    index, texts = load_index()

    while True:
        user_query = input("You: ").strip()
        if user_query.lower() in ["exit", "quit"]:
            print("👋 Exiting chat.")
            break

        context = retrieve_context(user_query, index, texts, embed_model)
        full_prompt = f""" You are a Retrieval QA assistant. ONLY answer using facts found in the context below. If the answer is not in the context, say: "I could not find this information in the provided documents."

        Context:
        {context}

        Question: {user_query}

        Answer:
        """


        try:
            answer = query_tamuai(full_prompt)
            print(f"\nAssistant: {answer}\n")
            print("\n--- Retrieved Context ---\n")
            print(context)

        except Exception as e:
            print(f"⚠️ Error: {e}\n")

if __name__ == "__main__":
    rag_chat()
