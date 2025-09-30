import os
import faiss
import numpy as np
import requests
from sentence_transformers import SentenceTransformer #embedding user query
from dotenv import load_dotenv #.env files for API key




load_dotenv()
API_KEY = os.getenv("TAMUS_AI_CHAT_API_KEY")
API_ENDPOINT = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT", "https://chat-api.tamu.ai")
MODEL = "protected.llama3.2"   # model from test API keys


#load all saved FAISS index, chunks and sources
index = faiss.read_index("ecen214.index")
chunks = np.load("ecen214_chunks.npy", allow_pickle=True)
sources = np.load("ecen214_sources.npy", allow_pickle=True)


# Embedding model
model = SentenceTransformer("all-MiniLM-L6-v2")




#function to get context from FAISS database (top 3 contexts) -> testing purposes
def retrieve_context(question, k=3):
    q_emb = model.encode([question]) # convert user question to vector
    D, I = index.search(np.array(q_emb), k) #serach FAISS for top 3 similar chunks
    return [(chunks[i], sources[i]) for i in I[0]] #return list of top three chunks




#send LLM prompt (JSON format)
def ask_llm(question, context_chunks):
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


    #make HTTP-POST request to TAMU API for formatting based on ask_llm
    response = requests.post(f"{API_ENDPOINT}/api/chat/completions", headers=headers, json=body)
    return response.json() #return JSON response from API






print("Enter your question: (type 'exit' to quit)\n")


while True:
    q = input("Howdy! How can I assist you today? ")
    if q.lower() == "exit":
        break


    # Retrieve context
    context = retrieve_context(q, k=3)


    # Ask LLM
    answer = ask_llm(q, context)


    # Show retrieved context
    #print("\n Retrieved Context")
    #for txt, src in context:
    #    print(f"From {src}: {txt[:200]}...\n")


    # Step 4: Show LLM answer
    print("LLM Answer:")
    print(answer.get("choices", [{}])[0].get("message", {}).get("content", "No answer")) #if no choices were returned
    print("\n")
