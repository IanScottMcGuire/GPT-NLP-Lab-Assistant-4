# build_index.py
import os
import pickle
import faiss
from sentence_transformers import SentenceTransformer
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import PyPDFLoader, TextLoader

DOCS_PATH = "documents"
INDEX_PATH = "index/vector_index.pkl"
MODEL_NAME = "all-MiniLM-L6-v2"

def load_documents():
    docs = []
    for filename in os.listdir(DOCS_PATH):
        path = os.path.join(DOCS_PATH, filename)
        if filename.endswith(".pdf"):
            loader = PyPDFLoader(path)
            docs.extend(loader.load())
        elif filename.endswith(".txt"):
            loader = TextLoader(path)
            docs.extend(loader.load())
    return docs

def build_index():
    print("Loading documents")
    docs = load_documents()
    text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=150)
    splits = text_splitter.split_documents(docs)

    print("Generating embeddings")
    model = SentenceTransformer(MODEL_NAME)
    texts = [chunk.page_content for chunk in splits]
    embeddings = model.encode(texts)

    print("Saving FAISS index")
    index = faiss.IndexFlatL2(embeddings.shape[1])
    index.add(embeddings)

    os.makedirs("index", exist_ok=True)
    with open(INDEX_PATH, "wb") as f:
        pickle.dump((index, texts), f)

    print(f"Index built and saved to {INDEX_PATH}")

if __name__ == "__main__":
    build_index()
