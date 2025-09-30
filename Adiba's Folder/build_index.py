import os #reads all files in ECEN 214
from PyPDF2 import PdfReader #library for reading and extracting text from the files
from sentence_transformers import SentenceTransformer #load pretrained embedding model
import faiss
import numpy as np


model = SentenceTransformer("all-MiniLM-L6-v2") #embedding model


docs_folder = "ECEN 214" #load all PDFs from ECEN 214 folder
all_chunks = [] #chunks of text list
doc_sources = [] #source list (where each chunk came from) 

#loop for all files in ECEN 214 folder
for file_name in os.listdir(docs_folder):
        file_path = os.path.join(docs_folder, file_name) #filepath
        reader = PdfReader(file_path) #open pdf
        text = " ".join([page.extract_text() or "" for page in reader.pages]) #extract text and join into one string

        # Split text into 500 character chunks
        chunks = [text[i:i+500] for i in range(0, len(text), 500)]
        all_chunks.extend(chunks) #add chunks to list
        doc_sources.extend([file_name] * len(chunks))  # track source doc
        print(f"Loaded {len(chunks)} chunks from {file_name}") #track progress


#FAISS index - vector DB
embeddings = model.encode(all_chunks) #convert text chunks into vectors
dim = embeddings.shape[1] #vector dimension (MiniLM = 384)
index = faiss.IndexFlatL2(dim) #initialize FAISS index
index.add(np.array(embeddings)) #add embeddings into FAISS index

print("FAISS Database loaded with", len(all_chunks), "chunks")

# Save to disk
faiss.write_index(index, "ecen214.index") #save FAISS index
np.save("ecen214_chunks.npy", np.array(all_chunks, dtype=object)) #save chunk into list using numpy
np.save("ecen214_sources.npy", np.array(doc_sources, dtype=object)) #save sources into list using numpy

print("Saved FAISS index + chunks + sources")



