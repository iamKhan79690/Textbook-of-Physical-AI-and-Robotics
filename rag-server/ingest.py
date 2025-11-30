import os
import uuid
import glob
import psycopg2
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance

# --- CONFIGURATION ---
load_dotenv()

# Initialize Clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
neon_conn = psycopg2.connect(os.getenv("NEON_DB_URL"))
cursor = neon_conn.cursor()

# Collection Name
COLLECTION_NAME = "textbook"

# --- DATABASE SETUP ---
def setup_databases():
    print("Setting up databases...")
    
    # 1. Qdrant: Create collection if missing
    collections = qdrant.get_collections()
    exists = any(c.name == COLLECTION_NAME for c in collections.collections)
    
    if not exists:
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )
        print(f"Created Qdrant collection: {COLLECTION_NAME}")
    else:
        print(f"Qdrant collection {COLLECTION_NAME} already exists.")

    # 2. Neon: Create table for text storage
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS book_content (
            id SERIAL PRIMARY KEY,
            chunk_id TEXT UNIQUE,
            content TEXT,
            source_file TEXT
        );
    """)
    neon_conn.commit()
    print("Neon table 'book_content' ready.")

# --- HELPER: TEXT SPLITTER ---
def split_text(text, chunk_size=800):
    """Splits text into chunks, respecting paragraphs."""
    chunks = []
    current_chunk = ""
    paragraphs = text.split('\n\n')
    
    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk: chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"
            
    if current_chunk: chunks.append(current_chunk.strip())
    return chunks

# --- MAIN INGESTION ---
def ingest_data():
    # Point this to your Docusaurus docs folder
    docs_path = "../book-source/docs"
    files = glob.glob(os.path.join(docs_path, "**/*.md"), recursive=True)
    
    print(f"Found {len(files)} markdown files. Starting ingestion...")

    count = 0
    for file_path in files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
        
        chunks = split_text(content)
        
        for chunk in chunks:
            if not chunk.strip(): continue
            
            chunk_id = str(uuid.uuid4())
            
            # 1. Generate Embedding (OpenAI)
            embedding = openai_client.embeddings.create(
                input=chunk,
                model="text-embedding-3-small"
            ).data[0].embedding
            
            # 2. Upload Vector (Qdrant)
            qdrant.upsert(
                collection_name=COLLECTION_NAME,
                points=[PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={"chunk_id": chunk_id}
                )]
            )
            
            # 3. Upload Text (Neon)
            cursor.execute(
                "INSERT INTO book_content (chunk_id, content, source_file) VALUES (%s, %s, %s)",
                (chunk_id, chunk, os.path.basename(file_path))
            )
            count += 1
            print(f"Processed chunk {count}...", end="\r")

    neon_conn.commit()
    print(f"\nSuccess! Ingested {count} chunks into Qdrant & Neon.")

if __name__ == "__main__":
    setup_databases()
    ingest_data()