"""
One-time migration: build RAG chunks from CSV/science data,
embed with Gemini, and upload to Supabase (rag_chunks table).

Usage:
  pip install supabase
  SUPABASE_URL=... SUPABASE_SERVICE_ROLE_KEY=... GEMINI_API_KEY=... python migrate_rag.py
"""
import os, sys, time, logging
from pathlib import Path

logging.basicConfig(format="%(asctime)s %(levelname)s %(message)s", level=logging.INFO)
log = logging.getLogger(__name__)

# -- Resolve paths -----------------------------------------------------------
CHATBOT_DIR = Path(__file__).parent
sys.path.insert(0, str(CHATBOT_DIR))

from rag import _load_dataframe, _build_csv_chunks, _build_science_chunks, _load_model_metrics_chunk
from google import genai
from google.genai import types as genai_types

# -- Build chunks -------------------------------------------------------------
def build_all_chunks() -> list[str]:
    chunks: list[str] = []
    try:
        df = _load_dataframe()
        csv_chunks = _build_csv_chunks(df)
        chunks.extend(csv_chunks)
        log.info("CSV chunks: %d", len(csv_chunks))
    except Exception as e:
        log.warning("Could not load CSV data: %s", e)

    sci = _build_science_chunks()
    chunks.extend(sci)
    log.info("Science chunks: %d", len(sci))

    metrics = _load_model_metrics_chunk()
    if metrics:
        chunks.append(metrics)
        log.info("Metrics chunk added")

    return chunks

# -- Main --------------------------------------------------------------------
def main():
    supabase_url = os.environ["SUPABASE_URL"]
    service_key = os.environ["SUPABASE_SERVICE_ROLE_KEY"]

    from supabase import create_client
    client = create_client(supabase_url, service_key)

    genai_client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])

    def embed_document(text: str) -> list[float]:
        result = genai_client.models.embed_content(
            model="models/gemini-embedding-001",
            contents=text,
            config=genai_types.EmbedContentConfig(task_type="RETRIEVAL_DOCUMENT", output_dimensionality=1536),
        )
        return result.embeddings[0].values

    log.info("Building RAG chunks...")
    chunks = build_all_chunks()
    log.info("Total chunks: %d", len(chunks))

    log.info("Clearing existing rag_chunks table...")
    client.table("rag_chunks").delete().neq("id", 0).execute()

    log.info("Embedding and uploading chunks...")
    batch = []
    for i, chunk in enumerate(chunks):
        try:
            emb = embed_document(chunk)
            batch.append({"content": chunk, "embedding": emb})
        except Exception as e:
            log.warning("Skipping chunk %d due to embed error: %s", i, e)
            time.sleep(1)
            continue

        if len(batch) >= 10:
            client.table("rag_chunks").insert(batch).execute()
            log.info("Uploaded batch up to chunk %d", i + 1)
            batch = []
            time.sleep(0.5)  # avoid rate limits

    if batch:
        client.table("rag_chunks").insert(batch).execute()
        log.info("Uploaded final batch (%d chunks)", len(batch))

    log.info("Done! %d chunks uploaded to Supabase.", len(chunks))


if __name__ == "__main__":
    main()
