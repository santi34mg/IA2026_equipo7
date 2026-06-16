import { createClient } from "jsr:@supabase/supabase-js@2";
import { embedQuery } from "./gemini.ts";

export async function retrieveRAG(query: string, k = 4): Promise<string[]> {
  const supabase = createClient(
    Deno.env.get("SUPABASE_URL")!,
    Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!
  );
  const embedding = await embedQuery(query);
  const { data, error } = await supabase.rpc("match_rag_chunks", {
    query_embedding: embedding,
    match_count: k,
  });
  if (error) throw new Error(`RAG error: ${error.message}`);
  return (data as { content: string }[]).map((r) => r.content);
}
