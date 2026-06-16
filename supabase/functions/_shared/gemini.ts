const GEMINI_API_KEY = Deno.env.get("GEMINI_API_KEY")!;
const BASE_URL = "https://generativelanguage.googleapis.com/v1beta";

export async function geminiChat(
  systemPrompt: string,
  history: { role: string; content: string }[],
  userMessage: string
): Promise<string> {
  const url = `${BASE_URL}/models/gemini-2.0-flash:generateContent?key=${GEMINI_API_KEY}`;
  const body = {
    systemInstruction: { parts: [{ text: systemPrompt }] },
    contents: [
      ...history.map((h) => ({
        role: h.role,
        parts: [{ text: h.content }],
      })),
      { role: "user", parts: [{ text: userMessage }] },
    ],
  };
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`Gemini error: ${res.status} ${await res.text()}`);
  const data = await res.json();
  return data.candidates[0].content.parts[0].text as string;
}

export async function embedQuery(text: string): Promise<number[]> {
  const url = `${BASE_URL}/models/gemini-embedding-001:embedContent?key=${GEMINI_API_KEY}`;
  const body = {
    model: "models/gemini-embedding-001",
    content: { parts: [{ text }] },
    taskType: "RETRIEVAL_QUERY",
    outputDimensionality: 1536,
  };
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`Gemini embed error: ${res.status} ${await res.text()}`);
  const data = await res.json();
  return data.embedding.values as number[];
}
