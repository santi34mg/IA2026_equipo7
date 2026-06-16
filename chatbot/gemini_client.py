import os
import google.generativeai as genai


class GeminiClient:
    def __init__(self) -> None:
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])

    def embed(self, text: str, task_type: str = "retrieval_query") -> list[float]:
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=text,
            task_type=task_type,
        )
        return result["embedding"]

    def embed_document(self, text: str) -> list[float]:
        return self.embed(text, task_type="retrieval_document")

    def chat(
        self,
        system_prompt: str,
        history: list[dict],
        user_message: str,
    ) -> str:
        model = genai.GenerativeModel(
            model_name="gemini-2.0-flash",
            system_instruction=system_prompt,
        )
        gemini_history = [
            {"role": m["role"], "parts": [m["content"]]}
            for m in history
        ]
        chat_session = model.start_chat(history=gemini_history)
        response = chat_session.send_message(user_message)
        return response.text
