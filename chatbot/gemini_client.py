import os
import time

import google.generativeai as genai
from google.api_core import exceptions as gexc

# Modelo configurable por env; default a uno con cuota gratuita disponible.
CHAT_MODEL = os.environ.get("GEMINI_MODEL", "gemini-2.5-flash")
# Reintentos ante 429 (ResourceExhausted) respetando el retry_delay sugerido.
MAX_RETRIES = int(os.environ.get("GEMINI_MAX_RETRIES", "3"))
MAX_RETRY_WAIT = float(os.environ.get("GEMINI_MAX_RETRY_WAIT", "30"))


def _retry_delay_seconds(exc: gexc.ResourceExhausted) -> float | None:
    """Extrae el retry_delay (segundos) que sugiere el error 429, si existe."""
    for detail in getattr(exc, "details", None) or []:
        retry_delay = getattr(detail, "retry_delay", None)
        if retry_delay is not None and getattr(retry_delay, "seconds", 0):
            return float(retry_delay.seconds)
    return None


class GeminiClient:
    def __init__(self) -> None:
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])

    def embed(self, text: str, task_type: str = "retrieval_query") -> list[float]:
        result = genai.embed_content(
            model="models/gemini-embedding-001",
            content=text,
            task_type=task_type,
            output_dimensionality=768,
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
            model_name=CHAT_MODEL,
            system_instruction=system_prompt,
        )
        gemini_history = [
            {"role": m["role"], "parts": [m["content"]]}
            for m in history
        ]
        chat_session = model.start_chat(history=gemini_history)

        last_exc = None
        for attempt in range(MAX_RETRIES):
            try:
                response = chat_session.send_message(user_message)
                return response.text
            except gexc.ResourceExhausted as e:
                last_exc = e
                wait = _retry_delay_seconds(e)
                if wait is None or wait > MAX_RETRY_WAIT or attempt == MAX_RETRIES - 1:
                    break
                time.sleep(wait)

        raise last_exc
