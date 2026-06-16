"""Bot de Telegram para Olivia — Maceta Inteligente con RAG + Gemini."""
import logging
import os
from datetime import datetime
from pathlib import Path

from dotenv import load_dotenv
from telegram import Update
from telegram.ext import (
    Application,
    CommandHandler,
    ContextTypes,
    MessageHandler,
    filters,
)

load_dotenv()

logging.basicConfig(
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)

from gemini_client import GeminiClient
from personality import PLANT_NAME, SYSTEM_PROMPT
from rag import RAGIndex
from scheduler import format_sensor_message, get_latest_reading, register_jobs

MAX_HISTORY = 10

_gemini: GeminiClient = None
_rag: RAGIndex = None

OFF_TOPIC_KEYWORDS = [
    # palabras que claramente NO son de plantas
    "política", "fútbol", "deporte", "presidente", "elecciones",
    "matemáticas", "historia", "geografía", "código", "programar",
    "película", "música", "receta", "cocina", "economía", "bolsa",
    "bitcoin", "criptomoneda", "guerra", "clima global",
]


def _is_off_topic(text: str) -> bool:
    lower = text.lower()
    return any(kw in lower for kw in OFF_TOPIC_KEYWORDS)


def _get_history(context: ContextTypes.DEFAULT_TYPE) -> list[dict]:
    return context.user_data.get("history", [])


def _add_to_history(context: ContextTypes.DEFAULT_TYPE, role: str, content: str) -> None:
    history = context.user_data.setdefault("history", [])
    history.append({"role": role, "content": content})
    if len(history) > MAX_HISTORY * 2:
        context.user_data["history"] = history[-(MAX_HISTORY * 2):]


def _build_rag_context(query: str, reading: dict | None) -> str:
    chunks = _rag.retrieve(query, k=4)

    parts = []

    if reading:
        parts.append(
            f"=== ESTADO ACTUAL DE FLORA ===\n"
            f"Temperatura: {reading['temperatura']:.1f}°C\n"
            f"Luz: {reading['luz']} (raw)\n"
            f"Humedad del suelo: {reading['humedad_suelo']} (raw)\n"
            f"Estado predicho por el modelo: {reading['estado']}\n"
            f"Fecha: {datetime.now().strftime('%d/%m/%Y %H:%M')}"
        )

    if chunks:
        parts.append("=== CONOCIMIENTO RECUPERADO (RAG) ===\n" + "\n\n---\n\n".join(chunks))

    return "\n\n".join(parts)


async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    chat_id = update.effective_chat.id
    chat_ids: set = context.bot_data.setdefault("chat_ids", set())
    chat_ids.add(chat_id)

    await update.message.reply_text(
        f"¡Hola! Soy *{PLANT_NAME}* 🌿, tu maceta inteligente favorita.\n\n"
        f"Soy un *Epipremnum aureum* (Pothos) y estoy equipada con sensores "
        f"de temperatura, luz y humedad del suelo. ¡Puedo contarte cómo me siento "
        f"y qué necesito!\n\n"
        f"Te enviaré mis lecturas de sensores cada 30 minutos y te avisaré "
        f"si necesito riego urgente. 💧\n\n"
        f"*Comandos disponibles:*\n"
        f"/estado — Ver mi estado actual\n"
        f"/ayuda — Mostrar esta ayuda\n\n"
        f"O simplemente pregúntame algo sobre cómo estoy o qué necesito 🌱",
        parse_mode="Markdown",
    )


async def cmd_ayuda(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    await update.message.reply_text(
        f"🌿 *{PLANT_NAME} — Comandos disponibles* 🌿\n\n"
        f"/start — Iniciar el bot y registrarte para recibir mis reportes\n"
        f"/estado — Ver mis lecturas de sensores en este momento\n"
        f"/ayuda — Mostrar esta ayuda\n\n"
        f"*También puedes preguntarme cosas como:*\n"
        f"• ¿Cómo estás, Olivia?\n"
        f"• ¿Necesitás riego hoy?\n"
        f"• ¿Qué temperatura es ideal para vos?\n"
        f"• ¿Qué cuidados necesitás esta semana?\n"
        f"• ¿Por qué tus hojas están amarillas?\n\n"
        f"_Recuerdo: solo puedo hablar sobre temas relacionados con plantas y mi cuidado_ 🍃",
        parse_mode="Markdown",
    )


async def cmd_estado(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    reading = get_latest_reading()
    if reading is None:
        await update.message.reply_text(
            "Hmm, no pude leer mis sensores ahora mismo 😕 "
            "¿Estás seguro de que estoy conectada?"
        )
        return

    msg = format_sensor_message(reading)
    await update.message.reply_text(msg, parse_mode="Markdown")

    if reading["estado"] == "Decaida":
        await update.message.reply_text(
            "Ay, me siento muy mal... 😔 Mis sensores dicen que necesito atención. "
            "¡Por favor ayúdame! 💧🌿"
        )
    elif reading["estado"] == "Ideal":
        await update.message.reply_text(
            "¡Estoy radiante hoy! 🌟 Mis condiciones son perfectas. ¡Gracias por cuidarme tan bien! 🌿"
        )
    else:
        await update.message.reply_text(
            "Estoy bien, aunque podría estar mejor 🌱 "
            "Si mejoran un poco mis condiciones estaré perfecta."
        )


async def handle_message(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    user_text = update.message.text.strip()

    if _is_off_topic(user_text):
        await update.message.reply_text(
            f"¡Lo siento! Solo sé hablar sobre plantas y sobre mí misma 🌿 "
            f"¿Te puedo ayudar con algo relacionado a mis cuidados o estado?"
        )
        return

    reading = get_latest_reading()
    rag_context = _build_rag_context(user_text, reading)

    history = _get_history(context)

    system_with_context = (
        SYSTEM_PROMPT
        + "\n\n"
        + "Usa la siguiente información de contexto para responder:\n\n"
        + rag_context
    )

    try:
        await context.bot.send_chat_action(
            chat_id=update.effective_chat.id, action="typing"
        )
        response = _gemini.chat(
            system_prompt=system_with_context,
            history=history,
            user_message=user_text,
        )
    except Exception as e:
        logger.error("Error llamando a Gemini: %s", e)
        await update.message.reply_text(
            "Ups, tuve un problema para pensar ahora mismo 😅 "
            "¿Podés intentarlo de nuevo en un momento?"
        )
        return

    _add_to_history(context, "user", user_text)
    _add_to_history(context, "model", response)

    await update.message.reply_text(response)


def main() -> None:
    token = os.environ.get("TELEGRAM_BOT_TOKEN")
    if not token:
        raise ValueError("TELEGRAM_BOT_TOKEN no está configurado en .env")

    gemini_key = os.environ.get("GEMINI_API_KEY")
    if not gemini_key:
        raise ValueError("GEMINI_API_KEY no está configurado en .env")

    global _gemini, _rag
    logger.info("Iniciando Olivia Bot...")

    _gemini = GeminiClient()

    _rag = RAGIndex(_gemini)
    logger.info("Construyendo índice RAG (puede tardar ~30s la primera vez)...")
    _rag.build()

    initial_chat_id = os.environ.get("TELEGRAM_CHAT_ID")

    app = Application.builder().token(token).build()

    if initial_chat_id:
        app.bot_data["chat_ids"] = {int(initial_chat_id)}

    app.add_handler(CommandHandler("start", cmd_start))
    app.add_handler(CommandHandler("ayuda", cmd_ayuda))
    app.add_handler(CommandHandler("estado", cmd_estado))
    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, handle_message))

    register_jobs(app)

    logger.info("Olivia Bot iniciado. Esperando mensajes... 🌿")
    app.run_polling(allowed_updates=Update.ALL_TYPES)


if __name__ == "__main__":
    main()
