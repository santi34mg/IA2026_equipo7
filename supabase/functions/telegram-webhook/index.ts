import { createClient } from "jsr:@supabase/supabase-js@2";
import { geminiChat } from "../_shared/gemini.ts";
import { retrieveRAG } from "../_shared/rag.ts";
import { predictState } from "../_shared/decision-tree.ts";
import {
  PLANT_NAME,
  SYSTEM_PROMPT,
  formatSensorMessage,
  isOffTopic,
} from "../_shared/personality.ts";

const MAX_HISTORY = 10;

function supabase() {
  return createClient(
    Deno.env.get("SUPABASE_URL")!,
    Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!
  );
}

async function sendMessage(
  botToken: string,
  chatId: number,
  text: string,
  parseMode = "Markdown"
) {
  await fetch(`https://api.telegram.org/bot${botToken}/sendMessage`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ chat_id: chatId, text, parse_mode: parseMode }),
  });
}

async function getHistory(chatId: number) {
  const { data } = await supabase()
    .from("conversation_history")
    .select("role, content")
    .eq("chat_id", chatId)
    .order("created_at", { ascending: false })
    .limit(MAX_HISTORY * 2);
  return (data ?? []).reverse() as { role: string; content: string }[];
}

async function appendHistory(chatId: number, role: string, content: string) {
  await supabase().from("conversation_history").insert({ chat_id: chatId, role, content });
  // Prune old messages beyond MAX_HISTORY * 2
  const { data: rows } = await supabase()
    .from("conversation_history")
    .select("id")
    .eq("chat_id", chatId)
    .order("created_at", { ascending: true });
  if (rows && rows.length > MAX_HISTORY * 2) {
    const toDelete = rows.slice(0, rows.length - MAX_HISTORY * 2).map((r: { id: number }) => r.id);
    await supabase().from("conversation_history").delete().in("id", toDelete);
  }
}

async function getLatestReading() {
  const { data } = await supabase()
    .from("sensor_readings")
    .select("temperatura, luz, humedad_suelo, label")
    .order("created_at", { ascending: false })
    .limit(1)
    .single();
  if (!data) return null;
  const estado = predictState(data.temperatura, data.luz, data.humedad_suelo);
  return { ...data, estado };
}

async function handleStart(botToken: string, chatId: number) {
  await supabase()
    .from("chat_registrations")
    .upsert({ chat_id: chatId }, { onConflict: "chat_id" });
  await sendMessage(
    botToken,
    chatId,
    `¡Hola! Soy *${PLANT_NAME}* 🌿, tu maceta inteligente favorita.\n\n` +
      `Soy un *Epipremnum aureum (Pothos)* y estoy equipada con sensores ` +
      `de temperatura, luz y humedad del suelo. ¡Puedo contarte cómo me siento ` +
      `y qué necesito!\n\n` +
      `Te enviaré mis lecturas de sensores cada 30 minutos y te avisaré ` +
      `si necesito riego urgente. 💧\n\n` +
      `*Comandos disponibles:*\n` +
      `/estado — Ver mi estado actual\n` +
      `/ayuda — Mostrar esta ayuda\n\n` +
      `O simplemente pregúntame algo sobre cómo estoy o qué necesito 🌱`
  );
}

async function handleAyuda(botToken: string, chatId: number) {
  await sendMessage(
    botToken,
    chatId,
    `🌿 *${PLANT_NAME} — Comandos disponibles* 🌿\n\n` +
      `/start — Iniciar el bot y registrarte para recibir mis reportes\n` +
      `/estado — Ver mis lecturas de sensores en este momento\n` +
      `/ayuda — Mostrar esta ayuda\n\n` +
      `*También puedes preguntarme cosas como:*\n` +
      `• ¿Cómo estás, Olivia?\n` +
      `• ¿Necesitás riego hoy?\n` +
      `• ¿Qué temperatura es ideal para vos?\n` +
      `• ¿Qué cuidados necesitás esta semana?\n` +
      `• ¿Por qué tus hojas están amarillas?\n\n` +
      `_Recuerdo: solo puedo hablar sobre temas relacionados con plantas y mi cuidado_ 🍃`
  );
}

async function handleEstado(botToken: string, chatId: number) {
  const reading = await getLatestReading();
  if (!reading) {
    await sendMessage(
      botToken,
      chatId,
      "Hmm, no pude leer mis sensores ahora mismo 😕 ¿Estás seguro de que estoy conectada?"
    );
    return;
  }
  await sendMessage(botToken, chatId, formatSensorMessage(reading));
  if (reading.estado === "Decaida") {
    await sendMessage(
      botToken,
      chatId,
      "Ay, me siento muy mal... 😔 Mis sensores dicen que necesito atención. ¡Por favor ayúdame! 💧🌿"
    );
  } else if (reading.estado === "Ideal") {
    await sendMessage(
      botToken,
      chatId,
      "¡Estoy radiante hoy! 🌟 Mis condiciones son perfectas. ¡Gracias por cuidarme tan bien! 🌿"
    );
  } else {
    await sendMessage(
      botToken,
      chatId,
      "Estoy bien, aunque podría estar mejor 🌱 Si mejoran un poco mis condiciones estaré perfecta."
    );
  }
}

async function handleMessage(botToken: string, chatId: number, userText: string) {
  if (isOffTopic(userText)) {
    await sendMessage(
      botToken,
      chatId,
      `¡Lo siento! Solo sé hablar sobre plantas y sobre mí misma 🌿 ¿Te puedo ayudar con algo relacionado a mis cuidados o estado?`
    );
    return;
  }

  const [reading, ragChunks, history] = await Promise.all([
    getLatestReading(),
    retrieveRAG(userText),
    getHistory(chatId),
  ]);

  const now = new Date().toLocaleString("es-AR", { timeZone: "America/Argentina/Buenos_Aires" });
  let ragContext = "";
  if (reading) {
    ragContext +=
      `=== ESTADO ACTUAL DE FLORA ===\n` +
      `Temperatura: ${reading.temperatura.toFixed(1)}°C\n` +
      `Luz: ${reading.luz} (raw)\n` +
      `Humedad del suelo: ${reading.humedad_suelo} (raw)\n` +
      `Estado predicho por el modelo: ${reading.estado}\n` +
      `Fecha: ${now}`;
  }
  if (ragChunks.length > 0) {
    ragContext += (ragContext ? "\n\n" : "") +
      "=== CONOCIMIENTO RECUPERADO (RAG) ===\n" +
      ragChunks.join("\n\n---\n\n");
  }

  const systemWithContext =
    SYSTEM_PROMPT +
    "\n\nUsa la siguiente información de contexto para responder:\n\n" +
    ragContext;

  let response: string;
  try {
    response = await geminiChat(systemWithContext, history, userText);
  } catch (e) {
    console.error("Gemini error:", e);
    await sendMessage(
      botToken,
      chatId,
      "Ups, tuve un problema para pensar ahora mismo 😅 ¿Podés intentarlo de nuevo en un momento?"
    );
    return;
  }

  await Promise.all([
    appendHistory(chatId, "user", userText),
    appendHistory(chatId, "model", response),
  ]);

  await sendMessage(botToken, chatId, response);
}

Deno.serve(async (req) => {
  const botToken = Deno.env.get("TELEGRAM_BOT_TOKEN");
  if (!botToken) return new Response("Missing TELEGRAM_BOT_TOKEN", { status: 500 });

  // Authenticate the caller as Telegram via the secret token header.
  // Required because this function is deployed with --no-verify-jwt.
  const webhookSecret = Deno.env.get("WEBHOOK_SECRET");
  if (webhookSecret) {
    const provided = req.headers.get("x-telegram-bot-api-secret-token");
    if (provided !== webhookSecret) {
      return new Response("Unauthorized", { status: 401 });
    }
  }

  let update: { message?: { chat: { id: number }; text?: string } };
  try {
    update = await req.json();
  } catch {
    return new Response("Bad request", { status: 400 });
  }

  const message = update.message;
  if (!message?.text) return new Response("OK");

  const chatId = message.chat.id;
  const text = message.text.trim();

  try {
    if (text === "/start" || text.startsWith("/start ")) {
      await handleStart(botToken, chatId);
    } else if (text === "/ayuda" || text.startsWith("/ayuda ")) {
      await handleAyuda(botToken, chatId);
    } else if (text === "/estado" || text.startsWith("/estado ")) {
      await handleEstado(botToken, chatId);
    } else if (!text.startsWith("/")) {
      await handleMessage(botToken, chatId, text);
    }
  } catch (e) {
    console.error("Handler error:", e);
  }

  return new Response("OK");
});
