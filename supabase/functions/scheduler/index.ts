import { createClient } from "jsr:@supabase/supabase-js@2";
import { predictState } from "../_shared/decision-tree.ts";
import { formatSensorMessage, WATERING_ALERT } from "../_shared/personality.ts";

function supabase() {
  return createClient(
    Deno.env.get("SUPABASE_URL")!,
    Deno.env.get("SUPABASE_SERVICE_ROLE_KEY")!
  );
}

async function sendMessage(botToken: string, chatId: number, text: string) {
  await fetch(`https://api.telegram.org/bot${botToken}/sendMessage`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ chat_id: chatId, text, parse_mode: "Markdown" }),
  });
}

Deno.serve(async () => {
  const botToken = Deno.env.get("TELEGRAM_BOT_TOKEN");
  if (!botToken) return new Response("Missing TELEGRAM_BOT_TOKEN", { status: 500 });

  const db = supabase();

  const [{ data: readingRow }, { data: chats }] = await Promise.all([
    db
      .from("sensor_readings")
      .select("temperatura, luz, humedad_suelo")
      .order("created_at", { ascending: false })
      .limit(1)
      .single(),
    db.from("chat_registrations").select("chat_id"),
  ]);

  if (!readingRow || !chats || chats.length === 0) {
    return new Response("No data or no subscribers", { status: 200 });
  }

  const estado = predictState(readingRow.temperatura, readingRow.luz, readingRow.humedad_suelo);
  const reading = { ...readingRow, estado };
  const message = formatSensorMessage(reading);

  await Promise.all(
    chats.map(async ({ chat_id }: { chat_id: number }) => {
      await sendMessage(botToken, chat_id, message);
      if (estado === "Decaida") {
        await sendMessage(botToken, chat_id, WATERING_ALERT);
      }
    })
  );

  return new Response(`Sent to ${chats.length} subscribers`);
});
