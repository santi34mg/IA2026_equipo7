# Olivia / Flora — Telegram plant bot (Tarea 3)

Telegram bot for the "Flora" smart pot (Epipremnum aureum / Pothos). RAG + Gemini.
The bot runs entirely on **Supabase Edge Functions** (`supabase/functions/`). The only Python
left in `chatbot/` is the RAG-ingestion tooling (`migrate_rag.py` + `rag.py`); the old Python
bot (`bot.py`, `gemini_client.py`, `personality.py`, `scheduler.py`) has been removed.

## Production runs on Supabase

The live bot is **Supabase Edge Functions** (Deno/TS) on project `qvxxffjrdnjbnblbpcrp`,
using a **Telegram webhook**.

- `supabase/functions/telegram-webhook/index.ts` — message/command handler (deployed `--no-verify-jwt`,
  authenticates Telegram via the `x-telegram-bot-api-secret-token` header vs the `WEBHOOK_SECRET` secret).
- `supabase/functions/scheduler/index.ts` — proactive sensor report, fired by pg_cron every 30 min.
- `supabase/functions/_shared/` — `gemini.ts`, `rag.ts`, `decision-tree.ts`, `personality.ts`
  (the `personality.ts` here holds the bot's system prompt — edit it to change Olivia's persona).
- Webhook URL: `https://qvxxffjrdnjbnblbpcrp.supabase.co/functions/v1/telegram-webhook`

## Chat model

Use **`gemini-2.5-flash`** (configurable via env `GEMINI_MODEL` in `gemini.ts`).

- `gemini-2.0-flash` hit free-tier `limit: 0` → `429 ResourceExhausted` (~2026-06-22). Do not use it.
- `gemini-2.5-flash` and `gemini-2.5-flash-lite` have free-tier quota.
- `gemini-2.0-flash-lite` is also exhausted; `gemini-1.5-flash` is no longer served (404).
- A `429` swallowed by the handler shows the user the generic *"Ups, tuve un problema para pensar…"* —
  so a quota problem looks like a crash.

## Embeddings

- **Supabase (prod):** 1536 dims (`gemini.ts` + `migrate_rag.py`), stored in `rag_chunks` (RPC `match_rag_chunks`).
- Uses `models/gemini-embedding-001` (text-embedding-004/001 no longer exist on this key).

## Data tables (Supabase)
`rag_chunks`, `sensor_readings`, `chat_registrations`, `conversation_history`.
RLS is **disabled** on all four (anon key can read/write everything — known risk).
`sensor_readings` is empty until the ESP32/a bridge POSTs readings, so `/estado` answers
"no puedo leer mis sensores" until then.

## Redeploy runbook (after a code change or a broken webhook)

```bash
cd <repo-root>
REF=qvxxffjrdnjbnblbpcrp

# 1. Deploy the function (must be --no-verify-jwt; it does its own secret-header auth)
supabase functions deploy telegram-webhook --no-verify-jwt --project-ref $REF

# 2. If the webhook was deleted (e.g. polling was run against the token): re-register it.
#    We only have the WEBHOOK_SECRET *digest*, so rotate it on both sides to guarantee a match.
SECRET=$(head -c 32 /dev/urandom | base64 | tr -dc 'A-Za-z0-9_-' | head -c 40)
supabase secrets set WEBHOOK_SECRET="$SECRET" --project-ref $REF

TOKEN=$(grep '^TELEGRAM_BOT_TOKEN=' chatbot/.env | cut -d= -f2-)
curl -s "https://api.telegram.org/bot${TOKEN}/setWebhook" \
  --data-urlencode "url=https://${REF}.supabase.co/functions/v1/telegram-webhook" \
  --data-urlencode "secret_token=${SECRET}" \
  --data-urlencode 'allowed_updates=["message"]' \
  --data-urlencode "drop_pending_updates=true"

# 3. Verify
curl -s "https://api.telegram.org/bot${TOKEN}/getWebhookInfo"   # expect url set, last_error null
curl -s -o /dev/null -w "%{http_code}\n" -X POST \
  "https://${REF}.supabase.co/functions/v1/telegram-webhook" \
  -H "Content-Type: application/json" -d '{}'                   # expect 401 (auth works)
```

Secrets already configured on the project: `GEMINI_API_KEY`, `TELEGRAM_BOT_TOKEN`, `WEBHOOK_SECRET`,
plus the auto-injected `SUPABASE_*`. `GEMINI_MODEL` is unset → code defaults to `gemini-2.5-flash`.

## Refresh RAG chunks (Supabase)
```bash
SUPABASE_URL=.. SUPABASE_SERVICE_ROLE_KEY=.. GEMINI_API_KEY=.. python chatbot/migrate_rag.py
```
