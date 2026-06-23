---
name: olivia-bot-deploy
description: Deploy or repair the Olivia/Flora Telegram bot on Supabase Edge Functions. Use when the bot stops responding to Telegram messages, after editing supabase/functions/*, when the Telegram webhook is missing/broken, or when changing the Gemini chat model. Covers redeploy, webhook re-registration, and the gemini-2.0-flash quota gotcha.
---

# Deploy / repair the Olivia (Flora) Telegram bot

The production bot is **Supabase Edge Functions** (Deno/TS), project ref `qvxxffjrdnjbnblbpcrp`,
driven by a **Telegram webhook**. The Python code in `chatbot/` is local-dev only. Full context:
`chatbot/CLAUDE.md`.

## Diagnose first

When "the bot doesn't respond to messages", check these in order — they are the actual past causes:

1. **Webhook deleted.** Anything that runs Telegram long-polling against this bot token calls
   `deleteWebhook` on startup and wipes the Supabase webhook. Check with `getWebhookInfo` —
   if `url` is empty, this is it.
   ```bash
   TOKEN=$(grep '^TELEGRAM_BOT_TOKEN=' chatbot/.env | cut -d= -f2-)
   curl -s "https://api.telegram.org/bot${TOKEN}/getWebhookInfo"   # look at .result.url and .last_error_message
   ```
2. **Gemini model out of quota.** Commands (`/start`, `/ayuda`, `/estado`) still work because they don't
   call Gemini — only free-text replies fail, surfacing as *"Ups, tuve un problema para pensar…"*.
   `gemini-2.0-flash` is exhausted (free-tier `limit: 0`, `429 ResourceExhausted`). Use **`gemini-2.5-flash`**.
   The model lives in `supabase/functions/_shared/gemini.ts` (`CHAT_MODEL`, env `GEMINI_MODEL`).
3. **Function error.** Check logs in the Supabase dashboard Functions tab, or `supabase functions list`.

Confirm a model has quota before committing to it:
```bash
.venv/bin/python -c "import os,google.generativeai as g; g.configure(api_key=os.environ['GEMINI_API_KEY']); \
print(g.GenerativeModel('gemini-2.5-flash').start_chat(history=[]).send_message('hola').text[:40])"
```
(Run from `chatbot/` with its `.venv`.) Working: 2.5-flash, 2.5-flash-lite. Dead: 2.0-flash(-lite), 1.5-flash(404).

## Redeploy

```bash
cd <repo-root>
REF=qvxxffjrdnjbnblbpcrp
supabase functions deploy telegram-webhook --no-verify-jwt --project-ref $REF
```
`--no-verify-jwt` is required: the function does its own auth via the `x-telegram-bot-api-secret-token`
header checked against the `WEBHOOK_SECRET` secret.

## Re-register the webhook (if it was deleted)

We only have the `WEBHOOK_SECRET` **digest**, not the plaintext, so rotate it on both sides:

```bash
REF=qvxxffjrdnjbnblbpcrp
SECRET=$(head -c 32 /dev/urandom | base64 | tr -dc 'A-Za-z0-9_-' | head -c 40)
supabase secrets set WEBHOOK_SECRET="$SECRET" --project-ref $REF

TOKEN=$(grep '^TELEGRAM_BOT_TOKEN=' chatbot/.env | cut -d= -f2-)
curl -s "https://api.telegram.org/bot${TOKEN}/setWebhook" \
  --data-urlencode "url=https://${REF}.supabase.co/functions/v1/telegram-webhook" \
  --data-urlencode "secret_token=${SECRET}" \
  --data-urlencode 'allowed_updates=["message"]' \
  --data-urlencode "drop_pending_updates=true"
```
Never print the bot token or secret to the terminal.

## Verify

```bash
TOKEN=$(grep '^TELEGRAM_BOT_TOKEN=' chatbot/.env | cut -d= -f2-)
REF=qvxxffjrdnjbnblbpcrp
curl -s "https://api.telegram.org/bot${TOKEN}/getWebhookInfo"          # url set, last_error_message null, pending 0
curl -s -o /dev/null -w "%{http_code}\n" -X POST \
  "https://${REF}.supabase.co/functions/v1/telegram-webhook" -d '{}'   # expect 401 -> secret auth works
```
Final end-to-end confirmation: send the bot a real Telegram message; it should reply via Gemini.

## Don'ts

- **Don't run any Telegram long-polling client** against the production token — it deletes the webhook.
- **Don't deploy `telegram-webhook` without `--no-verify-jwt`** — Telegram can't send a JWT, so updates would 401.
- **Don't reuse `gemini-2.0-flash`** — it's quota-dead.
