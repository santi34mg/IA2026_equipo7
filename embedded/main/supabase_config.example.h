#pragma once

// ─── Supabase telemetry config (TEMPLATE) ────────────────────────────────────
// Copy this file to `supabase_config.h` and fill in your project's values:
//
//     cp supabase_config.example.h supabase_config.h
//
// `supabase_config.h` is git-ignored so the key never gets committed.
//
// NOTE: the anon key embedded here ends up in the firmware binary. With RLS
// disabled on the project, anyone with the key can read/write the table. This
// is the deliberately-simple setup chosen for this project — do not reuse the
// key for anything sensitive.

#define SUPABASE_URL      "https://YOUR_PROJECT_REF.supabase.co"
#define SUPABASE_ANON_KEY "YOUR_SUPABASE_ANON_KEY"
