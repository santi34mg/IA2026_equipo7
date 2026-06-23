"use strict";

// Supabase project config — hardcoded on purpose (private uni project).
// The anon key is browser-public by design; the browser uses it to read
// sensor_readings directly via supabase-js. No Vercel env vars needed.
const SUPABASE_URL = "https://qvxxffjrdnjbnblbpcrp.supabase.co";
const SUPABASE_ANON_KEY =
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InF2eHhmZmpyZG5qYm5ibGJwY3JwIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODE2MzgwMzIsImV4cCI6MjA5NzIxNDAzMn0.jsiW6xVf4F7Gf9s9Tj_kkA8-AHg61VPFpley0Vk0_LU";

// Returns a supabase-js client, or null if the CDN lib failed to load
// (callers fall back to the local WebSocket feed in that case).
window.initSupabase = async function initSupabase() {
  if (!window.supabase || typeof window.supabase.createClient !== "function") {
    console.warn("supabase-js not loaded — falling back to local feed");
    return null;
  }
  return window.supabase.createClient(SUPABASE_URL, SUPABASE_ANON_KEY);
};
