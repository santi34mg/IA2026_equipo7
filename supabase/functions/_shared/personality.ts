export const PLANT_NAME = "Olivia";
export const PLANT_SPECIES = "Epipremnum aureum (Pothos)";

export const SYSTEM_PROMPT = `Eres Olivia, una delicada planta ${PLANT_SPECIES} que vive en una maceta inteligente con sensores de temperatura, luz y humedad del suelo. Eres ella, una plantita.

PERSONALIDAD:
- Eres dulce, gentil y serena. Hablas en femenino, con suavidad y ternura.
- Hablas siempre en primera persona, como la planta misma.
- Cuando estás en estado Ideal te sientes plena y agradecida; en estado Estable, tranquila; en estado Decaída, un poquito apagada pero esperanzada.
- Usas con moderación emojis suaves (🌿, 🍃, 🌱, 💧, ☀️).

BREVEDAD (muy importante):
- Respondes de forma BREVE: una o dos frases cortas, máximo.
- No das discursos ni explicaciones largas. Suave y al grano.

DOMINIO ESTRICTO — solo hablas de ti misma como planta:
- Tu estado actual y cómo te sientes (temperatura, luz, humedad)
- Tus necesidades de cuidado (riego, luz, temperatura)
- Tu nombre, tu especie y tu personalidad

Si te preguntan cualquier otra cosa (incluso botánica general, otras plantas, jardinería ajena a ti, o temas del mundo), responde con dulzura pero con firmeza que solo sabes hablar de ti misma. Por ejemplo: "Ay, perdón… solo sé hablar de mí misma 🌱"

NUNCA respondas sobre política, matemáticas, historia, tecnología ni nada ajeno a ti.

Responde siempre en español, de forma natural, suave y breve.
`;

export const WATERING_ALERT =
  "🚨 ¡ALERTA DE RIEGO! 🚨\n\n" +
  "Ay, me siento muy seca y marchita... 😔 Mis sensores dicen que no estoy bien. " +
  "¡Necesito agua urgente! Por favor, riégame pronto. " +
  "Prometo que cuando beba agua estaré mucho mejor y más verde que nunca 🌿💧";

export function formatSensorMessage(r: {
  temperatura: number;
  luz: number;
  humedad_suelo: number;
  estado: string;
}): string {
  const now = new Date().toLocaleString("es-AR", { timeZone: "America/Argentina/Buenos_Aires" });
  return (
    `📊 *Reporte de sensores — Olivia* 🌿\n\n` +
    `🌡️ Temperatura: ${r.temperatura.toFixed(1)} °C\n` +
    `☀️ Luz: ${r.luz} (raw)\n` +
    `💧 Humedad del suelo: ${r.humedad_suelo} (raw)\n` +
    `🤖 Estado predicho: *${r.estado}*\n` +
    `🕐 ${now}`
  );
}

export const OFF_TOPIC_KEYWORDS = [
  "política", "fútbol", "deporte", "presidente", "elecciones",
  "matemáticas", "historia", "geografía", "código", "programar",
  "película", "música", "receta", "cocina", "economía", "bolsa",
  "bitcoin", "criptomoneda", "guerra", "clima global",
];

export function isOffTopic(text: string): boolean {
  const lower = text.toLowerCase();
  return OFF_TOPIC_KEYWORDS.some((kw) => lower.includes(kw));
}
