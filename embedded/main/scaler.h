#pragma once
// Generado automáticamente desde eda/modelo.ipynb — NO EDITAR A MANO.
// Escala las 3 features antes de pasarlas al clasificador.
// Orden: temperatura_c, light_raw, moisture_raw
#include <stddef.h>

namespace plant_scaler {

static constexpr float kMean[3]  = {14.138270f, 2117.710366f, 2361.363567f};
static constexpr float kScale[3] = {7.287126f, 1369.805060f, 884.399703f};

inline void scale(const float in[3], float out[3]) {
    for (size_t i = 0; i < 3; ++i) {
        out[i] = (in[i] - kMean[i]) / kScale[i];
    }
}

}  // namespace plant_scaler
