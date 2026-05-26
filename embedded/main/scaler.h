#pragma once
// Generado automaticamente desde eda/modelo.ipynb - NO EDITAR A MANO.
// Escala las 3 features antes de pasarlas al clasificador.
// Orden: temperatura_c, light_raw, moisture_raw
#include <stddef.h>

namespace plant_scaler {

static constexpr float kMean[3]  = {14.146672f, 1424.130998f, 3054.890327f};
static constexpr float kScale[3] = {7.290623f, 851.188360f, 795.233106f};

inline void scale(const float in[3], float out[3]) {
    for (size_t i = 0; i < 3; ++i) {
        out[i] = (in[i] - kMean[i]) / kScale[i];
    }
}

}  // namespace plant_scaler
