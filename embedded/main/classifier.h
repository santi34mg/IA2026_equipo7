#pragma once

#include "sensor.h"

enum class PlantState {
    Decaida = 0,
    Estable = 1,
    Ideal   = 2,
};

namespace classifier {

PlantState predict(const SensorData &data);
const char *to_string(PlantState state);

}  // namespace classifier
