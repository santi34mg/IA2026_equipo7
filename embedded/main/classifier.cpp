#include "classifier.h"

#include "model.h"
#include "scaler.h"

namespace classifier {

PlantState predict(const SensorData &data) {
    // Features en el orden entrenado: temperatura_c, light_raw, moisture_raw
    float raw[3] = {
        data.dht11_temperature_c,
        static_cast<float>(data.light_raw),
        static_cast<float>(data.moisture_raw),
    };

    float scaled[3] = {};
    plant_scaler::scale(raw, scaled);

    Eloquent::ML::Port::PlantClassifier clf;
    const int label = clf.predict(scaled);

    switch (label) {
        case 0: return PlantState::Decaida;
        case 1: return PlantState::Estable;
        case 2: return PlantState::Ideal;
        default: return PlantState::Estable;
    }
}

const char *to_string(PlantState state) {
    switch (state) {
        case PlantState::Decaida: return "Decaida";
        case PlantState::Estable: return "Estable";
        case PlantState::Ideal:   return "Ideal";
        default:                  return "Estable";
    }
}

}  // namespace classifier
