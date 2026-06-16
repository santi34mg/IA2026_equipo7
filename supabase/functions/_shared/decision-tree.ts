// Extracted from eda/models/DecisionTree_d4.joblib (Pipeline: SimpleImputer → StandardScaler → DecisionTreeClassifier)
const IMPUTER_MEDIANS = [12.87, 1767.0, 3265.0]; // temperatura, luz, humedad_suelo
const SCALER_MEAN = [14.14667181689959, 1424.1309977151561, 3054.8903274942877];
const SCALER_SCALE = [7.290623480288619, 851.1883595054021, 795.2331057403655];

export type PlantState = "Decaida" | "Estable" | "Ideal";

export function predictState(
  temperatura: number,
  luz: number,
  humedad_suelo: number
): PlantState {
  const raw = [temperatura, luz, humedad_suelo];
  const imputed = raw.map((v, i) => (isNaN(v) ? IMPUTER_MEDIANS[i] : v));
  const [t, l, h] = imputed.map((v, i) => (v - SCALER_MEAN[i]) / SCALER_SCALE[i]);

  // DecisionTree depth-4 (thresholds are in scaled space)
  if (t <= -0.329419) {
    if (h <= -1.187564) return "Estable";
    if (l <= -1.379989) return "Decaida";
    return "Decaida";
  } else {
    if (h <= -0.167109) return "Ideal";
    if (l <= 0.020993) return "Ideal";
    return "Estable";
  }
}
