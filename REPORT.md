# OlivIA — Tarea 2

**Curso:** Inteligencia Artificial — Universidad de Montevideo, Facultad de Ingeniería  
**Equipo:** Joaquín Bertaux · Ignacio Borreani · Franco Rodríguez · Santiago Silvera

---

## 1. Introducción

El objetivo de esta entrega es construir un clasificador supervisado que prediga la condición de la planta (Decaida / Estable / Ideal) a partir de variables sensoriales (temperatura, luz, humedad de suelo), y ejecutar la inferencia en tiempo real directamente en la ESP32, accionando un indicador LED según el estado predicho.

---

## 2. Datos y preparación

**Dataset:** 9 sesiones de grabación (`eda/datos1..9.csv`), 1642 filas tras filtrar etiquetas válidas. Columnas originales: `timestamp, temperatura, humedad, luz, humedad_suelo, label`.

**Features eliminadas:**

- `timestamp`: metadato de sesión, no aporta al modelo.
- `humedad` (DHT11 humedad ambiente): el sensor falló en 3 de las 9 sesiones, leyendo 0; se descartó para evitar ruido.

**Features usadas:** `temperatura` (°C), `luz` (ADC raw), `humedad_suelo` (ADC raw).

**Codificación del target:** ordinal multiclase — `Decaida=0, Estable=1, Ideal=2`. Distribución: Estable 765 (46.6 %), Decaida 473 (28.8 %), Ideal 404 (24.6 %).

**Pipeline de limpieza:**

1. `pd.to_numeric(errors='coerce')` sobre las 3 features.
2. Clipping IQR (1.5×) por feature antes del split (clip, no drop, para preservar la clase minoritaria Decaida); 184 valores clipeados en `humedad_suelo` → rango [684.9, 3599.9].
3. Dentro de cada pipeline: `SimpleImputer(strategy='median')` + `StandardScaler`. Los parámetros del scaler se exportan al firmware.
4. `class_weight='balanced'` en todos los estimadores que lo soportan.

**Split:** `test_size=0.2, random_state=42, stratify` → train 1313 / test 329.  
**Validación cruzada:** `StratifiedKFold(n_splits=5, shuffle, random_state=42)`, métrica `f1_macro`, sobre el conjunto de entrenamiento.

**EDA destacado:** correlación entre `luz` y condición = −0.753 (mayor correlación individual); temperatura es el feature más discriminante por clase (Decaida ≈ 8–10 °C vs. Ideal/Estable ≈ 20–25 °C); los dos primeros componentes del PCA explican el 97.7 % de la varianza total.

---

## 3. Comparación de modelos

Se entrenaron 6 modelos de clasificación. Todos envueltos en `Pipeline(SimpleImputer → StandardScaler → estimador)`.

| Modelo                          | CV macro-F1 (±std)  | Test Acc.  | Test macro-F1 | Footprint   | Exportable |
| ------------------------------- | ------------------- | ---------- | ------------- | ----------- | ---------- |
| Logistic Regression             | 0.9828 ± 0.0039     | 0.9818     | 0.9814        | 1.72 KB     | sí         |
| **Decision Tree (d=4)**         | **1.0000 ± 0.0000** | **1.0000** | **1.0000**    | **2.12 KB** | **sí**     |
| Decision Tree (d=6)             | 1.0000 ± 0.0000     | 1.0000     | 1.0000        | 2.12 KB     | sí         |
| Random Forest (20 árboles, d=5) | 0.9993 ± 0.0014     | 1.0000     | 1.0000        | 52.37 KB    | sí         |
| Gaussian NB                     | 0.8149 ± 0.0243     | 0.8116     | 0.8264        | —           | no         |
| KNN (k=5)                       | 0.9981 ± 0.0023     | 1.0000     | 1.0000        | —           | no         |

Figuras generadas por `eda/modelo.ipynb`: matrices de confusión por modelo (`confusion_*.png`) y gráfico comparativo de métricas (`comparacion_modelos.png`), almacenadas en `eda/models/`.

---

## 4. Justificación técnica del modelo elegido

**Modelo seleccionado: Decision Tree de profundidad 4 (`DecisionTree_d4`).**

Criterio: máximo macro-F1 de test **entre los modelos exportables a código C/C++ con `micromlgen`** (KNN y GaussianNB quedan descartados por no ser compatibles). Entre los exportables, el árbol profundidad 4 empata en macro-F1 (1.0000) con el de profundidad 6 y con Random Forest, pero ofrece:

- **Footprint mínimo:** 2.12 KB (vs. 52.37 KB del Random Forest); crítico dado los ~320 KB de RAM del ESP32.
- **Inferencia determinista y O(log n):** sin bucles, sin multiplicaciones de vectores, sin dependencias externas.
- **Interpretabilidad:** el árbol puede inspeccionarse e incluirse como pseudocódigo en el firmware.
- **Paridad verificada:** parity check Python ↔ C sobre 50 muestras → 50/50 OK (0 % de divergencia).

El árbol de profundidad 6 no aporta mejora métrica sobre el de profundidad 4 en este dataset y añade ramas redundantes; se descartó por parsimonia.

---

## 5. Implementación en la ESP32

### 5.1 Pipeline embebido

El modelo exportado (`embedded/main/model.h` + `embedded/main/scaler.h`) implementa:

```
# Scaler (StandardScaler exportado de sklearn)
mean  = [14.146672,  2118.209444,  2306.753046]   # temperatura, luz, humedad_suelo
scale = [ 7.290623,  1369.380783,   785.331923]

función predict(temp_c, light_raw, moisture_raw):
    x[0] = (temp_c       - mean[0]) / scale[0]
    x[1] = (light_raw    - mean[1]) / scale[1]
    x[2] = (moisture_raw - mean[2]) / scale[2]

    # Decision Tree (features escaladas)
    si x[0] <= -0.3294:               # temperatura baja (≈ 11.7 °C)
        si x[2] <= -0.2499:           # suelo relativamente seco
            si x[1] <= 0.0207:        # poca luz
                retornar Estable (1)
            sino:
                retornar Decaida (0)
        sino:
            retornar Decaida (0)      # suelo húmedo + frío → Decaida
    sino:                             # temperatura normal/alta
        si x[1] <= 0.5870:            # luz moderada
            si x[2] <= 0.8675:
                retornar Ideal (2)
            sino:
                retornar Estable (1)
        sino:                         # mucha luz
            si x[2] <= -1.1011:
                retornar Ideal (2)
            sino:
                retornar Estable (1)
```

### 5.2 Ciclo de inferencia (cada 2 500 ms)

```
loop cada 2500 ms:
    leer sensores  →  SensorData { temp_c, hum_pct, ks_temp_c, light_raw, moisture_raw }
    clase = predict(temp_c, light_raw, moisture_raw)
    set_led(clase)
    row = format_csv(epoch, temp_c, hum_pct, ks_temp_c, light_raw, moisture_raw, clase)
    enviar por serial + TCP
```

### 5.3 Indicador LED (RGB cátodo común)

| Predicción | LED      | GPIO    |
| ---------- | -------- | ------- |
| Decaida    | Rojo     | GPIO 25 |
| Estable    | Amarillo | GPIO 26 |
| Ideal      | Verde    | GPIO 33 |

Solo un LED activo por ciclo (activo-alto, mutuamente exclusivo):

```cpp
// led_indicator.cpp
void set_state(PlantState s) {
    gpio_set_level(GPIO_NUM_25, s == Decaida);
    gpio_set_level(GPIO_NUM_26, s == Estable);
    gpio_set_level(GPIO_NUM_33, s == Ideal);
}
```

---

## 6. Pruebas de inferencia en tiempo real

### Log de serial (header + muestras reales)

```
timestamp_epoch,dht11_temp_c,dht11_humidity_pct,ks0033_temp_c,light_raw,moisture_raw,predicted
2026-04-26 00:10:23,9.12,0.00,8.90,1766,3699,Decaida    →  LED ROJO
2026-04-26 15:28:22,13.37,0.00,12.80,2202,3905,Estable  →  LED AMARILLO
2026-04-23 20:48:30,25.30,28.00,24.10,2153,2768,Ideal   →  LED VERDE
```

El frontend web (`/analysis`) muestra el panel **"Predicciones Live"**: al iniciar una sesión de grabación, cada fila sensorial recibida por USB o WiFi dispara `predict_all()` sobre todos los modelos cargados, mostrando en tiempo real la predicción de cada uno en una grilla. El valor `predicted` del firmware y el de cada modelo Python coinciden, confirmando la paridad del modelo embebido.

> **Insertar aquí captura del panel "Predicciones Live"**
>
> `![Panel Predicciones Live](fotos/predicciones_live.png)`

---

## 7. Discusión

**Ventajas del modelo elegido:** footprint de 2.12 KB (menos del 0.01 % de la RAM del ESP32), inferencia en O(profundidad) ≈ 4 comparaciones, completamente determinista, sin librerías de runtime externas, interpretable e inspeccionable directamente en el código.

**Limitaciones:**

- Las métricas de test (accuracy/macro-F1 = 1.000 en varios modelos) son probablemente optimistas: las filas dentro de una misma sesión están altamente correlacionadas en el tiempo, y el split aleatorio mezcla filas de la misma sesión en train y test; una validación cruzada por sesión ("leave-one-session-out") daría una estimación más honesta de la generalización.
- El sensor DHT11 de humedad ambiente fue descartado por fallar en la mayoría de las sesiones; su inclusión con hardware funcional podría mejorar el poder discriminativo.
- Sin features temporales: el modelo trabaja muestra a muestra y no captura tendencias (ej. descenso progresivo de humedad de suelo), que podrían anticipar el estado Decaida antes de que se manifieste.
- La clase Decaida es la más minoritaria (~29 %); con datos más balanceados el modelo sería más robusto.

**Mejoras propuestas:**

- Validación leave-one-session-out para reportar métricas realistas.
- Incorporar features de tendencia temporal (δhumedad_suelo en las últimas N lecturas).
- Sensores de reemplazo funcionales para humedad ambiente.
- Actuador físico de riego automático accionado cuando la predicción sea Decaida por N ciclos consecutivos.
- Re-entrenar con datos de más especies de plantas para generalizar el sistema.
