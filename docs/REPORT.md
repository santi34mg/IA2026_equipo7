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

## 6. Arquitectura del sistema

### 6.1 Parser compartido (`common/parser.py`)

Para evitar duplicación de lógica, se extrajo el parser de líneas seriales en un módulo compartido (`common/parser.py`). Tanto la herramienta CLI de grabación (`record_csv.py`) como el frontend web importan de aquí. El parser implementa:

- Filtrado de líneas de log de ESP-IDF.
- Conversión de segundos uptime del ESP32 a timestamp de reloj de pared.
- Validación y formateo de columnas CSV.
- Anexión de la columna `estado` (etiqueta asignada al inicio de sesión).

### 6.2 Frontend web — FastAPI

Se desarrolló una aplicación web moderna con FastAPI + Uvicorn que proporciona interfaz única para flashing, grabación y visualización de datos en vivo.

**Estructura:**

```
frontend/
├── requirements.txt                 # pyserial, esptool, fastapi, uvicorn, pydantic, pandas, numpy, scikit-learn, matplotlib, seaborn, joblib
├── app/
│   ├── main.py                      # Aplicación FastAPI, todas las rutas
│   ├── models.py                    # Esquemas request/response (Pydantic)
│   ├── session.py                   # Máquina de estados: idle → flashing → recording → idle
│   ├── flasher.py                   # Ejecuta esptool como subprocess, transmite salida en vivo
│   ├── recorder.py                  # Lee puerto serial, escribe CSV, emite filas por WebSocket
│   ├── firmware.py                  # Valida presencia de binarios pre-compilados en firmware/*.bin
│   ├── ports.py                     # Lista puertos seriales disponibles
│   ├── discovery.py                 # Descubrimiento WiFi via UDP multicast
│   ├── analysis.py                  # AnalysisService: carga modelos, renderiza gráficos EDA, predice en vivo
│   └── ws_bus.py                    # Bus de broadcast WebSocket a todas las pestañas abiertas
└── static/
    ├── index.html                   # SPA única con interfaz de usuario
    ├── analysis.html                # Página de análisis con gráficos EDA y predicciones live
    ├── style.css                    # Estilos
    ├── app.js                       # Máquina de estados vanilla JS
    └── analysis.js                  # Lógica de análisis y predicciones
```

**Rutas principales:**

- `GET /` → interfaz de flashing y grabación.
- `GET /analysis` → panel de EDA, modelos cargados y predicciones en vivo.
- `POST /flash` → inicia flashing, emite progreso por WebSocket.
- `POST /record/start` → inicia grabación.
- `POST /record/stop` → detiene grabación, devuelve path y número de filas.
- `WS /ws` → WebSocket para estado de sesión y filas CSV en vivo.

**Validación de modelos:** si el directorio `eda/models/` está ausente, la app inicia en modo degradado: `/` funciona normalmente pero `/analysis` muestra un banner indicando que los modelos no están cargados. **Regenerar modelos:** cada vez que cambien los archivos `eda/datos*.csv`, es necesario re-ejecutar `eda/modelo.ipynb` de principio a fin para regenerar `*.joblib`, `metrics.json` y gráficos de confusión.

### 6.3 Directorio `firmware/`

Almacena los tres binarios pre-compilados del ESP32:

```
firmware/
├── README.md
├── bootloader.bin
├── partition-table.bin
└── embedded.bin
```

Estos binarios son generados una vez por un desarrollador con ESP-IDF v6.0 instalado, y luego commiteados a git. De este modo, todos los demás pueden usar el frontend sin necesidad de tener ESP-IDF instalado.

**Compilación (desarrollador con ESP-IDF):**

```bash
cd embedded
idf.py build
cd ..

cp embedded/build/bootloader/bootloader.bin             firmware/
cp embedded/build/partition_table/partition-table.bin   firmware/
cp embedded/build/embedded.bin                          firmware/

git add firmware/*.bin && git commit -m "Add pre-built firmware binaries"
```

---

## 7. Pruebas de inferencia en tiempo real

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

## 8. Discusión

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

---

## 9. Instrucciones de instalación y ejecución

### 9.1 Setup inicial

**Instalación de dependencias Python:**

```bash
.venv/bin/pip install -r frontend/requirements.txt
```

**Permiso de puerto serial (solo Linux):**

```bash
sudo usermod -aG dialout $USER
# luego cerrar sesión y volver a iniciar (o ejecutar: newgrp dialout)
```

**Setup de firmware pre-compilado (una sola vez):**

Un desarrollador con ESP-IDF v6.0 debe compilar y copiar los binarios una vez. Luego se commitean a git para que todos los demás puedan usarlos directamente.

```bash
cd embedded
idf.py build
cd ..

cp embedded/build/bootloader/bootloader.bin             firmware/
cp embedded/build/partition_table/partition-table.bin   firmware/
cp embedded/build/embedded.bin                          firmware/

git add firmware/*.bin && git commit -m "Add pre-built firmware binaries"
```

### 9.2 Ejecución

**Modo normal (con ESP32 y binarios de firmware):**

```bash
PYTHONPATH=. .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

Abrir **http://localhost:8000** en el navegador.

1. Seleccionar puerto serial (click en **Refresh** si el ESP32 no aparece).
2. Elegir estado de la planta: `Decaida`, `Estable`, `Ideal`, o ingresar una etiqueta personalizada.
3. Ingresar ruta completa para el CSV de salida (ej: `/home/user/recordings/run1.csv`). Click en **Validate** para validar.
4. Click en **Run measurement** — se flashea el ESP32, luego inicia grabación automáticamente.
5. Click en **Stop** al terminar. Se muestran ruta de CSV y número de filas.

**Modo mock (sin ESP32):**

Útil para desarrollo e testing de la UI. Simula el flujo completo con datos sintéticos.

```bash
PYTHONPATH=. FRONTEND_MOCK=1 .venv/bin/python -m uvicorn frontend.app.main:app --host 127.0.0.1 --port 8000
```

**Grabadora CLI (sin cambios respecto a la versión anterior):**

La herramienta de línea de comandos original sigue funcionando:

```bash
# sin etiqueta
PYTHONPATH=. .venv/bin/python record_csv.py

# con etiqueta
PYTHONPATH=. .venv/bin/python record_csv.py Ideal
```

### 9.3 Formato de CSV de salida

```
timestamp,dht_temp,dht_humedad,ks_temp,light,soil_humidity,predicted,estado
2026-05-26 14:23:01,23.45,55.0,22.10,2153,1766,Ideal,Ideal
```

Una fila cada ~2.5 segundos. Compatible con `eda/main.ipynb` y `eda/modelo.ipynb`.

### 9.4 Notebook EDA

```bash
cd eda && jupyter notebook main.ipynb
```

Para regenerar los modelos tras cambios en `datos*.csv`:

```bash
cd eda && jupyter notebook modelo.ipynb
# ejecutar de principio a fin
```
