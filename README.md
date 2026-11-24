# Estimación de estado con filtros de localización

Actividad PGRA02_UA2_AA1 · Estimación de localización en MATLAB.

La solución compara EKF, UKF y MCL en un mapa de ocupación conocido, con mediciones GPS e IMU. Se conserva el ciclo protegido de la plantilla docente y se estudian las variaciones autorizadas de `Q_gps` y `Q_imu`.

## Estructura

```text
Localizacion_Robot_Movil/
├── actividad.mlx                # Live Script con explicación de los filtros
├── actividad.m                  # Código equivalente para ejecución automática
├── preparar_actividad.m         # Prepara una ejecución individual del Live Script
├── escenarios_evaluados.m        # Ejecuta E0–E4 y organiza resultados
├── evaluar_trayectorias.m        # APE/RPE SE(2) a partir de los historiales
├── guardar_historial.m           # Exporta poses y parámetros del ensayo
├── exampleMaps.mat              # Mapa complexMap
├── auxiliar/                    # Funciones de navegación, corrección y evaluación
├── verificacion_integridad.md    # Auditoría estática de código
└── resultados/
    ├── actividad/               # Ejecuciones individuales del Live Script
    └── escenarios_evaluados/    # Ejecuciones completas E0–E4
```

## Requisitos

MATLAB R2021a o superior, Robotics System Toolbox, Navigation Toolbox y Statistics and Machine Learning Toolbox. Las funciones de la plantilla pueden generar una advertencia de obsolescencia de `MaxAngularVelocity` según la versión de MATLAB. Se conserva el controlador docente.

## Ejecución completa

Descomprimir el proyecto y seleccionar su carpeta raíz como Current Folder de MATLAB. Guardar previamente otros trabajos abiertos: la inicialización original contiene `clear all`, `close all` y `clc`.

Desde Command Window:

```matlab
salida = escenarios_evaluados;
```

## Ejecución individual del Live Script

Desde la raíz:

```matlab
preparar_actividad
```

Se crea una carpeta de trabajo en `resultados/actividad/`, se abre la copia de `actividad.mlx` y se establece su directorio de trabajo. Pulsar **Run All**, guardar el Live Script y ejecutar:

```matlab
guardar_historial
```

Las gráficas y archivos se generan en esa carpeta. El Live Script de la raíz permanece como fuente limpia. El archivo de la raíz no contiene salidas antiguas. La ejecución automática utiliza `.m` y no guarda salidas incrustadas en un `.mlx`; para obtener el entregable académico con salidas, usar el procedimiento individual.

## Escenarios

| Escenario | Desviación GPS (m) | Desviación IMU (°) | Varianza GPS (m²/eje) | Varianza IMU (rad²) |
|---|---:|---:|---:|---:|
| E0 | 0.90 | 20 | 0.81 | 0.121847 |
| E1 | 0.45 | 20 | 0.2025 | 0.121847 |
| E2 | 1.80 | 20 | 3.24 | 0.121847 |
| E3 | 0.90 | 10 | 0.81 | 0.030462 |
| E4 | 0.90 | 40 | 0.81 | 0.487388 |

El automatizador no establece una semilla. **El planificador docente `auxiliar/planificar_ruta.m` sí contiene `rng(42)`**, conservado íntegramente. Esta configuración interna debe considerarse al interpretar la reproducibilidad; no se ha añadido ni retirado ninguna semilla. Las secuencias consumidas pueden variar entre escenarios.

## Funcionamiento de los filtros

- **EKF:** predice la pose con el modelo diferencial, propaga la covarianza mediante la jacobiana y corrige con la innovación y ganancia de Kalman. La forma de Joseph estabiliza la actualización de covarianza.
- **UKF:** genera siete puntos sigma para representar una pose de tres dimensiones, propaga cada punto, reconstruye la media y covarianza con pesos y aplica la corrección para observación directa. Utiliza media circular para la orientación.
- **MCL:** propaga 300 partículas con ruido de proceso, pondera las hipótesis mediante log-verosimilitudes de las mediciones válidas y estima la pose por media ponderada. Aplica remuestreo sistemático e inyección adaptativa local cuando corresponde.

El ciclo original propaga el estado a 10 Hz. EKF y UKF se corrigen cuando hay GPS (aproximadamente 1 Hz); MCL aprovecha IMU a 10 Hz. El criterio de parada compara las tres poses estimadas con la meta y no la pose real. Estas condiciones se mantienen.

## Resultados

```text
resultados/escenarios_evaluados/<fecha_hora>/
├── E0/ ... E4/
│   ├── actividad.m, auxiliar/, exampleMaps.mat, guardar_historial.m
│   ├── map_trajectory.png
│   ├── comparativa_x_y_theta.png
│   ├── metricas.csv
│   ├── historial_localizacion.mat
│   └── truth.txt, ekf_est.txt, ukf_est.txt, mcl_est.txt
├── comparativa_escenarios.csv
├── navegacion.csv
├── verificacion_bloques.txt
├── RMSE_pos_m.png
├── RMSE_theta_deg.png
├── Error_final_m.png
├── Latencia_ms_llamadas.png
└── evaluacion/
    ├── resumen_SE2.csv
    └── E0/ ... E4/
        ├── evaluacion.png
        ├── cajas_APE.png
        └── CSV individuales APE, RPE_1, RPE_10 y P95
```

La ejecución completa produce 24 gráficas: diez de simulación, cuatro comparativas y diez de evaluación ampliada. `navegacion.csv` distingue distancia real y estimada a la meta. Los resultados parciales se conservan si ocurre un error; consultar `verificacion_bloques.txt` y el mensaje de MATLAB.

## Evaluación

La evaluación se ejecuta automáticamente al finalizar E0–E4. Para repetirla sobre los historiales sin simular nuevamente, renombrar primero la carpeta `evaluacion` existente y ejecutar desde la raíz:

```matlab
resumen = evaluar_trayectorias(salida);
```

También puede suministrarse la ruta completa de una ejecución. Se calculan APE y RPE traslacional y angular en SE(2), con intervalos de 0,1 y 1 s, sin alineación ni escala. Se exportan estadísticas y P95.

## Funciones auxiliares

| Archivo | Función |
|---|---|
| cargar_mapa.m | Mapa original y mapa inflado para planificación |
| planificar_ruta.m | Ruta RRT*; incluye la semilla original del docente |
| controlador_robot.m | Control Pure Pursuit |
| corregir_estado.m | Innovación, ganancia y corrección de EKF/UKF |
| calcular_metricas.m | Métricas por filtro y CSV |
| graficar_evolucion_estados.m | Evolución de x, y y orientación |
| guardar_formato_tum.m | Exportación de poses en formato TUM |
| compute_*.m | Funciones originales conservadas por trazabilidad; no invocadas por el cálculo de métricas vigente |

Las funciones originales `compute_nees`, `compute_nis`, `compute_latency`, `compute_rmse` y `compute_rpe` no deben utilizarse como sustitución directa de las métricas actuales: sus definiciones no corresponden necesariamente a la evaluación temporal o a SE(2). NEES y NIS se reportan como NaN porque el ciclo protegido no guarda covarianzas e innovaciones temporales. La latencia es por llamada, no por ciclo completo.

## Informe técnico

El informe presenta el desarrollo matemático de los filtros
EKF, UKF y MCL, la metodología experimental, las métricas
de evaluación y el análisis de resultados.

[Consultar informe técnico en PDF](documentacion/Informe_Estimacion_Localizacion.pdf)

## Resultados experimentales

[Escenarios evaluados](resultados/escenarios_evaluados/)

[Ejecución del Live Script](resultados/actividad/)

[Verificación de integridad](verificacion_integridad.md)
