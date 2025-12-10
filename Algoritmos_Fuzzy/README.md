# Sistema de Detección de Fallos en Motores usando Corriente, Temperatura y Lógica Difusa (Fuzzy)

Este repositorio contiene el desarrollo de un sistema de detección de fallos utilizando:

- Señales de **corriente**,  
- Señales de **temperatura**,  
- Modelos de clasificación con **contexto multivariable** y un sistema de decisión basado en **Lógica Difusa (Fuzzy)**.

El objetivo principal es determinar si el motor se encuentra en **condición normal** o en **condición de fallo** mediante técnicas de machine learning y sistemas expertos.

---

# 1. Descripción General del Proyecto

La detección temprana de fallos requiere considerar el contexto físico del motor.  
Por ello, además de la señal eléctrica (corriente), se utiliza la señal térmica (temperatura), que aporta información crítica para identificar fallos mecánicos, eléctricos y de sobrecarga.

Este proyecto evalúa diferentes modelos:

- **Sin contexto**: solo corriente.  
- **Con contexto de entrada**: corriente + temperatura.  
- **Con contexto intermedio**: modulaciones intermedias de variables.  
- **Modelo Fuzzy**: sistema experto con reglas interpretables.

Cada modelo predice si el motor está:

- **0 = Normal**  
- **1 = Fallo**

---

# 2. Arquitectura del Sistema
```bash
       Corriente ───┐
                     ├──► Preprocesamiento ───► Ventaneo ───► Modelo ML ───► Probabilidad de fallo
   Temperatura ───┘
                                          │
                                          ▼
                                 Sistema Fuzzy
                                          │
                                          ▼
                               Decisión final (Normal/Fallo)

```

El sistema Fuzzy actúa como una **capa adicional de interpretación**, útil para validar o corregir decisiones ambiguas de los modelos neuronales.

---

# 3. Dataset Utilizado

El dataset utilizado contiene señales sincronizadas de:

- **Corriente**
- **Temperatura**

Fue almacenado en formato **HDF5 (.h5)** con un tamaño total de **2.51 GB**, por lo cual **NO se incluye en este repositorio** debido a las restricciones de GitHub (límite de 100 MB por archivo).

El dataset proviene de **Mendeley Data** y fue procesado para:

- limpieza,  
- interpolación,  
- sincronización temporal,  
- normalización,  
- generación de ventanas temporales.

Ejemplo de entrada al modelo:

[
ventana_corriente,
ventana_temperatura
] → etiqueta (0/1)


Este repositorio contiene **todos los notebooks y modelos entrenados**, exceptuando el dataset.

---

# 4. Estructura del Repositorio
Es una estructura con 2 carpetas, en una están los códigos y en la otra los pesos de los mejores resultados 


Los cuadernos permiten reproducir:

- el preprocesamiento,
- el entrenamiento,
- la evaluación del desempeño,
- las comparaciones entre modelos,
- y la integración con el sistema Fuzzy.

---

# 5. Modelos Implementados

Este proyecto evalúa diferentes arquitecturas que combinan:

- entradas numéricas (corriente y/o temperatura),
- modelos de machine learning,
- y un sistema de reglas difusas (Fuzzy Logic) integrado en la etapa de decisión.

La única excepción es el modelo **sin contexto**, el cual no utiliza lógica difusa.

---

## 5.1 Modelo Sin Contexto (sin reglas fuzzy)
Este modelo recibe **corriente y temperatura**, pero:

- No utiliza variables contextuales adicionales.
- No emplea sistema Fuzzy.
- La decisión depende únicamente del modelo de aprendizaje.

Sirve como línea base para comparar el impacto del contexto y la lógica difusa.

---

## 5.2 Modelos con Contexto (con reglas fuzzy)

En todos los modelos con contexto, se integran:

- **variables numéricas de entrada (corriente, temperatura o ambas)**,
- **un módulo de inferencia difusa** que refuerza o suaviza la decisión,
- **una etapa de fusión ML + Fuzzy** que entrega la clasificación final (normal/fallo).

La diferencia entre las variantes radica en **cómo y en qué punto del flujo del modelo se incorpora el contexto**.

---

### 5.2.1 Contexto de Entrada – Temperatura
- La temperatura ingresa directamente como variable contextual.
- La corriente complementa la clasificación.
- El sistema fuzzy utiliza reglas centradas en dinámicas térmicas.
- Alto rendimiento, ideal para fallos térmicos y sobrecalentamiento.

---

### 5.2.2 Contexto de Entrada – Corriente
- La corriente actúa como variable contextual principal.
- La temperatura acompaña como soporte.
- El módulo fuzzy emplea reglas orientadas a carga, torque y fallas eléctricas.
- Muy buen rendimiento en fallos mecánicos o eléctricos.

---

### 5.2.3 Contexto Intermedio – Temperatura
- La temperatura se introduce en una etapa interna del modelo.
- Se combinan características profundas con reglas fuzzy relacionadas con variaciones térmicas.
- Útil para capturar efectos lentos o acumulativos.

---

### 5.2.4 Contexto Intermedio – Corriente
- Similar al caso anterior, pero la corriente se incorpora en la fase intermedia.
- El módulo fuzzy opera sobre estimaciones internas de la red + corriente contextual.
- Excelente estabilidad en detección de fallos dinámicos.

---

# 6. Resultados (Cohen Kappa)

Se utilizó **Cohen Kappa** para evaluar el desempeño, ya que penaliza acuerdos por azar y es más robusto que accuracy para problemas desbalanceados.

Resultados obtenidos:

| Modelo                                   | Cohen Kappa |
|-------------------------------------------|-------------|
| **Sin contexto**                          | **86.88%**  |
| **Contexto Entrada – Temperatura**        | **92.53%**  |
| **Contexto Entrada – Corriente**          | **91.85%**  |
| **Contexto Intermedio – Temperatura**     | **91.71%**  |
| **Contexto Intermedio – Corriente**       | **92.20%**  |
| **Combinación (Entrada Temp + Intermedio Corriente)** | **91.07%** |

---

# 7. Interpretación de Resultados

1. **Todos los modelos con contexto superan significativamente al modelo sin contexto.**
2. El mejor modelo fue:
   - **Contexto Entrada – Temperatura (92.53%)**
3. La combinación de contextos **NO mejora el rendimiento**.
4. El contexto aporta información clave, pero:
   
> Un único tipo de contexto bien seleccionado es suficiente para capturar la dinámica asociada al fallo.

5. La lógica difusa ofrece interpretabilidad adicional y soporte en decisiones ambiguas.

---

# 8. Conclusiones

- El uso de **contexto multivariable** (corriente + temperatura) mejora significativamente la detección de fallos.  
- La combinación de varios contextos no incrementa el rendimiento, por lo que se recomienda mantener un solo tipo de contexto.  
- El sistema Fuzzy complementa al modelo ML ofreciendo reglas interpretables y una capa adicional de seguridad lógica.  
- El enfoque es viable para implementación industrial y educativa.

---

# 9. Créditos

Proyecto desarrollado por:

**Luis Miguel Porras Gaviria**  
Grupo de Investigación **GITCoD**  
Universidad Autónoma de Occidente

---

# 10. Licencia

Este proyecto se distribuye bajo licencia MIT.

