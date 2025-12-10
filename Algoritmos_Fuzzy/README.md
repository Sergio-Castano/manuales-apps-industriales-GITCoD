# Sistema de Detección de Fallos en Motores usando Corriente, Temperatura y Lógica Difusa (Fuzzy)

Este repositorio contiene el desarrollo de un sistema de detección de fallos utilizando:

- señales de **corriente**,  
- señales de **temperatura**,  
- modelos de clasificación con **contexto multivariable**,  
- y un sistema de decisión basado en **Lógica Difusa (Fuzzy)**.

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

