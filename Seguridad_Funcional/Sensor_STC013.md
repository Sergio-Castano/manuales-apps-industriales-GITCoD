# Medición de Corriente AC con SCT-013 y Arduino UNO

Este proyecto implementa la lectura de corriente alterna (AC) utilizando el sensor **SCT-013-010** junto con un **Arduino UNO**.  
La señal del sensor es acondicionada mediante una **PCB**, la cual integra el circuito de polarización, filtrado y adaptación de nivel para su correcta lectura por el ADC del Arduino. Para luego mediante serial ser medido por el PLC 19R+

---

## 📌 Descripción general del sistema

El sensor **SCT-013-010** es un transformador de corriente no invasivo que genera una señal AC proporcional a la corriente que circula por un conductor.

Dado que el ADC del Arduino solo puede medir voltajes positivos (0–5 V), la señal del sensor debe ser:

- Desplazada a un nivel medio de 2.5 V  
- Filtrada  
- Limitada dentro del rango del ADC  

Todo este acondicionamiento se realiza directamente en la **PCB**, por lo que el Arduino únicamente recibe una señal lista para ser digitalizada.

---

## ⚙️ Código Arduino

El siguiente código realiza:

- Promediado de lecturas ADC  
- Conversión de cuentas ADC a voltaje  
- Eliminación del offset de 2.5 V para obtener la señal AC centrada en cero  

```cpp
void setup() {
  Serial.begin(115200);
}

void loop() {
  float senal = voltaje_promedio(10) * (5.0 / 1023.0) - 2.5;
  Serial.println(senal);
}

int voltaje_promedio(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(A0);
  }
  return (suma / n);
}
```
