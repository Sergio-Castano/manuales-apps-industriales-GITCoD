# Anexo 0 - Creación de tareas periódicas en una ESP32 usando FREERTOS

Para programar la tarjeta ESP32 se emplea como entorno el IDE de Arduino, en esta guía se emplea la versión 1.8.19, se parte del hecho de que este software ya está instalado en el PC que será empleado para programar la tarjeta. La ESP32 no viene incluida por defecto en el IDE de arduino, pero se puede agregar mediante el “Gestor de tarjetas”.

## Paso 1 - Agregar compatibilidad con tarjetas ESP32

1) Abrir el IDE de Arduino, ingresar en la barra de opciones superior a “Archivo/File” > “Preferencias/Preferences”.
![Imagen de la interfaz de arduino](imgs/a0/img_1.png)
    
2) En el campo “URLs adicionales de gestor de tarjetas/Additional Boards Manager URLs”, añade la siguiente URL:
```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```
![Imagen de la interfaz de arduino](imgs/a0/img_2.png)

3) Tras guardar los cambios y cerrar la ventana, ingrese en el menú de opciones superior a “Herramientas/Tools” > “Placa/Board” > “Gestor de tarjetas/Boards Manager”.
![Imagen de la interfaz de arduino](imgs/a0/img_3.png)

4) En la ventana que aparece, empleando la barra de búsqueda se ingresa “ESP32” y al aparecer los resultados se identifica el que tiene como título "esp32" y en la descripción "by Espressif Systems". Posteriormente se da clic en el botón "Instalar/Install".
![Imagen de la interfaz de arduino](imgs/a0/img_4.png)

## Paso 2 - Seleccionar la tarjeta ESP32 y el puerto

Tras haber instalado las herramientas de compatibilidad con las tarjetas ESP32 y haber conectado la tarjeta al PC mediante cable USB. Se realiza el siguiente procedimiento: 

1) Se debe ingresar a la opción “Herramientas/Tools” > “Placa/Board” > "ESP32 Arduino" y seleccionar del la referencia de la respectiva placa a programar. En nuestro caso se debe seleccionar la opción "DOIT ESP32 DEVKIT V1".
![Imagen de la interfaz de arduino](imgs/a0/img_5.png)

2) Finalmente debe de seleccionar el puerto de comunicación serial al cual está conectado la tarjeta, mediante el menú  “Herramientas/Tools” > “Puerto/Port”. En caso de estar desde un sistema operativo Windows, los nombres de los puertos siguen la estructura "COMX", mientras que en sistemas operativos Linux tienen la estructura "/dev/USBX".
![Imagen de la interfaz de arduino](imgs/a0/img_6.png)   

## Librerías requeridas


## Índice
- [Introducción](#introducción)
- [Configuración del Entorno](#configuración-del-entorno)
- [Conceptos Básicos](#conceptos-básicos)
- [Ejemplos de Código](#ejemplos-de-código)
- [Conclusión](#conclusión)

## Introducción
Bienvenido a este tutorial de programación en Python. Aquí aprenderás los fundamentos de este poderoso lenguaje.

## Configuración del Entorno
Para empezar, necesitas instalar Python en tu computadora. Sigue estos pasos:

1. Descarga Python desde [python.org](https://www.python.org/).
2. Sigue las instrucciones de instalación para tu sistema operativo.

## Conceptos Básicos
### Hola Mundo
El siguiente código imprime "¡Hola, mundo!" en la consola:
```python
print("¡Hola, mundo!")
