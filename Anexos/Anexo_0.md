# ANEXO 0 - Tareas periódicas en una ESP32 usando FREERTOS

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

## Paso 3 - Librerías requeridas

FREERTOS está incorporado de fábrica en los microcontroladores ESP32, ya que estas tarjetas usan este sistema operativo para gestionar las tareas básicas del microcontrolador, con lo cual se puede utilizar sin necesidad de añadir cabeceras ni librerías adicionales para usarlo.

## Paso 4 -  Programación de tareas con FREERTOS

En general, existen dos bloques de código necesarios para la programación de tareas en FREERTOS: el bloque de creación de la tarea y la función que ejecuta la tarea. En este caso se va a crear una tarea que va a ejecutar una función de forma periódica.

### Creación de la tarea

Existen dos funciones de FREERTOS para crear tareas, estas son **"xTaskCreate"** y **"xTaskCreatePinnedToCore"**. Ambas funciones permiten crear tareas pero su diferencia es que **"xTaskCreate"** deja que el sistema operativo seleccione líbremente el núcleo al cual asignar la tarea, mientras que con **"xTaskCreatePinnedToCore"** mediante un parámetro se especifica a cual nucleo se le asignará la tarea para su ejecución. 

Se debe tener precaución al crear y asignar tareas manualmente, pues el ESP32 utiliza el núcleo 0 para ciertas tareas criticas (WiFi, bluetooth, entre otras), con lo cual, asignar otras tareas de alta prioridad y con un tiempo de ejecución alto a este núcleo puede dar lugar a que el microcontrolador se reinicie si se interrumpen o que no se ejecuten a tiempo. Es por ello que se recomienda asignar ese tipo de tareas al núcleo 1. 

Sin importar la función a emplear para crear la tarea, el bloque de código que corresponde a su llamado ha de disponerse al interior de la función "void setup()". En este ejemplo se creará una tarea usando "xTaskCreatePinnedToCore". A continuación se muestra el código correspondiente:

```C
void setup() {
  xTaskCreatePinnedToCore(
    TaskBlink, // Nombre de la función a ejecutar
    "Taskname", // Nombre de la tarea
    1024, // Tamaño de la pila
    NULL, // Parámetros a pasar a la función
    0, // Prioridad
    NULL, // Task handler
    1 // Nucleo
  );
}
```
Como se puede observar, el llamado a esta función requiere de varios parámetros, los cuales se detallan a continuación:
- Hola
- Si
- no

