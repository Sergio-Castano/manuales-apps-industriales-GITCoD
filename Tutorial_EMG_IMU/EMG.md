Elementos y Conexiones
   -
Redactar los elementos (ESP32, BioAmp Exg Pill, electrodos, cables, protoboard/pcb, bateria) que se necesitan

mas info y conexiones del BioAmp Exg Pill: https://www.crowdsupply.com/upside-down-labs/bioamp-exg-pill

poner diagrama de las conexiones y explcarlas (electrodos tambien)

Poner foto del montaje total (electrodos tambien)


Configuración y Codigo en Arduino
   -
Explicacion de la configuracion con pantallazos:
1. conectar con un cable USB al PC
2. escoger el puerto USB* que se haya activado
3. seleccionar la placa ESP32 que se este usando de no tener las tarjetas seguir este tutorial: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
4. cargar el siguiente codigo en un nuevo archivo de arduino, con este codigo se testeará que el BioAmp Exg Pill funcione correctamente
  ```
INSERTE EL CODIGO AQUI DE "TEST_ENVOLVENTE_EMG"
```
con este codigo se abre el serial plotter de arduino y deberiamos ver unas señales asi, la cual deberia de variar cada vez que se realiza un estimulo en los electrodos:

"INSERTAR FOTO DE LAS SEÑALES EN EL PLOTTER"

5. una vez sabemos que todo esta funcionando correctamente cargamos el siguiente codigo en la ESP32 para que transmita por medio de MQTT los datos:
  ```
INSERTE EL CODIGO AQUI DE "EMG_FINAL_MQTT"
```
  recordar cambiar algunos parametros iniciales como IP del broker, topic a publicar, topic a suscribirse, red wifi a conectarse y ID del cliente ante el broker

  Listo el EMG esta transmitiendo por medio de MQTT una vez reciba la señal!
