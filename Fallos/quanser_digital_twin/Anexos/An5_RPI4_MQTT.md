# Instalar Cliente MQTT para C++ en un contenedor con ROS2

Crear un contenedor de singularity con ros y ponerle el nombre (ROS2_MQTT)

Dentro del contenedor (ingresar con sudo singularity shell -w)

Paso 1: Instalar dependencias
```sh
sudo apt update
sudo apt install build-essential cmake libssl-dev
```
Paso 2: Clonar el repositorio de Paho MQTT

```sh
# Clona el repositorio de la biblioteca C
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c

# Crear un directorio de construcción
mkdir build
cd build

# Configurar el proyecto con CMake
cmake -DPAHO_WITH_SSL=ON -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_SHARED=ON ..

# Compilar la biblioteca
make

# Instalar la biblioteca
sudo make install

# Salir del directorio de compilación
cd ../..
```

Paso 3: Clonar y compilar la biblioteca Paho MQTT C++

```sh
# Clona el repositorio de la biblioteca C++
git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp

# Crear un directorio de construcción
mkdir build
cd build

# Configurar el proyecto con CMake
cmake ..

# Compilar la biblioteca
make

# Instalar la biblioteca
sudo make install
```

Paso 4: Verificar la instalación
```sh
ls /usr/local/lib | grep paho
```
Deberías ver archivos como libpaho-mqtt3c.so, libpaho-mqtt3cs.so, libpaho-mqttpp3.so, etc.

Paso 5: Configurar el cargador de bibliotecas dinámicas
```sh
echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf.d/paho-mqtt.conf
sudo ldconfig
```
