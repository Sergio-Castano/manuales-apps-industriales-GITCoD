# ANEXO 5 -  Ejecución de nodos ROS2 en Raspberry Pi usando contenedores Singularity

ROS 2 (Robot Operating System 2) es un conjunto de bibliotecas y herramientas diseñadas para ayudar a los desarrolladores a crear aplicaciones principalmente en el campo de la robótica. ROS 2 utiliza DDS (Data Distribution Service) como su middleware de comunicación, permitiendo una comunicación más robusta y flexible entre nodos en una red distribuida, representando una alternativa muy practica para el desarrollo de aplicaciones industriales en general. Los componentes de ROS 2 están diseñados para ser modulares, lo que facilita la personalización y escalabilidad del sistema según las necesidades del proyecto.

ROS2 para sistemas Linux se soporta en la distribución Ubuntu, por lo tanto, para conseguir su ejecución en una plataforma Rasoberry Pi es recomendable hacerlo mediante contenedores de software. Un contenedor de software es una tecnología que permite empaquetar una aplicación y todas sus dependencias (como bibliotecas, herramientas del sistema y configuraciones) en una sola unidad ejecutable. Esta unidad se llama "contenedor" y se caracteriza por ejecutarse de manera consistente en cualquier entorno que soporte la tecnología de contenedores, sin importar las diferencias en la infraestructura subyacente. Los contenedores son una forma de virtualización a nivel de sistema operativo que proporcionan aislamiento y eficiencia. 

Entre las alternativas mas conocidas para ejecutar contenedores se encuentran Docker y Singularity. Docker es ampliamente utilizado en la industria, soportando la creación de contenedores ligeros y portátiles con una gran comunidad y ecosistema de herramientas. Singularity, por otro lado, está optimizado para entornos HPC (High-Performance Computing) y científicos, permitiendo la ejecución de contenedores sin privilegios de root, garantizando mayor seguridad y compatibilidad con sistemas de archivos y recursos del host. Singularity está diseñado para integrarse más estrechamente con el sistema operativo host, aprovechando los recursos del hardware de manera más eficiente y reduciendo en gran medida la sobrecarga que podría introducirse con la virtualización. Tras realizar algunas pruebas se verificó que las tareas ejecutadas en contenedores de Singularity se desempeñaban con un comportamiento mas cercano al nativo en comparación a tareas en contenedores de Docker. Este es un factor deseable al momento de desarrollar aplicaciones de automatización industrial, en esepcial aquellas que deben cumplir con requisitos de tiempo real. 

A continuación se presentan los pasos para lograr la ejecución de nodos de ROS2 en una Raspberry Pi 4 mediante contenedores de Singularity.

# 5.1 - Instalación de Singularity

## Paso 1 - Instalar Go

Go, también conocido como Golang, es un lenguaje de programación desarrollado por Google, con el cual fue desarrollado Singularity, por lo tanto es un requisito para su ejecución. 

### 1) Descargar el código fuente de Go

Ingrese al [sitio oficial de Go](https://go.dev/dl/) y descargue el archivo correspondiente a sistema operativo Linux y arquitectura ARM64, de modo que sea compatible con Raspberry Pi OS. Al momento de redactada esta documentación, la última versión de Go disponible es la 1.22.25

![Imagen versión de Go a Descargar](imgs/RPI4/Singularity_Go_1.png)

### 2) Instalar los archivos

- Abra una terminal y navegue hasta la carpeta en la que se descargó el archivo, por defecto esta ruta es ~/Downloads, para ello puede usar el siguiente comando:

```sh
cd ~/Downloads
```
- Extraer el archivo tar de Go, para ello ingrese el siguiente comando en la terminal:

```sh
sudo tar -C /usr/local -xzf go1.22.5.linux-arm64.tar.gz
```
### 3) Configurar las variables de entorno

- En una terminal, abra el archivo ".profile" usando el siguiente comando:
```sh
sudo nano ~/.profile
```

- Copie y pegue las siguientes líneas al final del archivo (Ctrl+Shift+c y Ctrl+Shift+v):  
```sh
export PATH=$PATH:/usr/local/go/bin
export GOPATH=$HOME/go
export PATH=$PATH:$GOPATH/bin
```
- Guarde los cambios realizados al archivo (Ctrl+s) y cierre el archivo (Ctrl+x)
- Ejecuta el siguiente comando para aplicar los cambios:
```sh
source ~/.profile
```
### 4) Verificar la instalación
- Para verificar que Go se ha instalado correctamente, en una terminal ejecute el siguiente comando:
```sh
go version
```
- Como respuesta al comando ingresado anteriormente deberá encontrar la versión de Go, el sistema operativo y la arquitectura del sistema (go version go1.22.5 linux/arm64
)

## Paso 2 - Instalar Singularity

### 1) Verifique que estén instaladas todas las dependencias para la compilación
- Abra una terminal y verifique que todas las dependencias estén instaladas y actualizadas, para ello ejecute los siguientes comandos:
```sh
sudo apt-get update
```
```sh
sudo apt-get install build-essential libssl-dev uuid-dev libgpgme11-dev squashfs-tools libseccomp-dev wget pkg-config git cryptsetup libseccomp-dev libglib2.0-dev libfuse-dev libfuse3-dev autoconf automake libtool
```

### 2) Descargar el código fuente de Singularity
- Ingrese al repositorio en GitHub de Singularity, en la sección de [releases](https://github.com/sylabs/singularity/releases) descargue la versión "general" del código fuente, haciendo clic en el enlace inmerso en el texto o en la sección assets.

![Imagen ](imgs/RPI4/Singularity_1.png)

***Importante:*** Verifique que la versión de Go instalada sea igual o superior a la versión con la que se compiló la versión de Singularity.


### 3) Preparar los archivos para compilar
- Abra una terminal y navegue hasta la carpeta en la que ha descargado el archivo comprimido de singulárity, por defecto la carpeta de descargas, para ello puede usar el siguiente comando:
```sh
cd ~/Downloads
```
- Descomprima el archivo usando el siguiente comando:
```sh
tar -xzf singularity-ce-4.1.4.tar.gz
```
- Ingrese a la carpeta producto de la descompresión:
```sh
cd singularity-ce-4.1.4/
```
- Navegue hasta el directorio "squashfuse":
```sh
cd third_party/squashfuse
```
- Configure el entorno automáticamente para la compilación ejecutando el comando:
```sh
autoreconf -i 
```
- Retorne a la ruta de la carpeta principal:
```sh
cd ../..
```  

### 4) Compilación e Instalación
- Ejecute en orden los siguientes comandos:
```sh
./mconfig 
```
```sh
make -C builddir
```
```sh
sudo make -C builddir install
```

### 5) Verificar la instalación
- Para verificar que singulárity ha sido correctamente instalado, un una nueva terminal ejecute el sifuiente comando:
```sh
singularity --version
```
- La salida obtenida debe ser similar "singularity-ce version 4.1.4".

***Importante:*** Todos los comandos y salidas deberán ser modificados en función de la versión a instalar de Singulárity, en este ejemplo se instaló la última version disponible (4.1.4).

## Paso 3 - Crear un contenedor de Singularity a partir de una imagen con ROS2 preinstalado

Los contenedores Singularity se producen a partir de imágenes inmutables en el formato Singularity Image File (SIF). Esto garantiza imágenes reproducibles y verificables y permite muchos beneficios adicionales, como la capacidad de firmar y verificar sus contenedores. Sin embargo, durante las pruebas y la depuración es posible es deseable un formato de imagen en el que se pueda escribir. De esta manera, poder instalar software y dependencias hasta asegurar el correcto funcionamiento de la aplicación a contenerizar. Para estos escenarios, Singularity admite el formato ***"sandbox"***. Este es básicamente un directorio editable a partir del cual es posible crear contenedores.

Otra ventaja que ofrece Singularity es la de crear imágenes a partir de las disponibles en el repositorio de Docker. Aprovechando esta característica, se pueden crear sandbox que contenga ROS2 preinstalado, para ello basta con ejecutar seguir este procedimiento: 

- Abra una terminal y cree una carpeta en su directorio home:

- Cree una carpeta, para la cual se sigiere el nombre "singularity_containers" y posteriormente ingrese a la carpeta:

- Ejecute el siguiente comando para crear el sandbox:
```sh
sudo singularity build --sandbox --arch arm64 ROS2/ docker://ros:humble-ros-base
```
Explorando este comando encontramos en orden tras el comando "build", que permite crear imágenes, la bandera que indica que se va a construir una imagen de tipo sandbox, la arquitectura de la plataforma host, el nombre que tendrá la imagen y el archivo a partir del cual se va a crear, en este caso, una imagen previa alojada en el repositorio de Docker (DockerHub). En este caso una imagen de ubuntu con la versión "humble" de ROS2 preinstalada.

Esto crea entonces una carpeta llamada ROS2, que corresponde al archivo a partir del cual se pueden construir los contenedores y se podrá modificar segun se requiera en función a la aplicación a desarrollar.

## Paso 4 - Editar la imagen para agregar nodos de ROS2

Tras haber creado un sandbox con ROS2, vamos a modificar la imagen para instalar un par de nodos de ROS2.

### 1) Crear un contenedor para editar la imagen

Mediante el comando ```shell``` 








