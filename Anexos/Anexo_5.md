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

### 1) Descargar el código fuente de Singularity
- Ingrese al repositorio en GitHub de Singularity, en la sección de ["releases"](https://github.com/sylabs/singularity/releases) descargue la versión "general" del código fuente, haciendo clic en el enlace inmerso en el texto o en la sección assets.

***Importante:*** Verifique que la versión de Go instalada sea igual o superior a la versión con la que se compiló la versión de Singularity.

