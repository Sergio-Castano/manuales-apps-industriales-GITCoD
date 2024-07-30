# ANEXO 5 -  Ejecución de nodos ROS2 en Raspberry Pi usando contenedores Singularity

ROS 2 (Robot Operating System 2) es un conjunto de bibliotecas y herramientas diseñadas para ayudar a los desarrolladores a crear aplicaciones principalmente en el campo de la robótica. ROS 2 utiliza DDS (Data Distribution Service) como su middleware de comunicación, permitiendo una comunicación más robusta y flexible entre nodos en una red distribuida, representando una alternativa muy practica para el desarrollo de aplicaciones industriales en general. Los componentes de ROS 2 están diseñados para ser modulares, lo que facilita la personalización y escalabilidad del sistema según las necesidades del proyecto.

ROS2 para sistemas Linux se soporta en la distribución Ubuntu, por lo tanto, para conseguir su ejecución en una plataforma Rasoberry Pi es recomendable hacerlo mediante contenedores de software. Un contenedor de software es una tecnología que permite empaquetar una aplicación y todas sus dependencias (como bibliotecas, herramientas del sistema y configuraciones) en una sola unidad ejecutable. Esta unidad se llama "contenedor" y se caracteriza por ejecutarse de manera consistente en cualquier entorno que soporte la tecnología de contenedores, sin importar las diferencias en la infraestructura subyacente. Los contenedores son una forma de virtualización a nivel de sistema operativo que proporcionan aislamiento y eficiencia. 

Entre las alternativas mas conocidas para ejecutar contenedores se encuentran Docker y Singularity. Docker es ampliamente utilizado en la industria, soportando la creación de contenedores ligeros y portátiles con una gran comunidad y ecosistema de herramientas. Singularity, por otro lado, está optimizado para entornos HPC (High-Performance Computing) y científicos, permitiendo la ejecución de contenedores sin privilegios de root, garantizando mayor seguridad y compatibilidad con sistemas de archivos y recursos del host. Singularity está diseñado para integrarse más estrechamente con el sistema operativo host, aprovechando los recursos del hardware de manera más eficiente y reduciendo en gran medida la sobrecarga que podría introducirse con la virtualización. Tras realizar algunas pruebas se verificó que las tareas ejecutadas en contenedores de Singularity se desempeñaban con un comportamiento mas cercano al nativo en comparación a tareas en contenedores de Docker. Este es un factor deseable al momento de desarrollar aplicaciones de automatización industrial, en esepcial aquellas que deben cumplir con requisitos de tiempo real. 

A continuación se presentan los pasos para lograr la ejecución de nodos de ROS2 en una Raspberry Pi 4 mediante contenedores de Singularity.

# 5.1 - Instalación de Singularity

## Paso 1 - Descargar e Instalar Go

Go, también conocido como Golang, es un lenguaje de programación desarrollado por Google, con el cual fue desarrollado Singularity, por lo tanto es un requisito para su ejecución. 

1) Ingrese al [sitio oficial de Go](https://go.dev/dl/) y descargue el archivo correspondiente a sistema operativo Linux y arquitectura ARM64, de modo que sea compatible con Raspberry Pi OS. Al momento de redactada esta documentación, la última versión de Go disponible es la 1.22.25

![Imagen versión de Go a Descargar](imgs/RPI4/Singularity_Go_1.png)

