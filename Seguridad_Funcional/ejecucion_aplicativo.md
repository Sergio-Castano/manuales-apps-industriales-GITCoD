# Seguridad Funcional

## ⚙️ Requisitos previos  

Antes de ejecutar el prototipo, asegúrate de cumplir con lo siguiente:

1. **Descargar la carpeta correspondiente a `prototipo_v7`.**  
2. Tener instalado **Mosquitto**, **Node.js** y **MongoDB** en tu sistema.  
   - [Descargar Node.js](https://nodejs.org/)
  
   ```bash
     # Download and install nvm:
      curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
      
      # in lieu of restarting the shell
      \. "$HOME/.nvm/nvm.sh"
      
      # Download and install Node.js:
      nvm install 22
      
      # Verify the Node.js version:
      node -v # Should print "v22.20.0".
      
      # Verify npm version:
      npm -v # Should print "10.9.3".
    ```
   - Descargar node red (Tiene como requisito previo a Node JS)
     
     ```bash
        sudo npm install -g --unsafe-perm node-red
     ```
   
      
   - [Instalación de MongoDB](https://www.mongodb.com/docs/manual/installation/)
     
   ```bash
     sudo apt-get install gnupg curl
      curl -fsSL https://www.mongodb.org/static/pgp/server-8.0.asc | \
      sudo gpg -o /usr/share/keyrings/mongodb-server-8.0.gpg \
      --dearmor
    ```
   
   - Instalación Mosquitto
     ```bash
      sudo apt update
      sudo apt install mosquitto mosquitto-clients
      ```
3. Verificar que **MongoDB esté corriendo** en tu máquina.  
   El estado debería verse similar a la siguiente imagen:  

   ![Estado de MongoDB](imagenes/mongodb.png)

---

## ▶️ Ejecución del prototipo

1. Dirígete a la carpeta descargada `prototipo_v7`.  
2. Haz **clic derecho** y selecciona **Abrir en Terminal**.  
   La ventana debería abrirse como en la siguiente imagen:  

   ![Abrir terminal en la carpeta](imagenes/terminaluno.png)

3. En ese terminal, ejecuta los siguientes comandos:  

   ```bash
   nvm use 20
   npm run dev

4. Finalmente debe salir así la terminal

    ![Abrir terminal en la carpeta](imagenes/terminal_principal.png)

5. Para ejecutar el prototipo, ingresamos a nuestro buscador y ejecutamos el puerto

   ```bash
   http://localhost:3000/
   ```
![Texto alternativo](imagenes/fotoinicio.png)

   
