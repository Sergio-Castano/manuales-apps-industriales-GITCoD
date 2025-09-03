# Validaciones del Aplicativo Web

## 🧪 Tipos de validación

Después de seguir los pasos en el aplicativo web, llegamos a la parte de las **validaciones**. Estas pueden ser ejecutadas de dos maneras:

1. **De manera simulada con Node-RED**  
2. **Con el PLC**  

Empezaremos con la **validación utilizando Node-RED**.

## 🔄 Ejecución de la validación en Node-RED

Para comenzar con la validación simulada en Node-RED, seguimos estos pasos:

1. **Importar el hilo de validación en Node-RED**  
   Abre Node-RED y utiliza el siguiente comando para importar el hilo adecuado:

   ```bash
   [
    {
        "id": "b2295e74b7c3dc90",
        "type": "tab",
        "label": "Flow 1",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "693b325214d4311f",
        "type": "debug",
        "z": "b2295e74b7c3dc90",
        "name": "debug 9",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 420,
        "y": 500,
        "wires": []
    },
    {
        "id": "e80a4379c2aa73b3",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "enviar",
        "payloadType": "str",
        "x": 90,
        "y": 440,
        "wires": [
            [
                "fc70b4306a295f4f"
            ]
        ]
    },
    {
        "id": "fc70b4306a295f4f",
        "type": "mqtt out",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "topic": "/centelsa/mqtt/app/test/request",
        "qos": "2",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "9feb18610cd75adc",
        "x": 470,
        "y": 440,
        "wires": []
    },
    {
        "id": "ddcaec53de5a5b76",
        "type": "mqtt in",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "topic": "/centelsa/mqtt/app/test",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "9feb18610cd75adc",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 120,
        "y": 500,
        "wires": [
            [
                "693b325214d4311f"
            ]
        ]
    },
    {
        "id": "c9fc681641414548",
        "type": "mqtt in",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "topic": "/centelsa/mqtt/app/test/request",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "9feb18610cd75adc",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 80,
        "wires": [
            [
                "3ecf1eba32b2c135"
            ]
        ]
    },
    {
        "id": "c9f16e9bb4743609",
        "type": "mqtt out",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "topic": "/centelsa/mqtt/app/test",
        "qos": "2",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "9feb18610cd75adc",
        "x": 680,
        "y": 80,
        "wires": []
    },
    {
        "id": "3ecf1eba32b2c135",
        "type": "switch",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "property": "payload",
        "propertyType": "msg",
        "rules": [
            {
                "t": "eq",
                "v": "enviar",
                "vt": "str"
            }
        ],
        "checkall": "true",
        "repair": false,
        "outputs": 1,
        "x": 350,
        "y": 80,
        "wires": [
            [
                "9ddb271aca55d7e1"
            ]
        ]
    },
    {
        "id": "c4f5e405e70b3992",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "1",
        "payloadType": "str",
        "x": 90,
        "y": 140,
        "wires": [
            [
                "539bb4cb7876d971"
            ]
        ]
    },
    {
        "id": "3f954a574d76623c",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "0",
        "payloadType": "str",
        "x": 90,
        "y": 180,
        "wires": [
            [
                "539bb4cb7876d971"
            ]
        ]
    },
    {
        "id": "539bb4cb7876d971",
        "type": "change",
        "z": "b2295e74b7c3dc90",
        "name": "E1",
        "rules": [
            {
                "t": "set",
                "p": "E1",
                "pt": "global",
                "to": "payload",
                "tot": "msg"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 250,
        "y": 160,
        "wires": [
            []
        ]
    },
    {
        "id": "8f6975a7db52f365",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "1",
        "payloadType": "str",
        "x": 90,
        "y": 240,
        "wires": [
            [
                "211f1cca71eca650"
            ]
        ]
    },
    {
        "id": "4713599a092e9746",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "0",
        "payloadType": "str",
        "x": 90,
        "y": 280,
        "wires": [
            [
                "211f1cca71eca650"
            ]
        ]
    },
    {
        "id": "211f1cca71eca650",
        "type": "change",
        "z": "b2295e74b7c3dc90",
        "name": "E2",
        "rules": [
            {
                "t": "set",
                "p": "E2",
                "pt": "global",
                "to": "payload",
                "tot": "msg"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 250,
        "y": 260,
        "wires": [
            []
        ]
    },
    {
        "id": "2d043704958a9f63",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "1",
        "payloadType": "str",
        "x": 90,
        "y": 340,
        "wires": [
            [
                "b007bb042f08ba35"
            ]
        ]
    },
    {
        "id": "0974591e0cd69af6",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "0",
        "payloadType": "str",
        "x": 90,
        "y": 380,
        "wires": [
            [
                "b007bb042f08ba35"
            ]
        ]
    },
    {
        "id": "b007bb042f08ba35",
        "type": "change",
        "z": "b2295e74b7c3dc90",
        "name": "E3",
        "rules": [
            {
                "t": "set",
                "p": "E3",
                "pt": "global",
                "to": "payload",
                "tot": "msg"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 250,
        "y": 360,
        "wires": [
            []
        ]
    },
    {
        "id": "9ddb271aca55d7e1",
        "type": "function",
        "z": "b2295e74b7c3dc90",
        "name": "function 2",
        "func": "const E1 = global.get(\"E1\");\nconst E2 = global.get(\"E2\");\nconst E3 = global.get(\"E3\");\nconst E31 = global.get(\"E31\");\nconst N1 = global.get(\"N1\");\n\nmsg.payload = {\n    \"M341E1\": E1,\n    \"M341E2\": E2,\n    \"M341E3\": E3,\n    \"M341E31\": E31,\n    \"M341N1\": N1\n};\nreturn msg;",
        "outputs": 1,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 500,
        "y": 80,
        "wires": [
            [
                "c9f16e9bb4743609"
            ]
        ]
    },
    {
        "id": "5394e5935e2b9d66",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "1",
        "payloadType": "str",
        "x": 430,
        "y": 340,
        "wires": [
            [
                "fd0629622cdcea60"
            ]
        ]
    },
    {
        "id": "23f75847b6a16655",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "0",
        "payloadType": "str",
        "x": 430,
        "y": 380,
        "wires": [
            [
                "fd0629622cdcea60"
            ]
        ]
    },
    {
        "id": "fd0629622cdcea60",
        "type": "change",
        "z": "b2295e74b7c3dc90",
        "name": "E31",
        "rules": [
            {
                "t": "set",
                "p": "E31",
                "pt": "global",
                "to": "payload",
                "tot": "msg"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 590,
        "y": 360,
        "wires": [
            []
        ]
    },
    {
        "id": "9c00f2d18f8c54e6",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "1",
        "payloadType": "str",
        "x": 810,
        "y": 220,
        "wires": [
            [
                "35699341ece046da"
            ]
        ]
    },
    {
        "id": "fb8931deac33a3a0",
        "type": "inject",
        "z": "b2295e74b7c3dc90",
        "name": "",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "0",
        "payloadType": "str",
        "x": 810,
        "y": 260,
        "wires": [
            [
                "35699341ece046da"
            ]
        ]
    },
    {
        "id": "35699341ece046da",
        "type": "change",
        "z": "b2295e74b7c3dc90",
        "name": "N1",
        "rules": [
            {
                "t": "set",
                "p": "N1",
                "pt": "global",
                "to": "payload",
                "tot": "msg"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 970,
        "y": 240,
        "wires": [
            []
        ]
    },
    {
        "id": "9feb18610cd75adc",
        "type": "mqtt-broker",
        "name": "",
        "broker": "localhost",
        "port": "1883",
        "clientid": "",
        "autoConnect": true,
        "usetls": false,
        "protocolVersion": "5",
        "keepalive": "60",
        "cleansession": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closeQos": "0",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    }
   ]


Asegúrate de tener Node-RED instalado y corriendo en tu sistema antes de continuar.
1. **Importar el hilo de validación en Node-RED**  
   Abre Node-RED y utiliza el siguiente comando para importar el hilo adecuado:
2. **Ir a la carpeta views para hacer cambios importantes**
   
   Dirígete a la carpeta views donde se encuentran los archivos paso_4.hbs y paso_11.hbs. Aquí deberás configurar la dirección IP de acuerdo a la de tu computador. En      este caso, la dirección IP es 192.168.0.107.

   Modificar el código para la validación
   Luego de haber configurado la IP, procederemos a validar variando entre los valores 0 y 1. Para la versión actual, la validación solo está disponible con E3.1, pero puedes ajustarlo para trabajar también con E1, E2, y E3.


   En los archivos paso_4.hbs y paso_11.hbs, debes cambiar el siguiente bloque de código:
   ```bash
   switch (paso_actual) {
       case 0:
           divElement.innerText = "Se ha detectado tensión eléctrica en la sección energizada por E3.1.";
           return data.M341E31 === "0";  // Retorna true o false
       case 1:
           divElement.innerText = "Se ha detectado tensión eléctrica en la sección energizada por E3.";
           return true;  // Retorna true o false data.M341E3 === "0"
       case 2:
           divElement.innerText = "Se ha detectado tensión eléctrica en la sección energizada por E2.";
           return true;  // Retorna true o false data.M341E2 === "0"
       case 3:
           divElement.innerText = "Se ha detectado tensión eléctrica en la sección energizada por E1.";
           return true;  // Retorna true o false data.M341E1 === "0"
       default:
           return false;  // Retorna false si no coincide ningún caso
   }

Cambiar el return true en cada caso
Deberás cambiar el return true de cada uno de los casos por la línea correspondiente de código, como sigue:

Para E31, E3, E2 y E1: 
   ```bash
   return data.M341E31 === "0";
   return data.M341E1 === "0";
   return data.M341E2 === "0";
   return data.M341E3 === "0";
---

Hola

