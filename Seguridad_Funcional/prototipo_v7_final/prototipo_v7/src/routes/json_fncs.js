const app = require('express').Router();
const fs = require('fs');
const nodemailer = require('nodemailer');
const User = require('../models/User');
const MttoProg = require('../models/mtto_prog');
const path = require('path');

app.get('/api/maquinas/:id', (req, res) => {
    const maquinaId = req.params.id;

    // Suponemos que el archivo JSON tiene el nombre basado en el id de la máquina
    const filePath = path.join(__dirname, '../data/Maquinas', `${maquinaId}.json`);

    fs.readFile(filePath, 'utf8', (err, data) => {
        if (err) {
            console.error('Error leyendo el archivo JSON:', err);
            return res.status(500).send('Error al leer los datos de la máquina');
        }

        // Parsear el archivo y enviar el JSON como respuesta
        res.json(JSON.parse(data));
    });
});

app.get('/obtenerOpciones', async (req, res) => {
    const tipo = req.query.tipo;
    let opciones;

    const equipo = await User.find({ team: tipo }).lean().sort({ año: -1 });
    const nombres = equipo.map(equipo => equipo.name);
    opciones = nombres;

    res.json({ opciones });
});

module.exports = app;