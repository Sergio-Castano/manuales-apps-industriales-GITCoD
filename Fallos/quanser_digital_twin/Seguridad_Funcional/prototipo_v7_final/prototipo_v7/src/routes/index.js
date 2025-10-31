const router = require('express').Router();
const fs = require('fs');
const nodemailer = require('nodemailer');
const User = require('../models/User');
const MttoProg = require('../models/mtto_prog');
const moment = require("moment-timezone");

// -- Principal
router.get('/', async (req, res) => {
    res.redirect('/signin');
});

// -- Final
router.get('/acabar', async (req, res) => {
    const redirectUrl = `/`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// -- Test JSON
router.get('/steps.json', (req, res) => {
    fs.readFile('src/models/steps.json', 'utf8', (err, data) => {
        if (err) {
            console.error('Error reading file:', err);
            res.status(500).send('Internal Server Error');
            return;
        }
        const jsonData = JSON.parse(data);
        res.json(jsonData);
    });
});

// -- Test JSON
router.get('/mecanicos.json', (req, res) => {
    fs.readFile('src/models/mecanicos.json', 'utf8', (err, data) => {
        if (err) {
            console.error('Error reading file:', err);
            res.status(500).send('Internal Server Error');
            return;
        }
        const jsonData = JSON.parse(data);
        res.json(jsonData);
    });
});

// -- Iniciar sesion - Login
router.get('/signin', async (req, res) => {
    const usuarios = await User.find({}).lean().sort({ name: 'asc' });
    res.render('signin', { layout: 'login', usuarios });
});

router.post('/signin', async (req, res) => {
    const errors = [];

    const { usuario, password } = req.body;
    const usuario_actual = await User.findOne({ email: usuario }).lean();

    if (password != usuario_actual.password) {
        errors.push({ text: 'Contraseña incorrecta' });
        res.render('signin', { layout: 'login', errors });
    }
    else {
        if (usuario_actual.type == "admin") {
            res.redirect(`/asignar/${encodeURIComponent(usuario_actual._id)}`);
        }
        if (usuario_actual.type == "user") {
            res.redirect(`/inicioApp/${encodeURIComponent(usuario_actual._id)}`);
        }
    }
});

// --- Página inicio usuario normal
router.get('/inicioApp/:usuario', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const nombreUsuario = usuario.name;
    const datosFiltrados = await MttoProg.find({
        person: nombreUsuario,
        status: "programado"
    }).lean().sort({ date_creation: -1 });
    console.log(datosFiltrados);
    res.render("inicioApp", { datosFiltrados, usuario })
});

router.post('/iniciar/:usuario', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const selectedOptionID = req.body.option;

    await MttoProg.findByIdAndUpdate(selectedOptionID, {
        start_date: Date.now(),
    });

    console.log(selectedOptionID);
    res.redirect(`/paso_1_al_3/${id}/${selectedOptionID}`);
});

// ----------- PASOS DEL 1 AL 3 -------

router.get('/paso_1_al_3/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_1_al_3', { usuario, selectedOptionID });
});


// ----------- PASO 4 - MQTT -------

router.get('/paso_0_4/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_0_4', { usuario, selectedOptionID });
});

router.post('/paso_0_4', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_0_4/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

router.get('/paso_4/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);

    //Determinar tipo de bloqueo
    const mantenimiento = await MttoProg.findById(selectedOptionID).lean();
    const electrico = mantenimiento.type === "electrico";

    console.log(electrico);

    res.render('paso_4', { usuario, selectedOptionID, electrico });
});

// Nueva ruta para manejar la solicitud POST y redirigir
router.post('/paso_4', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_4/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// ----------- PASO 5 - Liberación -------

router.get('/paso_5_al_8/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_5_al_8', { usuario, selectedOptionID });
});


router.post('/paso_5_al_8', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_5_al_8/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// ----------- PASO 9 - Intervención -------

router.get('/paso_9/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    const datosFiltrados = await MttoProg.findById(selectedOptionID);
    const listaDeTitulos = datosFiltrados.maintenance_act;

    //Determinar tipo de bloqueo
    const mantenimiento = await MttoProg.findById(selectedOptionID).lean();
    const electrico = mantenimiento.type === "electrico";

    console.log(electrico);

    res.render('paso_9', { layout: 'intervencion', listaDeTitulos, usuario, selectedOptionID, electrico });
});

router.post('/paso_9', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_9/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// ----------- PASO 10 - Video -------
router.get('/paso_10/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_10', { usuario, selectedOptionID });
});

router.post('/paso_10', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_10/${id}/${selectedOptionID}`;
    console.log(req.body.steps);
    console.log(req.body.observations);

    await MttoProg.findByIdAndUpdate(selectedOptionID, {
        maintenance_exec: req.body.steps,
        comment: req.body.observations
    });

    res.json({ success: true, redirectUrl: redirectUrl });
});


// ----------- PASO 11 - MQTT -------

router.get('/paso_11/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);

    //Determinar tipo de bloqueo
    const mantenimiento = await MttoProg.findById(selectedOptionID).lean();
    const electrico = mantenimiento.type === "electrico";

    console.log(electrico);

    res.render('paso_11', { usuario, selectedOptionID, electrico });
});

router.post('/paso_11', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_11/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// ----------- PASO 12 -------
router.get('/paso_12/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_12', { usuario, selectedOptionID });
});

router.post('/paso_12', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_12/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

// ----------- PASO 13 -------
router.get('/paso_13/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_13', { usuario, selectedOptionID });
});

router.post('/paso_13', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;
    const id = req.body.usuario;
    const redirectUrl = `/paso_13/${id}/${selectedOptionID}`;
    res.json({ success: true, redirectUrl: redirectUrl });
});

router.post('/visto_bueno/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    await MttoProg.findByIdAndUpdate(selectedOptionID, {
        received: req.body
    });
    const redirectUrl = `/paso_14/${id}/${selectedOptionID}`;
    res.redirect(redirectUrl);
});

// ----------- PASO 14 -------
router.get('/paso_14/:usuario/:mtto', async (req, res) => {
    const id = decodeURIComponent(req.params.usuario);
    const usuario = await User.findById(id).lean();
    const selectedOptionID = decodeURIComponent(req.params.mtto);
    res.render('paso_14', { usuario, selectedOptionID });
});


// --------- Finalizar --------------

router.post('/finalizar_mtto', async (req, res) => {
    const selectedOptionID = req.body.selectedOptionID;

    await MttoProg.findByIdAndUpdate(selectedOptionID, {
        finish_date: Date.now(),
        status: "finalizado",
    });

    res.json({ success: true, redirectUrl: "/" });
});


router.get('/asignar/:usuario', async (req, res) => {
    try {
        const id = decodeURIComponent(req.params.usuario);
        const usuario = await User.findById(id).lean();
        const users = await User.find({}).lean().sort({ name: 'asc' });
        const mttoProg = await MttoProg.find({ status: "programado" }).lean().sort({ date_creation: -1 });
        let mttoFinished = await MttoProg.find({ status: "finalizado" }).lean().sort({ date_creation: -1 });

        // 🔹 Formatear `finish_date` en cada documento
        mttoFinished = mttoFinished.map(item => {
            return {
                ...item,
                finish_date: item.finish_date
                    ? moment(item.finish_date).tz("America/Bogota").format("YYYY-MM-DD, HH:mm")
                    : "No disponible",
                start_date: item.start_date
                    ? moment(item.start_date).tz("America/Bogota").format("YYYY-MM-DD, HH:mm")
                    : "No disponible"
            };
        });

        const successCode = req.query.success;
        let mensaje = [];

        if (successCode) {
            switch (successCode) {
                case "1":
                    mensaje.push({ text: "Procedimiento asignado correctamente" });
                    break;
                case "2":
                    mensaje.push({ text: "Usuario creado con éxito" });
                    break;
                default:
                    mensaje.push({ text: "Operación realizada con éxito" });
                    break;
            }
        }

        res.render('asignar', { usuario, users, mensaje, mttoProg, mttoFinished, layout: "asignarLay" });

    } catch (error) {
        console.error("Error al obtener datos:", error);
        res.status(500).send("Error interno del servidor");
    }
});

router.post('/asignar/:usuario', async (req, res) => {
    try {
        const id = decodeURIComponent(req.params.usuario);
        const { maquina, orden, intervenciones } = req.body;

        for (const intervencion of intervenciones) {
            const newMtto = new MttoProg({
                prog_date_start: intervencion.date_I,
                prog_date_end: intervencion.date_F,
                order: orden,
                machine: maquina,
                person: intervencion.persona || null,
                type: intervencion.tipo,
                lock_rout: intervencion.rutina_bloq,
                section: intervencion.partesMaq,
                maintenance_act: intervencion.componentes || [],
                status: "programado",
                last_step: "N/A",
                start_date: null, // Permite valores nulos
                finish_date: null, // Permite valores nulos
                comment: "",
                date_creation: Date.now()
            });
            await newMtto.save();
        }
        // Redirigir después de procesar el formulario
        res.redirect(`/asignar/${encodeURIComponent(id)}?success=1`);
    } catch (error) {
        console.error("Error al asignar mantenimiento:", error);
        res.status(500).send("Error interno del servidor");
    }
});

router.get('/validacion', (req, res) => {
    res.render('validacion');
});

router.post('/validacion', (req, res) => {
    const { correo, pass } = req.body;
    const errors = [];
    console.log(correo)
    fs.readFile('src/models/users.json', 'utf8', (err, data) => {
        if (err) {
            console.error('Error reading file:', err);
            res.status(500).send('Internal Server Error');
            return;
        }
        const users = JSON.parse(data);

        const indiceUsuario = users.findIndex(users => users.email === correo);

        if (indiceUsuario === -1) {
            errors.push({ text: 'Usuario no registrado en la base de datos' });
            res.render('validacion', { errors, correo });
            return;
        }
        else {
            console.log(users[indiceUsuario].password);
            console.log(pass);
            if (users[indiceUsuario].password == pass) {
                const usuario = users[indiceUsuario].name;
                console.log("todo ok");
                res.redirect(`/${encodeURIComponent(usuario)}`);
            }
            else {
                errors.push({ text: 'Contraseña incorrecta' });
                res.render('validacion', { errors, correo });
            }
        }
    });
});

module.exports = router;

