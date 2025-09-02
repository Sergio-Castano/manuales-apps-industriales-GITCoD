const path = require('path');
const multer = require('multer');
const router = require('express').Router();
const User = require('../models/User');
const fs = require('fs');

const uploadDir = path.join(__dirname, '../public/uploads/users');

router.get('/users/signup', (req, res) => {
    res.render('signup');
});

router.get('/prueba', (req, res) => {
    res.render('test', { layout: 'test'});
});

// Verificar si la carpeta existe, si no, crearla
if (!fs.existsSync(uploadDir)) {
    fs.mkdirSync(uploadDir, { recursive: true });
}

const storage = multer.diskStorage({
    destination: function (req, file, cb) {
        cb(null, uploadDir); // Carpeta donde se guardarán las imágenes
    },
    filename: function (req, file, cb) {
        const ext = path.extname(file.originalname); // Obtener la extensión (.jpg, .png, etc.)
        const baseName = path.basename(file.originalname, ext); // Nombre sin extensión
        const timestamp = Date.now(); // Marca de tiempo

        cb(null, `${timestamp}_${baseName}${ext}`); // Guardar con la extensión correcta
    }
});

const upload = multer({ storage: storage });


router.post('/users/signup', upload.single('image'), async (req, res) => {
    console.log(req.body); // Verificar qué está llegando en la solicitud

    const { user_name_123, user_email_123, tipo, equipo, user_password_123, confirm_password } = req.body;

    if (!user_name_123 || !tipo || !equipo || !user_password_123 || !confirm_password) {
        return res.status(400).json({ error: 'Todos los campos son obligatorios' });
    }

    if (user_password_123 !== confirm_password) {
        return res.status(400).json({ error: 'Las contraseñas no coinciden' });
    }

    const imagePath = req.file ? `${req.file.filename}` : null;
    console.log(imagePath);

    const newUser = new User({
        name: user_name_123,
        email: user_email_123,
        password: user_password_123,
        team: equipo,
        type: tipo,
        picture: imagePath
    });

    await newUser.save();
    return res.redirect('/users/signup');
});


router.post('/signup/:usuario', upload.single('image'), async (req, res) => {
    try {
        const id = decodeURIComponent(req.params.usuario);
        console.log(req.body);

        const { user_name_123, user_email_123, tipo, equipo, user_password_123, confirm_password } = req.body;

        if (!user_name_123 || !tipo || !equipo || !user_password_123 || !confirm_password) {
            return res.status(400).json({ error: 'Todos los campos son obligatorios' });
        }

        if (user_password_123 !== confirm_password) {
            return res.status(400).json({ error: 'Las contraseñas no coinciden' });
        }

        const imagePath = req.file ? `${req.file.filename}` : null;
        console.log(imagePath);

        const newUser = new User({
            name: user_name_123,
            email: user_email_123,
            password: user_password_123,
            team: equipo,
            type: tipo,
            picture: imagePath
        });

        await newUser.save();

        // Redirigir después de procesar el formulario
        res.redirect(`/asignar/${encodeURIComponent(id)}?success=2`);
    } catch (error) {
        console.error("Error al asignar mantenimiento:", error);
        res.status(500).send("Error interno del servidor");
    }
});


module.exports = router;
