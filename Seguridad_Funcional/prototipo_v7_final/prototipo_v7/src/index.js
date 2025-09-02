const express = require('express');
const path = require('path');
const exphbs = require('express-handlebars');
const session = require('express-session');
const fs = require('fs');

// Initializations
const app = express();
require('./database');

// Settings
app.set('port', process.env.PORT || 3000);
app.set('views', path.join(__dirname, 'views'));
const hbs = exphbs.create({
    defaultLayout: 'main', 
    layoutsDir: path.join(__dirname, 'views', 'layouts'), 
    partialsDir: path.join(__dirname, 'views', 'partials'), 
    extname: '.hbs',
    helpers: {
        json: function (context) {
            return JSON.stringify(context);
        }
    }
});
app.engine('.hbs', hbs.engine);
app.set('view engine', '.hbs');

// Middlewares
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Routes
app.use(require('./routes/users'));
app.use(require('./routes/json_fncs'));
app.use(require('./routes/index'));

// Global Variables
app.use((req, res, next) => {
    res.locals.user = req.user || null;
    next();
});

// Static Files
app.use(express.static(path.join(__dirname, 'public')));

// Server init
app.listen(app.get('port'), () => {
    console.log('Server running on port', app.get('port'));
});
