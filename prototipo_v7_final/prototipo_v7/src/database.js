const mongoose = require('mongoose');

mongoose.connect('mongodb://localhost/centelsa')
.then(db => console.log('DB is connected'))
.catch(err => console.log(err));
