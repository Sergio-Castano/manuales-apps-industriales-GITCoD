const mongoose = require('mongoose');
const { Schema } = mongoose;

const UserSchema = new Schema({
    name: { type: String, required: true },
    email: { type: String, required: true },
    picture: { type: String, required: false },
    password: { type: String, requiered: true },
    team: { type: String, requiered: false },
    type: { type: String, requiered: true },
    date: { type: Date, default: Date.now }
});

module.exports = mongoose.model('User', UserSchema); //Nombre del modelo 

