const mongoose = require('mongoose');
const { Schema } = mongoose;

const UserSchema = new Schema({
    prog_date_start: { type: String, required: true },
    prog_date_end: { type: String, required: true },
    order: { type: String, requiered: true },
    machine: { type: String, requiered: true },
    person: { type: String, requiered: false },
    type: { type: String, requiered: true },
    lock_rout: { type: String, requiered: true },
    section: {type: String, requiered: true},
    maintenance_act: {type: [String], requiered: true},
    status: {type: String, requiered: true},
    last_step: {type: String, requiered: true},
    start_date: { type: Date},
    finish_date: { type: Date},
    maintenance_exec: { type: Schema.Types.Mixed },
    comment: {type: String},
    received: { type: Schema.Types.Mixed },
    date_creation: { type: Date, default: Date.now }
});

module.exports = mongoose.model('MttoProg', UserSchema); //Nombre del modelo 
