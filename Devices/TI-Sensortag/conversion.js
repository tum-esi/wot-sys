
function wait(ms) {
    var start = Date.now();
    now = start;
    while (now - start < ms) {
        now = Date.now();
    }
}


function convertAmb_Obj_Temperature(data, option) {
    console.log(typeof data);
    if (option == 1) {
        if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
            if (data.length >= 8) {
                var buf = Buffer.from(data, "hex");
                // var buf = Buffer.from('e0 08 74 0c',"hex");
                var ambientTemperature;
                ambientTemperature = buf.readInt16LE(2) / 128.0;
                return ambientTemperature;
            }
            else { console.log("not converted"); }
        }
        else {
            ambientTemperature = "no sensor value is read yet, make sure sensor is ON "
            return ambientTemperature;
        }
        


    }
    if (option == 2) {
        if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
            if (data.length >= 8) {
                var buf = Buffer.from(data, "hex");
                var objectTemperature = buf.readInt16LE(0) / 128.0
            }
            else { console.log("not converted"); }
        }
        else {
            objectTemperature = "no sensor value is read yet, make sure sensor is ON "
        }
        return objectTemperature;
    }
}


function convertHumidity(data) {
    if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
        if (data.length >= 8) {
            var buf = Buffer.from(data, "hex");
            var temperature = -40 + ((165 * buf.readUInt16LE(0)) / 65536.0); //temp data is also sent with humidity but we dont need to use it
            var humidity = buf.readUInt16LE(2) * 100 / 65536.0;
        }
        else { console.log("not converted"); }
    }
    else {
        humidity = "no sensor value is read yet, make sure sensor is ON "
    }
    return humidity;
}


function convertPressure(data) {
    if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
        if (data.length >= 12) {
            var buf = Buffer.from(data, "hex");
            var flTempBMP;
            var flPressure;

            if (buf.length > 4) {
                // Firmware 1.01

                flTempBMP = (buf.readUInt32LE(0) & 0x00ffffff) / 100.0;
                flPressure = ((buf.readUInt32LE(2) >> 8) & 0x00ffffff) / 100.0;
            } else {
                // Firmware 0.89

                var tempBMP = buf.readUInt16LE(0);
                var tempExponent = (tempBMP & 0xF000) >> 12;
                var tempMantissa = (tempBMP & 0x0FFF);
                flTempBMP = tempMantissa * Math.pow(2, tempExponent) / 100.0;

                var tempPressure = buf.readUInt16LE(2);
                var pressureExponent = (tempPressure & 0xF000) >> 12;
                var pressureMantissa = (tempPressure & 0x0FFF);
                flPressure = pressureMantissa * Math.pow(2, pressureExponent) / 100.0;
            }
        }
        else { console.log("not converted"); }
    }
    else {
        flPressure = "no sensor value is read yet, make sure sensor is ON "
    }
    return flPressure;
}


function convertLux(data) {
    if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) {//they are initialised to integer 0 so if no value measured yet they wont be changed to string
        if (data.length >= 4) {
            var buf = Buffer.from(data, "hex");
            var rawLux = buf.readUInt16LE(0);

            var exponent = (rawLux & 0xF000) >> 12;
            var mantissa = (rawLux & 0x0FFF);

            var flLux = mantissa * Math.pow(2, exponent) / 100.0;
        }
        else { console.log("not converted"); }
    }
    else {
        flLux = "no sensor value is read yet, make sure sensor is ON "
    }
    return flLux;
}

function convertGyro_Acc_Mag(data, option) {

    if (option == 1) { //gyro
        if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
            if (data.length >= 36) {
                var buf = Buffer.from(data, "hex");
                var xG = buf.readInt16LE(0) / 128.0;
                var yG = buf.readInt16LE(2) / 128.0;
                var zG = buf.readInt16LE(4) / 128.0;
                var result = {
                    "xG": `${xG}`,
                    "yG": `${yG}`,
                    "zG": `${zG}`
                };
                
            }
            else { console.log("not converted"); }
        }
        else {
            result = "no sensor value is read yet, make sure sensor is ON "
        }
        return result;
    }
    else if (option == 2) { //acc
        if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
            if (data.length >= 36) {
                var buf = Buffer.from(data, "hex");
                // we specify 8G range in setup
                var x = buf.readInt16LE(6) / 4096.0;
                var y = buf.readInt16LE(8) / 4096.0;
                var z = buf.readInt16LE(10) / 4096.0;
                var result = {
                    "x": `${x}`,
                    "y": `${y}`,
                    "z": `${z}`
                };
                
            }
            else { console.log("not converted"); }
        }
        else {
            result = "no sensor value is read yet, make sure sensor is ON "
        }
        return result;
    }
    else if (option == 3) { //magnet
        if ((typeof data == "string") && (data.includes("u") == false) && (data.includes("s")== false) && (data.includes("t")== false)) { //they are initialised to integer 0 so if no value measured yet they wont be changed to string
            if (data.length >= 36) {
                var buf = Buffer.from(data, "hex");
                // magnetometer (page 50 of http://www.invensense.com/mems/gyro/documents/RM-MPU-9250A-00.pdf)
                var xM = buf.readInt16LE(12) * 4912.0 / 32768.0;
                var yM = buf.readInt16LE(14) * 4912.0 / 32768.0;
                var zM = buf.readInt16LE(16) * 4912.0 / 32768.0;
                var result = {
                    "xM": `${xM}`,
                    "yM": `${yM}`,
                    "zM": `${zM}`
                };
            }
            else { console.log("not converted"); }
        }
        else {
            result = "no sensor value is read yet, make sure sensor is ON "
        }
        return result;
    }
    else {
        console.log("no valid sensor type choosen")
    }
}


module.exports.convertAmb_Obj_Temperature = convertAmb_Obj_Temperature;
module.exports.convertHumidity = convertHumidity;
module.exports.convertPressure = convertPressure;
module.exports.convertLux = convertLux;
module.exports.convertGyro_Acc_Mag = convertGyro_Acc_Mag;

// converTemperature(4408380c);
// convertHumidity(8864303e);
// convertPressure(d109009f6e01);  4c 0f f4 33 65 20 e9 00 80 fc 36 11 d6 ff f9 00 da fd   


