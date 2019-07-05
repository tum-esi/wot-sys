
import { Servient, ExposedThing } from "@node-wot/core";
import { HttpServer } from "@node-wot/binding-http";


const conversion = require("./conversion.js");
const exec = require('child_process').exec;
const fs = require("fs");

const NAME_PROPERTY_AMBIENT_TEMPERATURE = "ambientTemperature";
const NAME_PROPERTY_OBJECT_TEMPERATURE = "objectTemperature";
const NAME_PROPERTY_HUMIDITY = "humidity";
const NAME_PROPERTY_BAR = "pressure";
const NAME_PROPERTY_GYRO = "gyro";
const NAME_PROPERTY_ACC = "acc";
const NAME_PROPERTY_MAGNET = "magnet";
const NAME_PROPERTY_LUX = "luminosity";
const resetCommand = "sudo hciconfig hci0 down; sudo hciconfig hci0 up";

var amb_obj_Temp = [];
var amb_obj_Temp = [];
var humidity = [];
var bar = [];
var gyro_acc_mag = [];
var lux = [];

var addressList = [];
var thing = [];


var httpServer = new HttpServer({ port: 8080 });
var servient = new Servient();



function wait(ms) {
    var start = Date.now();
    var now = start;
    while (now - start < ms) {
        now = Date.now();
    }
}

var config;
function readConfig() {
    return new Promise((resolve, reject) => {
        console.log("promise");
        fs.readFile("./conf.json", "utf-8", (err, data) => {
            console.log("read");
            if (err) {
                console.log("reject");
                reject(err);

            }
            if (data) {
                console.log("resolve");
                config = JSON.parse(data);
                var strArray = (config.address);
                resolve(strArray);


            }
        })
    })
}


function readAmb_Obj_Temperature(address) {
    return new Promise((resolve, reject) => {
        var yourscript = exec(`sh measure_all.sh temp ${address}`,
            (error, stdout, stderr) => {
                // console.log(stdout);
                let strOut = stdout.toString();
                let substrOut = strOut.substr(strOut.indexOf("descriptor:") + 12, 11);
                // console.log(substrOut);
                let noSpace = substrOut.replace(/\s/g, '');
                // console.log(noSpace);
                if (error !== null) {
                    reject(console.log("exec error: ", stderr));
                }
                else {
                    resolve(noSpace);
                }
            });


    })
}


function readHumidity(address) {
    return new Promise((resolve, reject) => {
        var yourscript = exec(`sh measure_all.sh humidity ${address}`,
            (error, stdout, stderr) => {
                // console.log(stdout);
                let strOut = stdout.toString();
                let substrOut = strOut.substr(strOut.indexOf("descriptor:") + 12, 11);
                // console.log(substrOut);
                let noSpace = substrOut.replace(/\s/g, '');
                // console.log(noSpace);
                if (error !== null) {
                    reject(console.log("exec error: ", stderr));
                }
                else {
                    resolve(noSpace);
                }
            });

    })
}

function readPressure(address) {
    return new Promise((resolve, reject) => {
        var yourscript = exec(`sh measure_all.sh barometer ${address}`,
            (error, stdout, stderr) => {
                // console.log(stdout);
                let strOut = stdout.toString();
                let substrOut = strOut.substr(strOut.indexOf("descriptor:") + 12, 17);
                // console.log(substrOut);
                let noSpace = substrOut.replace(/\s/g, '');
                // console.log(noSpace);
                if (error !== null) {
                    reject(console.log("exec error: ", stderr));
                }
                else {
                    resolve(noSpace);
                }
            });

    })
}

function readLux(address) {
    return new Promise((resolve, reject) => {
        var yourscript = exec(`sh measure_all.sh lux ${address}`,
            (error, stdout, stderr) => {
                // console.log(stdout);
                let strOut = stdout.toString();
                let substrOut = strOut.substr(strOut.indexOf("descriptor:") + 12, 5);
                // console.log(substrOut);
                let noSpace = substrOut.replace(/\s/g, '');
                // console.log(noSpace);
                if (error !== null) {
                    reject(console.log("exec error: ", stderr));
                }
                else {
                    resolve(noSpace);
                }
            });

    })
}

function readGyro_Acc_Mag(address) {
    return new Promise((resolve, reject) => {
        var yourscript = exec(`sh measure_all.sh gyro_acc_mag ${address}`,
            (error, stdout, stderr) => {
                // console.log(stdout);
                let strOut = stdout.toString();
                let substrOut = strOut.substr(strOut.indexOf("descriptor:") + 12, 53);
                // console.log(substrOut);
                let noSpace = substrOut.replace(/\s/g, '');
                // console.log(noSpace);
                if (error !== null) {
                    reject(console.log("exec error: ", stderr));
                }
                else {
                    resolve(noSpace);
                }
            });

    })
}


function x() {
    return new Promise(function (resolve, reject) {
        exec(resetCommand,
            (error, stdout, stderr) => {

                console.log("bash script")
                if (error !== null) {
                    reject(console.log(`exec error: ${error}`));

                };
            });


        resolve();
    });
};


function producer(i, WoT) {
    thing[i] = WoT.produce({
        title: "TI_Sensortag" + i,
        id: "urn:dev:wot:com:example:servient:sensortag" + i,
        "@context": "https://www.w3.org/2019/wot/td/v1"
    });


    thing[i].addProperty(NAME_PROPERTY_AMBIENT_TEMPERATURE, {
        "title": "Ambient Temperature Value",
        "description": "Ambient temperature value read from temperature sensor",
        "type": "number",
        "readOnly": true
    }, 0);
    thing[i].setPropertyReadHandler(NAME_PROPERTY_AMBIENT_TEMPERATURE, function () {
        return new Promise(function (resolve, reject) {
            var value = conversion.convertAmb_Obj_Temperature(amb_obj_Temp[i], 1); //make the calculations here//
            console.log(value);
            resolve(value);
        });
    });
    thing[i].addProperty(NAME_PROPERTY_OBJECT_TEMPERATURE, {
        "title": "Object Temperature Value",
        "description": "Object temperature value read from IR-temperature sensor",
        "type": "number",
        "readOnly": true
    }, 0);
    thing[i].setPropertyReadHandler(NAME_PROPERTY_OBJECT_TEMPERATURE, function () {
        var start = Date.now();
        return new Promise(function (resolve, reject) {
            var value = conversion.convertAmb_Obj_Temperature(amb_obj_Temp[i], 2); //make the calculations here//
            console.log("obj. Temp is: " + value);
            resolve(value);
        });
    });

    thing[i].addProperty(NAME_PROPERTY_HUMIDITY, {
        "title": "Humidity Value",
        "description": "humidity value read from humidity sensor",
        "type": "number",
        "readOnly": true
    }, 0);
    thing[i].setPropertyReadHandler(NAME_PROPERTY_HUMIDITY, function () {
        var start = Date.now();
        return new Promise(function (resolve, reject) {
            var value = conversion.convertHumidity(humidity[i]); //make the calculations here//
            console.log("Humidity is: " + value);
            resolve(value);
        });
    });

    thing[i].addProperty(NAME_PROPERTY_BAR, {
        "title": "Pressure Value",
        "description": "Pressure value read from barometer sensor",
        "type": "number",
        "readOnly": true
    }, 10);
    thing[i].setPropertyReadHandler(NAME_PROPERTY_BAR, function () {
        var start = Date.now();
        return new Promise(function (resolve, reject) {
            var value = conversion.convertPressure(bar[i]); //make the calculations here//
            console.log("Pressure is: " + value);
            resolve(value);
        });
    });

    thing[i].addProperty(NAME_PROPERTY_LUX, {
        "title": "Luminosity",
        "description": "Luminosity value measured by luxometer",
        "type": "number",
        "readOnly": true
    }, 0);
    thing[i].setPropertyReadHandler(NAME_PROPERTY_LUX, function () {
        var start = Date.now();
        return new Promise(function (resolve, reject) {
            var value = conversion.convertLux(lux[i]); //make the calculations here//
            console.log("Lux is: " + value);
            resolve(value);
        });
    });

    if (config.gyro_acc_mag == true) {
        thing[i].addProperty(NAME_PROPERTY_GYRO, {
            "title": "Movement Value",
            "description": "Movement direction read from gyroscope",
            "type": "number",
            "readOnly": true
        }, 0);
        thing[i].setPropertyReadHandler(NAME_PROPERTY_GYRO, function () {
            var start = Date.now();
            return new Promise(function (resolve, reject) {
                var value = conversion.convertGyro_Acc_Mag(gyro_acc_mag[i], 1);
                console.log("gyro is: " + value);
                resolve(value);
            });
        });
        thing[i].addProperty(NAME_PROPERTY_ACC, {
            "title": "Acceleration Value",
            "description": "Acceleration values read from accelerometer",
            "type": "number",
            "readOnly": true
        });
        thing[i].setPropertyReadHandler(NAME_PROPERTY_ACC, function () {
            var start = Date.now();
            return new Promise(function (resolve, reject) {
                var value = conversion.convertGyro_Acc_Mag(gyro_acc_mag[i], 2);
                console.log("Acceleration is: " + value);
                resolve(value);
            });
        });
        thing[i].addProperty(NAME_PROPERTY_MAGNET, {
            "title": "Magnetic Field",
            "description": "Magnetic field measured by magnetometer",
            "type": "number",
            "readOnly": true
        });
        thing[i].setPropertyReadHandler(NAME_PROPERTY_MAGNET, function () {

            return new Promise(function (resolve, reject) {
                var value = conversion.convertGyro_Acc_Mag(gyro_acc_mag[i], 3);
                console.log("Magnetism is: " + value);
                resolve(value);
            });
        });
    }

    thing[i].expose().then(function () {
        console.log(i);
        console.info(thing[i].title + " ready");
    });

}
async function loopMeasure(i) {
    while (true) {
        for (var k = 0; k < i; k++) {

            amb_obj_Temp[k] = await readAmb_Obj_Temperature(addressList[k]);
            humidity[k] = await readHumidity(addressList[k]);
            bar[k] = await readPressure(addressList[k]);
            if (config.gyro_acc_mag == true) {
                gyro_acc_mag[k] = await readGyro_Acc_Mag(addressList[k]);
            }
            lux[k] = await readLux(addressList[k]);
        }
    }

};

servient.addServer(httpServer);

readConfig().then((array: any[]) => {
    addressList = array;
    x().then(() => {

        servient.start().then((wot) => {
            for (var i = 0; i < addressList.length; i++) {
                producer(i, wot);

            }

            loopMeasure(addressList.length);

        }).catch((err) => {
            console.log(err);

        })


    }).catch((err) => {
        console.log(err);
    })

})
