const args = process.argv;
// add noWot for not building wot server 
// @warning add -d or --debug for testing device with keyboard
if (args.indexOf("--noWot") == -1 ) {
    WotMotorController = require("./dist/wot-motor-controller.js").WotMotorController
    Servient = require("@node-wot/core").Servient
    HttpServer = require("@node-wot/binding-http").HttpServer


    // ### Change this if using another direcoty ###
    const TD_DIRECTORY = "http://localhost:8080/td"
    // ###  -----------------------------------  ###


    var httpServer = new HttpServer({port: 8080});
    var servient = new Servient();

    servient.addServer(httpServer);
    servient.start().then((thingFactory) => {
        motorController = new WotMotorController(thingFactory, TD_DIRECTORY);
    });
} 

if (args.indexOf("--debug") > -1 || args.indexOf("-d") > -1 ) {
    //debug mode is active open keyboard controls
    var MotorController = require("./lib/MotorDriver.js").MotorDriver;
    var controller = new MotorController();
    const readline = require('readline');

    readline.emitKeypressEvents(process.stdin);
    process.stdin.setRawMode(true);

    console.log("use keys w a s d or arrow keys to drive x to stop");
    process.stdin.on('keypress', (str, key) => {
    console.log(str);
    if ( str === '\u0003' ) {
        // ctrl-c ( end of text )
        controller.stop();
        process.exit();
    }

    if ( str === 'w' || key.code === '[A') { //w or up arrow key
        console.log("Motor: going forward");
        controller.goStraight(255);
    } else if ( str === 's' || key.code === '[B') { //s or down arrow key
        console.log("Motor: going backward");
        controller.goStraight(-255);
    } else if ( str === 'a' || key.code === '[D') { // a or left arrow key
        console.log("Motor: turning left");
        controller.turnAround(-255);
    } else if ( str === 'd' || key.code === '[C') { //d or right arrow key
        console.log("Motor: turning right");
        controller.turnAround(255);
    } else if ( str === 'x' ) {
        console.log("Motor: stopped");
        controller.stop();
    } else if ( str === 'e' ) {
        console.log("Motor: enabled");
        controller.enable();
    } else if ( str === 'f' ) {
        if(!controller.getFaults()){
        console.log("no faults");
        }
    }
    });
} 
