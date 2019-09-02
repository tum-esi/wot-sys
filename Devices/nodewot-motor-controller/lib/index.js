
//debug mode is active open keyboard controls
var MotorController = require("./MotorDriver.js").MotorDriver;
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
