const Gpio = require('pigpio').Gpio;
const EventEmitter = require('events').EventEmitter;
const util = require('util');

var Motor = function(pwm_pin,direction_pin,enable_pin,diagnose_pin){
  var isEnabled = true;
  const MAX_SPEED = 255;

  const pwmPin = new Gpio(pwm_pin, {mode: Gpio.OUTPUT});
  var directionPin = new Gpio(direction_pin, {mode: Gpio.OUTPUT});
  var enablePin = new Gpio(enable_pin, {mode: Gpio.OUTPUT});
  var diagnosePin = new Gpio(diagnose_pin, {mode: Gpio.INPUT});
  
  diagnosePin.pullUpDown(Gpio.PUD_UP);
  enablePin.digitalWrite(1);
     
  // sets motor speed 255 for maximum speed - values for back direction
  this.setSpeed = function(speed) {
    if(speed < 0) {
        speed = -speed;
        direction = 1;
    } else {
      direction = 0;
    }
    if (speed > MAX_SPEED) {
      speed = MAX_SPEED;
    }
    directionPin.digitalWrite(direction);
    pwmPin.analogWrite(speed);
  }
  this.stop = function() {
    this.setSpeed(0);
  }
  this.enable = function() {
    isEnabled = true;
    enablePin.digitalWrite(1);
  }
  this.disable = function() {
    isEnabled = false;
    enablePin.digitalWrite(0);
  }
  this.getFault = function() {
    return !diagnosePin.digitalRead();
  }
  this.getIsEnabled = function () {
    return isEnabled;
  }
  
}
util.inherits(Motor, EventEmitter);
exports.Motor = Motor
