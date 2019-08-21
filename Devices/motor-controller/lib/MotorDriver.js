
const EventEmitter = require('events').EventEmitter;
const util = require('util');
var Motor = require('./Motor.js').Motor;

const MAX_SPEED = 255;
//analog value between 0 - 255

// pin definitions of driver hat
const leftMotorDiagnosticPin = 5;
const leftMotorPWMPin = 12;
const leftMotorEnablePin = 22;
const leftMotorDirectionPin = 24;

const rightMotorDiagnosticPin = 6;
const rightMotorPWMPin = 13;
const rightMotorEnablePin = 23;
const rightMotorDirectionPin = 25;


var MotorDriver = function(){
  var leftMotor = new Motor(leftMotorPWMPin, leftMotorDirectionPin, leftMotorEnablePin, leftMotorDiagnosticPin);
  var rightMotor = new Motor(rightMotorPWMPin, rightMotorDirectionPin, rightMotorEnablePin, rightMotorDiagnosticPin);

  this.version = function(msg, cb){
    cb('complete', '1.0.0');
  }

  // set speeds
  this.setSpeeds = function(leftSpeed, rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }

  //movement functions
  this.goStraight = function(speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    this.fixIfErrorExists();
  }

  this.turnLeft = function(speed) {
    //@warning give positive value to turn left
    //if negative value is given it will turn to right by going backward
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(speed);
    this.fixIfErrorExists();
  }

  this.turnRight = function(speed) {
    //@warning give positive value to turn right
    //if negative value is given it will turn to left by going backward
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(0);
    this.fixIfErrorExists();
  }

  this.turnAround = function (speed) {
    // turning right in + value
    // give minus speed for turn around in other direction
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(-speed);
    this.fixIfErrorExists();
  }

  //enable disable methods
  this.stop = function() {
    leftMotor.stop();
    rightMotor.stop();
  } 

  this.enable = function () {
    leftMotor.enable();
    rightMotor.enable();
  }

  this.disable = function () {
    leftMotor.disable();
    rightMotor.disable();
  }

  //there may be errors because of agile direction change in fast speeds,
  //which causes over current circuit protection to stop one or more motors
  // below code fixes it via disable wait enable
  this.fixIfErrorExists = function() {
    setTimeout(() => {
      if(this.getFaults()){
        this.disable();
        setTimeout(() => {
          this.enable();
          console.log("fixed");
        }, 100);
      }
    }, 100);
  }

  this.getFaults = function() {
    if(leftMotor.getFault()){
      console.log("there is a problem with left motor");
    }
    if(rightMotor.getFault()){
      console.log("there is a problem with right motor");
    }
    return leftMotor.getFault() || rightMotor.getFault()
  }
  this.getEnabled = function() {
    return leftMotor.getIsEnabled() && rightMotor.getIsEnabled();
  }  
}

util.inherits(MotorDriver, EventEmitter);
exports.MotorDriver = MotorDriver;
