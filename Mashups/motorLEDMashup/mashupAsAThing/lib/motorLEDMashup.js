"use strict";
Object.defineProperty(exports, "__esModule", { value: true });

//Mashup Itself
const util = require('util');
const EventEmitter = require('events').EventEmitter;

const Motor_Driver_TD_ADDRESS = "http://192.168.0.113:8080/MotorController";
const strip_TD_ADDRESS = "http://192.168.0.103:8080/";
var ledThing;
var motorThing;
var blinkingStatus; //0 for not signaling 1 for signaling
var blinkingDirection; //0 for left 1 for right

var MotorLEDMashup = function () {   
    WoT.fetch(Motor_Driver_TD_ADDRESS).then(async (motorTD) => {
        WoT.fetch(strip_TD_ADDRESS).then(async (stripTD) => {
            motorThing = WoT.consume(motorTD);
            ledThing = WoT.consume(stripTD);
        });
    });
    
    // motor functions

    this.getIsBlinking = function () {
        return blinkingStatus;
    }

    this.goStraight = function(speed) {
        motorThing.actions["moveStraight"].invoke({
            "speed": speed
        });
    }

    this.stop = function() {
        motorThing.actions["stop"].invoke();
    }
    this.turnLeft = function() {
        motorThing.actions["turnLeft"].invoke();
    }
    this.turnRight = function() {
        motorThing.actions["turnRight"].invoke();
    }

    // blink functions
    this.blinkRight  = function() {
        blinkingStatus = 1;
        openLedsYellow(12,18)
        blinkingDirection = 1;
    }

    this.blinkLeft  = function() {
        blinkingStatus = 1;
        openLedsYellow(23,29)
        blinkingDirection = 0;
    }

    this.stopBlinking  = function() {
        closeLeds();
        blinkingStatus = 0;
    }

    this.closeLeds = function() {
        var ledBegin = 23;
        var ledEnd = 29;
        if(blinkingDirection == 1) {//right
            ledBegin = 12;
            ledEnd = 18;
        }
        ledThing.actions["fill_array"].invoke({
            "ledBegin":ledBegin,
            "ledEnd":ledEnd,
            "color":{
                "red": 0,
                "green" : 0,
                "blue" : 0
            }
        });
    }

    this.openLedsYellow = function( ledBegin, ledEnd) {
        ledThing.actions["fill_array"].invoke({
            "ledBegin":ledBegin,
            "ledEnd":ledEnd,
            "color":{
                "red": 255,
                "green" : 106,
                "blue" : 0
            }
        });
    }
}
util.inherits(MotorLEDMashup, EventEmitter);
exports.MotorLEDMashup = MotorLEDMashup;