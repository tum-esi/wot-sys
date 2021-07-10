var i2c = require('@abandonware/i2c');
var _ = require('underscore');

var REGISTER_DISPLAY_SETUP        = 0x80
var REGISTER_SYSTEM_SETUP         = 0x20
var REGISTER_DIMMING              = 0xE0

var ADDRESS_KEY_DATA              = 0x40

//Blink rate
var BLINKRATE_OFF                 = 0x00
var BLINKRATE_2HZ                 = 0x01
var BLINKRATE_1HZ                 = 0x02
var BLINKRATE_HALFHZ              = 0x03


function LEDBackpack(address, bus) {
    address = address || 0x70;
    bus = bus || 1 
    this.wire = new i2c(address, {device: '/dev/i2c-' + bus}); // point to your i2c address, debug provides REPL interface

    this.buffer = [0x0000, 0x0000, 0x0000, 0x0000];

    // Turn the oscillator on
    this.wire.writeBytes(REGISTER_SYSTEM_SETUP | 0x01, [0x00], function() {});

    //Turn blink off
    this.setBlinkRate(BLINKRATE_OFF);

    this.setBrightness(10);

    this.clear();
}

LEDBackpack.prototype.setBrightness = function(brightness) {
    // brightness 0-15
    if (brightness > 15) brightness = 15
    if (brightness < 0) brightness = 0
    this.wire.writeBytes(REGISTER_DIMMING | brightness, [0x00], function() {});
};

LEDBackpack.prototype.setBlinkRate = function(blinkRate){

    if (blinkRate > BLINKRATE_HALFHZ)
       blinkRate = BLINKRATE_OFF;

    this.wire.writeBytes(REGISTER_DISPLAY_SETUP | 0x01 | (blinkRate << 1), [0x00], function() {});
}

LEDBackpack.prototype.setBufferRow = function(row, value, update) {
    update = update || true;

    //Updates a single 16-bit entry in the 8*16-bit buffer
    if (row > 7)
      return;  //Prevent buffer overflow

    this.buffer[row] = value;
    if (update)
      this.writeDisplay();       //Update the display
}

LEDBackpack.prototype.writeDisplay = function() {
    var bytes = [];
    _.each(this.buffer, function(item) {
      bytes.push(item & 0xFF)
      bytes.push((item >> 8) & 0xFF)  
    });
      
    this.wire.writeBytes(0x00, bytes, function() {})
}

LEDBackpack.prototype.clear = function() {
    this.buffer = [ 0, 0, 0, 0 ];
    this.writeDisplay();
}


module.exports = LEDBackpack
