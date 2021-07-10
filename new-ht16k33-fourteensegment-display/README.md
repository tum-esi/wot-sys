###Fourteen segment display with HT16K33 backpack
Forked from Magnus Nordin's seven segment display driver
Driver for the HT16K33 backpack and 14-segement display from [Adafruit](https://learn.adafruit.com/adafruit-led-backpack/0-54-alphanumeric).

###Installation
```sh
npm install ht16k33-fourteensegment-display
```

###Example
```js
var FourteenSegment = require('ht16k33-fourteensegment-display');

var display = new FourteenSegment(0x70, 1);
display.writeString("Test");

```

###Licensing
MIT
