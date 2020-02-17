const Gpio = require('../node_modules/pigpio').Gpio;

class A4988 {

    constructor({ step = 15, dir = 14, ms1 = 24, ms2 = 23, ms3 = 18, enable = 25 }) {

        this._abort = false;
        this._delay = 1;
        this._direction = false;
        this._steps = 0;
        this._step_size = 'FULL';
        this._enabled = true;
        this._turning = false;

        this._step = new Gpio(step, { mode: Gpio.OUTPUT });
        this._dir = new Gpio(dir, { mode: Gpio.OUTPUT });
        this._step.digitalWrite(false);
        this._dir.digitalWrite(false);

        if (ms1 && ms2 && ms3) {

            this._ms1 = new Gpio(ms1, { mode: Gpio.OUTPUT });
            this._ms2 = new Gpio(ms2, { mode: Gpio.OUTPUT });
            this._ms3 = new Gpio(ms3, { mode: Gpio.OUTPUT });

            this._ms1.digitalWrite(false);
            this._ms2.digitalWrite(false);
            this._ms3.digitalWrite(false);

        }

        if (enable) {
            this._enable = new Gpio(enable, { mode: Gpio.OUTPUT });
            this._enable.digitalWrite(false);
        }

    }

    get delay() {
        return this._delay;
    }

    set delay(d) {
        if (typeof d != 'number') throw `'delay' must be a number (${d})`;
        if (d <= 0) throw `'delay' must be >= 0 (${d})`;
        this._delay = d;
    }

    get direction() {
        return this._direction;
    }

    set direction(d) {
        if (typeof d != 'boolean') throw `'direction' must be boolean (${d})`;
        this._direction = d;
        this._dir.digitalWrite(d);
    }

    get step_size() {
        return this._step_size;
    }

    set step_size(ss) {
        if (!this._ms1) return;
        if (typeof ss != 'string') throw `'step_size' must be a string (${ss})`;
        switch (ss.toUpperCase()) {
            case 'FULL':
                this._ms1.digitalWrite(false);
                this._ms2.digitalWrite(false);
                this._ms3.digitalWrite(false);
                this._step_size = 'FULL';
                break;
            case 'HALF':
                this._ms1.digitalWrite(true);
                this._ms2.digitalWrite(false);
                this._ms3.digitalWrite(false);
                this._step_size = 'HALF';
                break;
            case 'QUARTER':
                this._ms1.digitalWrite(false);
                this._ms2.digitalWrite(true);
                this._ms3.digitalWrite(false);
                this._step_size = 'QUARTER';
                break;
            case 'EIGHTH':
                this._ms1.digitalWrite(true);
                this._ms2.digitalWrite(true);
                this._ms3.digitalWrite(false);
                this._step_size = 'EIGHTH';
                break;
            case 'SIXTEENTH':
                this._ms1.digitalWrite(true);
                this._ms2.digitalWrite(true);
                this._ms3.digitalWrite(true);
                this._step_size = 'SIXTEENTH';
                break;
            default:
                break;
        }
    }

    get enabled() {
        return this._enabled;
    }

    set enabled(e) {
        if (e) {
            this._enable.digitalWrite(false);
            this._enabled = true;
        } else {
            this._enable.digitalWrite(true);
            this._enabled = false;
        }
    }

    get turning() {
        return this._turning;
    }

    enable() {
        this._enable.digitalWrite(false);
        this._enabled = true;
        return new Promise(res => setTimeout(res, 5));
    }

    disable() {
        this._enable.digitalWrite(true);
        this._enabled = false;
        return new Promise(res => setTimeout(res, 5));
    }

    turn_direction(td) {
        this.direction = td;
    }

    turn_speed(s) {
        this.delay = s;       
    }

    turn_step_size(tss) {
        this.step_size = tss;       
    }

    // _turn(steps, res) {
    _turn(res) {
        if (this._abort) {
            this._turning = false;
            res(this._steps);
            return;
        }
        // this._steps++;
        this._step.digitalWrite(true);
        this._step.digitalWrite(false);
        // if (this._steps == steps) {
            // this._turning = false;
        //     res(this._steps);
        //     return;
        // }
        // setTimeout(() => this._turn(steps, res), this._delay);
        setTimeout(() => this._turn(res), this._delay);
    }

    turn(steps = 1, callback) {
        if (this._turning) return Promise.reject(new Error('Motor already running'));
        this._steps = 0;
        this._abort = false;
        this._turning = true;
        if (typeof callback == 'function') {
            // this._turn(steps, callback);
            this._turn(callback);
        } else {
            // return new Promise(res => this._turn(steps, res));
            return new Promise(res => this._turn(res));
        }
    }

    stop() {
        this._abort = true;
    }

}

module.exports = A4988;
