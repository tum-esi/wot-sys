const { Servient } = require('@node-wot/core')
const { HttpServer } = require('@node-wot/binding-http')
const gpio = require('onoff').Gpio
const piCamera = require('pi-camera')

let servient = new Servient()
servient.addServer(new HttpServer())

servient.start().then( (WoT) => {
	console.log('Servient started')
	let thing = WoT.produce(
	{
		name: "light_sensor",
		description: "example light sensor",
		id: "travis:light:sensor"
	})
	.addProperty(
		"brightness",
		{
			type: "integer",
			minimum: 0,
			maximum: 255,
			writable: true
		},
		100
	)
	.expose()
	.then( () => { console.log('light sensor ready')})
})
