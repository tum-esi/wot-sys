from envirophat import light,motion,weather,leds
import sys

if __name__ == "__main__":
	command = sys.argv[1]
	if command == 'led_on':
		leds.on()
		print "turned leds on"
	elif command == 'led_off':
		leds.off()
		print "turned leds off"
	elif command == "read_temp":
		print(weather.temperature())
	elif command == "read_motion":
		x, y, z, = motion.accelerometer()
		print('x:'+str(x)+' y:'+str(y)+' z:'+str(z))
	elif command == "read_heading":
		print(motion.heading())
	elif command == "read_pressure":
		print(weather.pressure(unit='hPa'))
	elif command == "read_light_level":
		print light.light()
	elif command == "read_rgb_values":
		r, g, b, = light.rgb()
		print('r:'+str(r)+' g:'+str(g)+' b:'+str(b))
	else:
		print "error: wrong command"

