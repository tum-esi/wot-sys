import network
import socket
import json
import machine
from machine import ADC
import time

def run():
    # setup network here
    our_wifi = b"TP-Link_5442"
    our_wifi_pw = "46484765"
    
    # debug led
    led = machine.Pin(5,machine.Pin.OUT)
    led.on()
    sta = network.WLAN(network.STA_IF)
    sta.active(True)

    try:
        print("connecting to our wifi...")
        sta.connect(our_wifi,our_wifi_pw)
        time.sleep(1)
    except Exception as e:
        print("error while trying to connect to wifi: {}".format(e))

    while not sta.isconnected():
        led.on()

    led.off()
    print("connected! Info:", sta.ifconfig())
    # setup ADC here
    adc = ADC(0)

    # setup socket here
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 80))
    s.listen(5)

    while True:
        try:
            conn, addr = s.accept()
            print("Connection with {} established".format(addr))
            request = conn.recv(1024)
            print("Content:")
            request_part = str(request).split(' ')
            if request_part[1] == "/properties/intensity":
                val = adc.read() # between 0 - 1000
                payload = {"intensity": val}
                serialized = json.dumps(payload)
                payload_length = len(serialized)
                conn.send('{}\n'.format(len(serialized)))
                conn.sendall(serialized)
            conn.close()
        except KeyboardInterrupt:
            s.close()
            break

run()
