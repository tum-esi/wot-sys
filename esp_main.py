import network
import socket
import machine
from machine import ADC
import time
import urequests
import ujson

def get_td(ip_address):

    td = {
            'id': 'light:sensor:{}'.format(ip_address),
            'name': 'light_sensor',
            'description': 'sensor that measures the intensity of light',
            'properties': {
                'intensity': {
                    "type": "number",
                    "forms": [{
                        "href": "https://{}/properties/intensity".format(ip_address),
                        "mediaType": "application/json"
                    }],
                    "minimum": 0,
                    "maximum": 1000
                }
            }
    }
    return td

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
    ifconfig = sta.ifconfig()
    print("connected! Info:", sta.ifconfig())
    
    # submit TD
    td = get_td(ifconfig[0])
    td_server_address = "http://192.168.0.100:8080" 
    while True:
        try:
            r = urequests.post("{}/td".format(td_server_address), json = td)
            if r.status_code == 201:
                print('TD uploaded!')
                break # check connectivity
        except:
            continue # don't stop, keep trying...
    # now serve the values
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
