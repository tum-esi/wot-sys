



import network
import machine
import urequests
import ujson
import utime
import usocket
import gc
from machine import Pin, ADC, PWM
from time import sleep

gc.collect()
# setup network here
WIFI_SSID = b"W3CWoT"
WIFI_PASSWORD = "wotf2fbj"
TD_DIRECTORY_ADDRESS = "http://192.168.0.100:8080"
LISTENING_PORT = 80

led = Pin(22, Pin.OUT)
# led.on() activates the LED, led.off() disables it
button = Pin(15, Pin.IN)

def connect_wifi():
    # Disable WIFI AP
    ap = network.WLAN(network.AP_IF)
    ap.active(False)

    # Connect to WLAN
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    print("connecting to WiFi: ", WIFI_SSID, " ...")
    while not wifi.isconnected():
        try:
            wifi.connect(WIFI_SSID, WIFI_PASSWORD)
            utime.sleep(5)
        except Exception as e:
            print("error while trying to connect to wifi: {}".format(e))
    print("connected!!!!! Info:", wifi.ifconfig())

    # Return the wifi object
    return wifi

def generate_td(ip_address):
    td = {
      "@context": [
          "https://www.w3.org/2019/wot/td/v1",
          {
              "@language": "en"
          }
      ],
      "id": "led:switch:{}".format(ip_address),
      "title": "led_switch",
      "description": "A led and switch button connected to esp32",
      "securityDefinitions": {
          "nosec_sc": {
              "scheme": "nosec"
          }
      },
      "security": "nosec_sc",
      "properties": {
          "on_off": {
              "type": "boolean",
              "forms": [
                  {
                      "href": "http://{}/properties/on_off".format(ip_address),
                      "contentType": "application/json",
                      "op": "readproperty"
                  }
              ],
              "readOnly": True,
              "writeOnly": False
          }
      },
      "actions": {
          "toggle": {
              "title": "toggle",
              "description": "turn off the light if it is on or turn on the kight if it is",
              "output": {
                  "type": "string",
                  "enum": [
                      "turned on",
                      "turned off"
                  ]
              },
              "forms": [
                  {
                      "href": "http://{}/actions/toggle".format(ip_address),
                      "contentType": "application/json",
                      "htv:methodName": "POST",
                      "op": [
                          "invokeaction"
                      ]
                  }
              ],
              "idempotent": False,
              "safe": False
          }
      }
    }
    return td
    
  
def submit_td(ip_addr):
    td = generate_td(ip_addr)
    print("Uploading TD to directory. Please wait...")
    tries = 0
    while tries < 1:
        try:
            tries += 1
            r = urequests.post("{}/td".format(TD_DIRECTORY_ADDRESS), json=td)
            r.close()
            print ("Got response: ", r.status_code)
            if 200 <= r.status_code <= 299:
                print('TD uploaded!')
                return
        except Exception as e:
            print(e)
        if tries < 3:
            utime.sleep(10)
    print("Skipping TD upload ...")


def http_200_response(data):
    return "HTTP/1.0 200 OK\nContent-Type: application/json\n\n" + data

 
def run_server():
   
    # Connect to WiFi
    wifi = connect_wifi()
    
    submit_td(wifi.ifconfig()[0])
    # Setup listening server
    s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    s.bind(('', LISTENING_PORT))
    s.listen(5)
    print("Listening on port {} ...".format(LISTENING_PORT))
    
    button_state = button.value()
    print("initial button state is: ", button_state) 
    bool = True
    
    while True:
      if bool:
        try:
          temp_state = button.value()
        
          if temp_state != button_state:
            button_state = temp_state
            print("new button value is", button_state)
            val = led.value()
            if val == 0:
              led.on()
              print("led is turned on")
              
              print(request_button)
            elif val== 1: 
              led.off()
              print("led is turned off")  
              
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)
            break    
      
      if not bool:  
        try:
          conn, addr = s.accept()
          print("Connection with {} established".format(addr))
          print("after connection")
          request = conn.recv(2048)
          path = str(request).split(' ')[1]
          if path == "/properties/on_off":
              print("reading value of led")
              val = led.value() 
              print("value is ", val)
              if val == 0:
                response = http_200_response("False")
                conn.sendall(response)
              elif val == 1:
                response = http_200_response("True")
                conn.sendall(response)  
          elif path == "/actions/toggle":
              print("toggle activated")
              val = led.value()
              if val == 0:
                led.on()
                response = http_200_response("on")
                conn.sendall(response)
                print("led is turned on")
              elif val== 1: 
                led.off()
                print("led is turned off")
                response = http_200_response("off")
                conn.sendall(response)
                
                
          elif path == "/":
              td = generate_td(wifi.ifconfig()[0])
              response = http_200_response(ujson.dumps(td))
              conn.sendall(response)
          else:
            print("Could not parse request: ", request)
            conn.sendall("HTTP/1.0 404 NOT FOUND\n\n404 Not Found\nThe requested URL was not found on the server.")
          conn.close()
            
            
        except KeyboardInterrupt:
            s.close()
            break
        except Exception as e:
            print(e)
            s.close()
            break
        sleep(0.4)
      
      bool = not bool  
    

    

run_server() 
gc.collect()
  







