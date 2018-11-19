# instructions
- clone this repo
- `python3 td.py &`
- `python3 camera_server.py &`
- The server serving TD is listening to port `3000`; Whilst the server communicating with the camera is listening to port `5000`

# requirements
- python >= 3.5
- picamera
- flask

# ESP8266 light sensor instructions
- prepare your ESP8266 with MicroPython's installation instruction
- change `esp_main.py` to `main.py`
- upload the script to ESP8266 with `ampy`
- now plug it in, find out its address from router's config page.
- You're set:)
