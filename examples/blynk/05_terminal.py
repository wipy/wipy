"""
Blynk is a platform with iOS and Android apps to control
Arduino, Raspberry Pi and the likes over the Internet.
You can easily build graphic interfaces for all your
projects by simply dragging and dropping widgets.

  Downloads, docs, tutorials: http://www.blynk.cc
  Blynk community:            http://community.blynk.cc
  Social networks:            http://www.fb.com/blynkapp
                              http://twitter.com/blynk_app

This example shows how to add a custom terminal widget.

In your Blynk App project:
  Add a Terminal widget, bind it to Virtual Pin V3.
  Run the App (green triangle in the upper right corner).

Don't forget to change WIFI_SSID, WIFI_AUTH and BLYNK_AUTH ;)
"""

import BlynkLib
from network import WLAN

WIFI_SSID  = 'YOUR_WIFI'
WIFI_AUTH  = (WLAN.WPA2, 'YOUR_PASS')
BLYNK_AUTH = 'YOUR_AUTH_TOKEN'

# connect to WiFi
wifi = WLAN(mode=WLAN.STA)
wifi.connect(WIFI_SSID, auth=WIFI_AUTH)
while not wifi.isconnected():
    pass

print('IP address:', wifi.ifconfig()[0])

# initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH)

def v3_write_handler(value):
    # execute the command echo it back
    blynk.virtual_write(3, 'Command: ' + value + '\n')
    blynk.virtual_write(3, 'Result: ')
    try:
        blynk.virtual_write(3, str(eval(value)))
    except:
        try:
            exec(value)
        except Exception as e:
            blynk.virtual_write(3, 'Exception:\n  ' + repr(e))
    finally:
        blynk.virtual_write(3, '\n')

def v3_read_handler(value):
    pass

# attach virtual pin 3 to our handlers
blynk.add_virtual_pin(3, v3_read_handler, v3_write_handler)

# start Blynk (this call should never return)
blynk.run()
