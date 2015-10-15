"""
Blynk is a platform with iOS and Android apps to control
Arduino, Raspberry Pi and the likes over the Internet.
You can easily build graphic interfaces for all your
projects by simply dragging and dropping widgets.

  Downloads, docs, tutorials: http://www.blynk.cc
  Blynk community:            http://community.blynk.cc
  Social networks:            http://www.fb.com/blynkapp
                              http://twitter.com/blynk_app

This example shows how to handle a button press and
send Twitter & Push notifications.

In your Blynk App project:
  Add a Button widget, bind it to Virtual Pin V4.
  Add a Twitter widget and connect it to your account.
  Add a Push notification widget.
  Run the App (green triangle in the upper right corner).

Don't forget to change WIFI_SSID, WIFI_AUTH and BLYNK_AUTH ;)
"""

import BlynkLib
from network import WLAN
import time

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

# to register virtual pins first define a handler
def v4_write_handler(value):
    if value: # is the the button is pressed?
        blynk.notify('You pressed the button and I know it ;)')
        blynk.tweet('My WiPy project is tweeting using @blynk_app and it’s awesome! #IoT #blynk @wipyio @micropython')

# attach virtual pin 4 to our handler
blynk.add_virtual_pin(4, write=v4_write_handler)

# start Blynk (this call should never return)
blynk.run()
