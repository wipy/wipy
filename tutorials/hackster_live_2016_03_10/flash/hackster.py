#!/usr/bin/env python3

"""
This example shows how to create a dashboard to monitor temperature
and ambient light with a gauge widget. It also duplicates the
MicroPython REPL on a terminal widget (allowing on the fly hacking :-))

In your Blynk App project:
  Add a gauge widget, bind it to Virtual Pin V0.
  Add a second gauge widget, bind it to Virtual Pin V1.
  Add a Terminal widget, bind it to Virtual Pin V2.
  Run the App (green triangle in the upper right corner).

Don't forget to change WIFI_SSID, WIFI_AUTH and BLYNK_AUTH ;)
"""

import config
import onewire
import BlynkLib
import machine
import os
from machine import Pin
from machine import RTC
from machine import I2C
from network import WLAN
from bh1750fvi import BH1750FVI

WIFI_SSID  = config.ssid
WIFI_AUTH  = config.auth
BLYNK_AUTH = config.token

MAIN_TASK_PERIOD = const(50)


def connect_to_wlan(wlan):
    # try connecting to wifi until succeeding
    while True:
        try:
            wlan.connect(WIFI_SSID, auth=WIFI_AUTH, timeout=7500)
            while not wlan.isconnected():
                machine.idle()
            return
        except OSError:
            pass


class MainTask:
    BH1750FVI_ADDR = const(35)

    def __init__(self, blynk, ow_pin, i2c_pins, period):
        self.blynk = blynk
        self.ds18b20 = onewire.DS18X20(onewire.OneWire(Pin(ow_pin)))
        i2c = I2C(baudrate=100000, pins=i2c_pins)
        self.bh1750fvi = BH1750FVI(i2c, BH1750FVI_ADDR, period)
        self.ds_state = 'CONV'
        self.lux = 0
        self.enabled = True

    def run(self):
        if self.enabled:
            if self.ds_state == 'CONV':
                self.ds18b20.start_convertion(self.ds18b20.roms[0])
                self.ds_state = 'READ'
            elif self.ds_state == 'READ':
                tmp = self.ds18b20.read_temp_async(self.ds18b20.roms[0])
                if tmp != None:
                    self.blynk.virtual_write(0, '{:02d}.{:02d}'.format(tmp // 100, tmp % 100))
                    self.ds_state = 'CONV'
    
            lux = self.bh1750fvi.read()
            if self.lux != lux:
                self.lux = lux
                self.blynk.virtual_write(1, self.lux)


# change to STA mode and connect to the configured network
wlan = WLAN(mode=WLAN.STA)
connect_to_wlan(wlan)

# set the current time (mandatory to validate certificates)
RTC(datetime=(2016, 3, 9, 19, 30, 0, 0, None))

blynk = BlynkLib.Blynk(BLYNK_AUTH, wdt=True, ssl=True) # initialize Blynk with SSL enabled

# create the user task, pin GP30 for the one wire sensor, GP11 and GP12 as SDA and SCL
s_task = MainTask(blynk, 'GP30', ('GP11', 'GP12'), MAIN_TASK_PERIOD)
blynk.set_user_task(s_task.run, MAIN_TASK_PERIOD)

term = blynk.repl(2) # register the terminal REPL on v2
os.dupterm(term)

blynk.run()
