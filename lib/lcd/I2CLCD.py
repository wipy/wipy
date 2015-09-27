from machine import I2C
from machine import Pin

class I2CLCD:
    def __init__ (self, i2c=0, addr=60, backlight_pin='GP9'):
        self.lines = 2
        self.columms = 20
        self.addr = addr
        self.bl_pin = backlight_pin
        if backlight_pin:
            if isinstance(backlight_pin, Pin):
                self.bl_pin(0)
            else:
                self.bl_pin = Pin(backlight_pin, mode=Pin.OUT, value=0)
        if isinstance(i2c, I2C):
            self.i2c = i2c
        else:
            self.i2c = I2C(i2c, I2C.MASTER, baudrate=100000, pins=('GP15', 'GP10'))
        self.i2c.writeto(self.addr, bytes([0x80, 0x38, 0x00, 0x39, 0x14, 0x74, 0x54, 0x6F, 0x0C, 0x01]))

    def write (self, line, col, text):
        data = bytearray([0x00, 0x80])
        if col > 20:
            col = 20
        data[1] |= ((line - 1) * 0x40) + (col - 1)
        self.i2c.writeto(self.addr, data) # set the line and col
        self.i2c.writeto(self.addr, bytearray([0x40]) + bytearray(text)) # write the text

    def backlight(self, on):
        if self.bl_pin:
            self.bl_pin(on)

    def cursor(self, on):
        if on:
            data = bytes([int('00001111', 2)])
        else:
            data = bytes([int('00001100', 2)])
        self.i2c.writeto(self.addr, b'\x00' + data)
