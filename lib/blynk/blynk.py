#!/usr/bin/python3

"""
Micro Python library that brings out-of-the-box Blynk support to 
the WiPy. Requires a previously established internet connection 
and a valid token string.

Example usage:

    import blynk
    blk = blynk.Blynk()

    # optionally define a virtual pin handler
    def vrHandler (pin_num, param_list=None):
        if param_list is None:
            # virtual read
        else:
            # virtual write
            for param in param_list:
                # do some stuff

    # register virtual pins 0, 1 and 2 with a common handler
    # (separate handlers could also be used if desired)
    blk.registerVirtualPin(0, vrHandler)
    blk.registerVirtualPin(1, vrHandler)
    blk.registerVirtualPin(2, vrHandler)

    # start Blynk (this method never returns)
    blk.run('e7c0a812347f45aebf6ae8403abcdefg')
"""

import socket
import time
import os
try:
    import pyb
except ImportError:
    import pybstub as pyb
    const = lambda x: x


# message types
MSG_RSP                     = const(0)
MSG_LOGIN                   = const(2)
MSG_PING                    = const(6)
MSG_TWEET                   = const(12)
MSG_EMAIL                   = const(13)
MSG_BRIDGE                  = const(15)
MSG_HW                      = const(20)


# message status codes
STATUS_SUCCESS              = const(200)
STATUS_QUOTA_LIMIT          = const(1)
STATUS_ILLEGAL_COMMAND      = const(2)
STATUS_NOT_REGISTERED       = const(3)
STATUS_ALREADY_REGISTERED   = const(4)
STATUS_NOT_AUTHENTICATED    = const(5)
STATUS_NOT_ALLOWED          = const(6)
STATUS_NO_CONNECTION        = const(7)
STATUS_NO_ACTIVE_DASHBOARD  = const(8)
STATUS_INVALID_TOKEN        = const(9)
STATUS_DEVICE_WENT_OFFLINE  = const(10)
STATUS_ALREADY_LOGGED_IN    = const(11)
STATUS_TIMEOUT              = const(16)

# heart beat period
HB_FREQUENCY                = const(10)

# timeouts
DEF_SOCK_TIMEOUT            = const(1)      # 1 second
AUT_SOCK_TIMEOUT            = const(5)      # 5 seconds
WDT_TIMEOUT                 = const(7000)   # 7 seconds

# virtual pins
MAX_VIRTUAL_PINS            = const(32)


def pack_header (type, id, ls):
    return b''.join([bytes([type]), bytes([(id >> 8) & 0xff]), bytes([id & 0xff]), 
                     bytes([(ls >> 8) & 0xff]), bytes([ls & 0xff])])


def unpack_header (hdr):
    return hdr[0], (int(hdr[1]) << 8) + hdr[2], (int(hdr[3]) << 8) + hdr[4]


class HwPin:

    __ADCPinMap   = {'GPIO2':  1, 'GPIO3':   2, 'GPIO4':   3, 'GPIO5':   4}
    __PWMPinMap   = {'GPIO9':  3, 'GPIO10':  3, 'GPIO11':  3, 'GPIO24':  5, 'GPIO25':  9}
    __TimerPinMap = {'GPIO9':  (3, pyb.Timer.B), 'GPIO10':  (4, pyb.Timer.A), 'GPIO11':  (4, pyb.Timer.B), 
                     'GPIO24': (1, pyb.Timer.A), 'GPIO25':  (2, pyb.Timer.A)}

    __HeartBeatPinNum = 25 if 'WiPy' in os.uname().machine else 9

    def __init__ (self, pin_num, mode, pull):
        # mode can be either b'in' or b'out'
        # pull can be one of b'pu, b'pd' or None
        # function can be one of 'dig', 'ana', 'pwm' or None
        self.mode = mode
        self.pull = pull
        self.function = ''
        self.pin = None
        self.adc = None
        self.timer = None
        self.pwm = None
        pin_num = int(pin_num)
        self.name = 'GPIO' + str(pin_num)
        if pin_num == HwPin.__HeartBeatPinNum:
            # disable the heart beat
            pyb.HeartBeat().disable()

    def __config(self, _duty_cycle=0):
        if self.function == 'dig':
            _mode = pyb.Pin.OUT if self.mode == b'out' else pyb.Pin.IN
            if self.pull == b'pu':
                _type = pyb.Pin.STD_PU
            elif self.pull == b'pd':
                _type = pyb.Pin.STD_PD
            else:
                _type = pyb.Pin.STD
            self.pin = pyb.Pin(self.name, af=0, mode=_mode, type=_type, strength=pyb.Pin.S6MA)
        elif self.function == 'ana':
            self.adc = pyb.ADC(HwPin.__ADCPinMap[self.name])
        else:   # 'pwm'
            self.pin = pyb.Pin(self.name, af=HwPin.__PWMPinMap[self.name], type=pyb.Pin.STD, strength=pyb.Pin.S6MA)
            self.timer = pyb.Timer(HwPin.__TimerPinMap[self.name][0], mode=pyb.Timer.PWM)
            self.pwm = self.timer.channel(HwPin.__TimerPinMap[self.name][1], freq=20000, duty_cycle=_duty_cycle)

    def digitalRead (self):
        if self.function != 'dig':
            self.function = 'dig'
            self.__config()
        return self.pin.value()

    def digitalWrite (self, value):
        if self.function != 'dig':
            self.function = 'dig'
            self.__config()
        self.pin.value(value)

    def analogRead (self):
        if self.function != 'ana':
            self.function = 'ana'
            self.__config()
        return self.adc.read()

    def analogWrite (self, value):
        if self.function != 'pwm':
            self.function = 'pwm'
            self.__config(value)
        else:
            self.pwm.duty_cycle(value)


class Blynk:

    def __init__ (self, with_wdt=True):
        self.hw_pins = {}
        self.vr_pins = {}
        self.tweets = []
        self.emails = []
        self.msg_id = 1
        self.logged_in = False
        self.pins_configured = False
        self.with_wdt = with_wdt
        self.timeout = None

    def __format_msg (self, msg_type, *args):
        # convert params to string and join using \0
        data = bytes('\0'.join(map(str, args)), 'ascii')
        # prepend the hw command header
        return pack_header(msg_type, self.__new_msg_id(), len(data)) + data

    def __handle_hw (self, data):
        params = data.split(b'\0')
        cmd = params.pop(0)
        if cmd == b'info':
            pass
        elif cmd == b'pm':
            pairs = zip(params[0::2], params[1::2])
            for (pin, mode) in pairs:
                print ("New pin config {:} {:}".format(pin, mode))
                if mode != b'in' and mode != b'out' and mode != b'pu' and mode != b'pd':
                    raise ValueError("Unknown pin %s mode: %s" % (pin, mode))
                self.hw_pins[pin] = HwPin(pin, mode, mode)
            self.pins_configured = True
        elif cmd == b'vw':
                pin = int(params.pop(0))
                if pin in self.vr_pins:
                    self.vr_pins[pin](pin, params)
                else:
                    print ("Warning: Virtual write to unregistered pin %d" % pin)
        elif cmd == b'vr':
                pin = int(params.pop(0))
                if pin in self.vr_pins:
                    val = self.vr_pins[pin](pin)
                else:
                    print ("Warning: Virtual read from unregistered pin %d" % pin)
                    val = 'Error'
                self.conn.send(self.__format_msg(MSG_HW, 'vw', pin, val))
        elif self.pins_configured:
            if cmd == b'dw':
                pin = params.pop(0)
                val = int(params.pop(0))
                self.hw_pins[pin].digitalWrite(val)
            elif cmd == b'aw':
                pin = params.pop(0)
                val = int(params.pop(0))
                self.hw_pins[pin].analogWrite(val)
            elif cmd == b'dr':
                pin = params.pop(0)
                val = self.hw_pins[pin].digitalRead()
                self.conn.send(self.__format_msg(MSG_HW, 'dw', pin, val))
            elif cmd == b'ar':
                pin = params.pop(0)
                val = self.hw_pins[pin].analogRead()
                self.conn.send(self.__format_msg(MSG_HW, 'aw', pin, val))
            else:
                raise ValueError("Unknown message cmd: %s" % cmd)

    def __new_msg_id(self):
        self.msg_id += 1
        return self.msg_id

    def __send_heartbeat(self):
        if self.with_wdt:
            self.wdt.kick()
        if time.time() - self.hb_time >= HB_FREQUENCY and self.logged_in:
            self.hb_time = time.time()
            self.conn.send(pack_header(MSG_PING, self.__new_msg_id(), 0))

    def __settimeout (self, timeout):
        if timeout is not None and timeout != self.timeout:
            self.timeout = timeout
            self.conn.settimeout(timeout)

    def __receive(self, length, timeout=None):
        rx_data = b''
        r_length = length
        self.__settimeout (timeout)
        while (True):
            self.__send_heartbeat()
            self.__send_tweets()
            self.__send_emails()
            try:
                rx_data += self.conn.recv(r_length)
            except socket.timeout:
                if timeout is not None:
                    return b''
            r_length = length - len(rx_data)
            if not r_length:
                return rx_data

    def __send_tweets (self):
        if self.tweets and self.logged_in:
            print ('Sending tweet')
            self.conn.send(self.__format_msg(MSG_TWEET, self.tweets.pop(0)))

    def __send_emails (self):
        if self.emails and self.logged_in:
            print ('Sending email')
            email = self.emails.pop(0)
            self.conn.send(self.__format_msg(MSG_EMAIL, email[0], email[1], email[2]))

    def tweet (self, msg):
        print ('Tweet appended')
        self.tweets.append(msg)

    def email (self, to, subject, body):
        print ('Email appended')
        self.emails.append((to, subject, body))

    def registerVirtualPin (self, pin, handler):
        if isinstance(pin, int) and pin in range (0, MAX_VIRTUAL_PINS):
            self.vr_pins[pin] = handler
        else:
            raise ValueError('Virtual pin must be an integer in the range (0, 32)')

    def run (self, token, server='cloud.blynk.cc', port=8442):
        if self.with_wdt:
            self.wdt = pyb.WDT(WDT_TIMEOUT)

        try:
            print ('Connecting to %s:%d' % (server, port))
            self.conn = socket.socket()
            self.conn.connect(socket.getaddrinfo(server, port)[0][4])
        except:
            print ('Connection to the Blynk servers failed!')
            raise

        # start the heart beat timer 
        self.hb_time = time.time()
        # authenticate
        hdr = pack_header(MSG_LOGIN, self.__new_msg_id(), len(token))
        if isinstance (token, str):
            token = bytes(token, 'ascii')
        self.conn.send(hdr + token)
        data = self.__receive(len(hdr), timeout=AUT_SOCK_TIMEOUT)
        if not data:
            raise TimeoutError ('Blynk authentication timed out')

        print ('Blynk connection successful, going to authenticate')
        msg_type, msg_id, status = unpack_header(data)
        if status != STATUS_SUCCESS or msg_id == 0:
            raise Exception('Blynk authentication failed!')

        # log-in succesful, set the default socket timeout
        print ('Access granted, happy Blynking!')
        self.logged_in = True
        self.__settimeout(DEF_SOCK_TIMEOUT)

        while (True):
            data = self.__receive(len(hdr))
            if data:
                msg_type, msg_id, msg_len = unpack_header(data)
                if msg_id == 0:
                    raise ValueError('Invalid msg_id %d' % msg_id)
                if msg_type == MSG_RSP:
                    pass
                elif msg_type == MSG_PING:
                    # got ping, send Pong
                    self.conn.send(pack_header(MSG_RSP, msg_id, STATUS_SUCCESS))
                elif msg_type == MSG_HW or msg_type == MSG_BRIDGE:
                    data = self.__receive(msg_len)
                    self.__handle_hw(data)
                else:
                    raise ValueError('Unknown message type %d' % msg_type)
