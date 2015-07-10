#!/usr/bin/env python3

"""
Micro Python library that brings out-of-the-box Blynk support to 
the WiPy. Requires a previously established internet connection 
and a valid token string.

Example usage:

    import blynk
    blk = blynk.Blynk('e7c0a812347f12345f6ae8403abcdefg')

    # to register virtual pins first define a handler
    def vrhandler (request):
        if request.type == blynk.VrRequest.READ:
            # do some stuff

    # create the virtual pin and register it
    vrpin = blynk.VrPin(0, vrhandler)
    blk.registerVrPin(vrpin)

    # a user task the will be called periodically can also
    # be registered
    def my_user_task():
        # do any non-blocking operations

    # register the task and specify the period which
    # must be a multiple of 10 ms
    blk.registerUserTask (my_user_task, period_multiple_of_10_ms)

    # start Blynk (this call should never return)
    blk.run()

The `request` object passed to the virtual handler contains the
following attributes:
    - `pin`: the pin number
    - `type`: can be either `VrRequest.READ` or `VrRequest.WRITE`
    - `args`: list of arguments passed to a virtual write, equals `None` 
           in the case of a virtual read.
"""

import socket
import struct
import time
import os
try:
    import pyb
except ImportError:
    import pybstub as pyb
    const = lambda x: x

HDR_LEN = const(5)
HDR_FMT = "!BHH"

BLYNK_MAX_MSGS_PER_SEC = const(20)

MSG_RSP = const(0)
MSG_LOGIN = const(2)
MSG_PING  = const(6)
MSG_TWEET = const(12)
MSG_EMAIL = const(13)
MSG_BRIDGE = const(15)
MSG_HW = const(20)

STA_SUCCESS = const(200)

HB_PERIOD = const(10)
NON_BLOCK_SOCKET = const(0)
MIN_SOCK_TIMEOUT = const(1) # 1 second
MAX_SOCK_TIMEOUT = const(5) # 5 seconds, must be < HB_PERIOD
WDT_TIMEOUT = const(10000) # 10 seconds
RECONNECT_DELAY = const(1) # 1 second
USER_TASK_PERIOD_RES = const(50) # 50 ms
IDLE_TIME_MS = const(5) # 5 ms

RETRANSMIT_DELAY = const(2)
MAX_TX_RETRIES = const(3)

MAX_VIRTUAL_PINS = const(32)

DISCONNECTED = 0
CONNECTING = 1
AUTHENTICATING = 2
AUTHENTICATED = 3

EAGAIN = const(11)

def sleep_from_until (start, delay):
    while pyb.elapsed_millis(start) < delay:
        pyb.Sleep.idle()
    return start + delay

class HwPin:
    __ADCMap = {'GPIO2': 1, 'GPIO3': 2, 'GPIO4': 3, 'GPIO5': 4}
    __PWMMap = {'GPIO9': 3, 'GPIO10': 3, 'GPIO11': 3, 'GPIO24': 5, 'GPIO25': 9}
    __TimerMap = {'GPIO9': (3, pyb.Timer.B), 'GPIO10': (4, pyb.Timer.A), 'GPIO11': (4, pyb.Timer.B), 
                  'GPIO24': (1, pyb.Timer.A), 'GPIO25': (2, pyb.Timer.A)}

    __HBPin = 25 if 'WiPy' in os.uname().machine else 9

    def __init__(self, pin_num, mode, pull):
        self.__mode = mode
        self.__pull = pull
        self.__function = ''
        self.__pin = None
        self.__adc = None
        self.__pwm = None
        pin_num = int(pin_num)
        self.__name = 'GPIO' + str(pin_num)
        if pin_num == HwPin.__HBPin:
            pyb.HeartBeat().disable()

    def __config(self, _duty_cycle=0):
        if self.__function == 'dig':
            _mode = pyb.Pin.OUT if self.__mode == b'out' else pyb.Pin.IN
            if self.__pull == b'pu':
                _type = pyb.Pin.STD_PU
            elif self.__pull == b'pd':
                _type = pyb.Pin.STD_PD
            else:
                _type = pyb.Pin.STD
            self.__pin = pyb.Pin(self.__name, af=0, mode=_mode, type=_type, strength=pyb.Pin.S6MA)
        elif self.__function == 'ana':
            self.__adc = pyb.ADC(HwPin.__ADCMap[self.__name])
        else:
            pyb.Pin(self.__name, af=HwPin.__PWMMap[self.__name], type=pyb.Pin.STD, strength=pyb.Pin.S6MA)
            timer = pyb.Timer(HwPin.__TimerMap[self.__name][0], mode=pyb.Timer.PWM)
            self.__pwm = timer.channel(HwPin.__TimerMap[self.__name][1], freq=20000, duty_cycle=_duty_cycle)

    def digitalRead(self):
        if self.__function != 'dig':
            self.__function = 'dig'
            self.__config()
        return self.__pin.value()

    def digitalWrite(self, value):
        if self.__function != 'dig':
            self.__function = 'dig'
            self.__config()
        self.__pin.value(value)

    def analogRead(self):
        if self.__function != 'ana':
            self.__function = 'ana'
            self.__config()
        return self.__adc.read()

    def analogWrite(self, value):
        if self.__function != 'pwm':
            self.__function = 'pwm'
            self.__config(value)
        else:
            self.__pwm.duty_cycle(value)

class VrRequest:
    READ = 0
    WRITE = 1

    def __init__(self, pin, type, args=None):
        self.pin = pin
        self.type = type
        self.args = args


class VrPin:
    def __init__(self, pin, handler):
        if isinstance(pin, int) and pin in range(0, MAX_VIRTUAL_PINS):
            self.pin = pin
            self.__handler = handler
        else:
            raise ValueError('the pin must be an integer between 0 and %d' % (MAX_VIRTUAL_PINS - 1))

    def request(self, request):
        if request.pin == self.pin:
            if self.__handler:
                return self.__handler(request)
        else:
            raise ValueError('virtual {:} on pin {:} with pin {:} request'.
                             format('read' if request.type == VrRequest.READ else 'write', request.pin, self.pin))

class Blynk:
    def __init__(self, token, server='cloud.blynk.cc', port=8442, connect=True, enable_wdt=True):
        self.__wdt = None
        self.__vr_pins = {}
        self.__connect = False
        self.__user_task = None
        self.__user_task_period = 0
        self.__token = token
        if isinstance (self.__token, str):
            self.__token = bytes(token, 'ascii')
        self.__server = server
        self.__port = port
        self.__connect = connect
        self.__wdt = enable_wdt
        self.state = DISCONNECTED

    def __format_msg(self, msg_type, *args):
        data = bytes('\0'.join(map(str, args)), 'ascii')
        return struct.pack(HDR_FMT, msg_type, self.__new_msg_id(), len(data)) + data

    def __handle_hw(self, data):
        params = data.split(b'\0')
        cmd = params.pop(0)
        if cmd == b'info':
            pass
        elif cmd == b'pm':
            pairs = zip(params[0::2], params[1::2])
            for (pin, mode) in pairs:
                if mode != b'in' and mode != b'out' and mode != b'pu' and mode != b'pd':
                    raise ValueError("Unknown pin %s mode: %s" % (pin, mode))
                self.__hw_pins[pin] = HwPin(pin, mode, mode)
            self.__pins_configured = True
        elif cmd == b'vw':
                pin = int(params.pop(0))
                if pin in self.__vr_pins:
                    self.__vr_pins[pin].request(VrRequest(pin, VrRequest.WRITE, params))
                else:
                    print("Warning: Virtual write to unregistered pin %d" % pin)
        elif cmd == b'vr':
                pin = int(params.pop(0))
                if pin in self.__vr_pins:
                    val = self.__vr_pins[pin].request(VrRequest(pin, VrRequest.READ))
                else:
                    print("Warning: Virtual read from unregistered pin %d" % pin)
                    val = 'Error'
                self.__send(self.__format_msg(MSG_HW, 'vw', pin, val))
        elif self.__pins_configured:
            if cmd == b'dw':
                pin = params.pop(0)
                val = int(params.pop(0))
                self.__hw_pins[pin].digitalWrite(val)
            elif cmd == b'aw':
                pin = params.pop(0)
                val = int(params.pop(0))
                self.__hw_pins[pin].analogWrite(val)
            elif cmd == b'dr':
                pin = params.pop(0)
                val = self.__hw_pins[pin].digitalRead()
                self.__send(self.__format_msg(MSG_HW, 'dw', pin, val))
            elif cmd == b'ar':
                pin = params.pop(0)
                val = self.__hw_pins[pin].analogRead()
                self.__send(self.__format_msg(MSG_HW, 'aw', pin, val))
            else:
                raise ValueError("Unknown message cmd: %s" % cmd)

    def __new_msg_id(self):
        self.__msg_id += 1
        if (self.__msg_id > 0xFFFF):
            self.__msg_id = 1
        return self.__msg_id

    def __settimeout(self, timeout):
        if timeout != self.__timeout:
            self.__timeout = timeout
            self.conn.settimeout(timeout)

    def __receive(self, length, timeout=0):
        self.__settimeout (timeout)
        try:
            self.__rx_data += self.conn.recv(length)
        except socket.timeout:
            return b''
        except socket.error as e:
            if e.args[0] ==  EAGAIN:
                return b''
            else:
                raise
        if len(self.__rx_data) >= length:
            data = self.__rx_data[:length]
            self.__rx_data = self.__rx_data[length:]
            return data
        else:
            return b''

    def __send(self, data, send_anyway = False):
        if self.__tx_count < BLYNK_MAX_MSGS_PER_SEC or send_anyway:
            retries = 0
            while retries <= MAX_TX_RETRIES:
                try:
                    self.conn.send(data)
                    self.__tx_count += 1
                    break
                except socket.error as er:
                    if er.args[0] != EAGAIN:
                        raise
                    else:
                        pyb.delay(RETRANSMIT_DELAY)
                        retries += 1

    def __close(self, emsg=None):
        self.conn.close()
        self.state = DISCONNECTED
        time.sleep (RECONNECT_DELAY)
        if emsg:
            print('Error: %s, connection closed' % emsg)

    def __server_alive(self):
        c_time = int(time.time())
        if self.__m_time != c_time:
            self.__m_time = c_time
            self.__tx_count = 0
            if self.__wdt:
                self.__wdt.kick()
            if self.__last_hb_id != 0 and c_time - self.__hb_time >= MAX_SOCK_TIMEOUT:
                return False
            if c_time - self.__hb_time >= HB_PERIOD and self.state == AUTHENTICATED:
                self.__hb_time = c_time
                self.__last_hb_id = self.__new_msg_id()
                self.__send(struct.pack(HDR_FMT, MSG_PING, self.__last_hb_id, 0), True)
        return True

    def __run_user_task(self):
        if self.__user_task:
            c_millis = pyb.millis()
            if c_millis - self.__user_task_millis >= self.__user_task_period:
                self.__user_task_millis += self.__user_task_period
                self.__user_task()

    def tweet(self, msg):
        if self.state == AUTHENTICATED:
            self.__send(self.__format_msg(MSG_TWEET, msg))

    def email(self, to, subject, body):
        if self.state == AUTHENTICATED:
            self.__send(self.__format_msg(MSG_EMAIL, to, subject, body))

    def registerVrPin(self, pin):
        self.__vr_pins[pin.pin] = pin

    def registerUserTask(self, task, ms_period):
        if ms_period % USER_TASK_PERIOD_RES != 0:
            raise ValueError('the user task period must be a multiple of %d ms' % USER_TASK_PERIOD_RES)
        self.__user_task = task
        self.__user_task_period = ms_period

    def connect(self):
        self.__connect = True

    def disconnect(self):
        self.__connect = False

    def run(self):
        self.__start_time = pyb.millis()
        self.__user_task_millis = self.__start_time
        self.__hw_pins = {}
        self.__rx_data = b''
        self.__msg_id = 1
        self.__pins_configured = False
        self.__timeout = None
        self.__tx_count = 0
        self.__m_time = 0
        self.state = DISCONNECTED

        if self.__wdt:
            self.__wdt = pyb.WDT(WDT_TIMEOUT)

        while True:
            while self.state != AUTHENTICATED:
                self.__run_user_task()
                if self.__wdt:
                    self.__wdt.kick()
                if self.__connect:
                    try:
                        print('Connecting to %s:%d' % (self.__server, self.__port))
                        self.conn = socket.socket()
                        self.state = CONNECTING
                        self.conn.connect(socket.getaddrinfo(self.__server, self.__port)[0][4])
                    except:
                        self.__close('connection with the Blynk servers failed')
                        continue

                    self.state = AUTHENTICATING
                    hdr = struct.pack(HDR_FMT, MSG_LOGIN, self.__new_msg_id(), len(self.__token))
                    print('Blynk connection successful, authenticating...')
                    self.__send(hdr + self.__token, True)
                    data = self.__receive(HDR_LEN, timeout=MAX_SOCK_TIMEOUT)
                    if not data:
                        self.__close('Blynk authentication timed out')
                        continue

                    msg_type, msg_id, status = struct.unpack(HDR_FMT, data)
                    if status != STA_SUCCESS or msg_id == 0:
                        self.__close('Blynk authentication failed')
                        continue

                    self.state = AUTHENTICATED
                    print('Access granted, happy Blynking!')
                else:
                    self.__start_time = sleep_from_until(self.__start_time, USER_TASK_PERIOD_RES)

            self.__hb_time = 0
            self.__last_hb_id = 0
            self.__tx_count = 0
            while self.__connect:
                data = self.__receive(HDR_LEN, NON_BLOCK_SOCKET)
                if data:
                    msg_type, msg_id, msg_len = struct.unpack(HDR_FMT, data)
                    if msg_id == 0:
                        self.__close('invalid msg id %d' % msg_id)
                        break
                    if msg_type == MSG_RSP:
                        if msg_id == self.__last_hb_id:
                            self.__last_hb_id = 0
                    elif msg_type == MSG_PING:
                        self.__send(struct.pack(HDR_FMT, MSG_RSP, msg_id, STA_SUCCESS), True)
                    elif msg_type == MSG_HW or msg_type == MSG_BRIDGE:
                        data = self.__receive(msg_len, MIN_SOCK_TIMEOUT)
                        if data:
                            self.__handle_hw(data)
                    else:
                        self.__close('unknown message type %d' % msg_type)
                        break
                else:
                    self.__start_time = sleep_from_until(self.__start_time, IDLE_TIME_MS)
                if not self.__server_alive():
                    self.__close('Blynk server is offline')
                    break
                self.__run_user_task()

            if not self.__connect:
                self.__close()
                print('Blynk disconnection requested by the user')
