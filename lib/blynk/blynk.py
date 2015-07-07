#!/usr/bin/env python3

"""
Micro Python library that brings out-of-the-box Blynk support to 
the WiPy. Requires a previously established internet connection 
and a valid token string.

Example usage:

    import blynk
    blk = blynk.Blynk()

    # to register virtual pins first define a handler
    def vrhandler (request):
        if request.type == blynk.VirtualRequest.READ:
            # do some stuff

    # create the virtual pin and register it
    vrpin = blynk.VirtualPin(0, vrhandler)
    blk.registerVirtualPin(vrpin)

    # a user task the will be called periodically can also
    # be registered
    def my_user_task():
        # do any non-blocking operations

    # register the task and specify the period which
    # must be a multiple of 10 ms
    blk.registerUserTask (my_user_task, period_multiple_of_10_ms)

    # start Blynk (this call should never return)
    blk.run('e7c0a812347f12345f6ae8403abcdefg')

The `request` object passed to the virtual handler contains the
following attributes:
    - `pin`: the pin number
    - `type`: can be either `VirtualRequest.READ` or `VirtualRequest.WRITE`
    - `args`: list of arguments passed to a virtual write, equals `None` 
           in the case of a virtual read.
"""

import socket
import time
import os
try:
    import pyb
except ImportError:
    import pybstub as pyb
    const = lambda x: x

# Blynk header length
HDR_LEN = const(5)

# message types
MSG_RSP = const(0)
MSG_LOGIN = const(2)
MSG_PING  = const(6)
MSG_TWEET = const(12)
MSG_EMAIL = const(13)
MSG_BRIDGE = const(15)
MSG_HW = const(20)

# message status codes
STATUS_SUCCESS = const(200)
STATUS_QUOTA_LIMIT = const(1)
STATUS_ILLEGAL_COMMAND = const(2)
STATUS_NOT_REGISTERED = const(3)
STATUS_ALREADY_REGISTERED = const(4)
STATUS_NOT_AUTHENTICATED = const(5)
STATUS_NOT_ALLOWED = const(6)
STATUS_NO_CONNECTION = const(7)
STATUS_NO_ACTIVE_DASHBOARD = const(8)
STATUS_INVALID_TOKEN = const(9)
STATUS_DEVICE_WENT_OFFLINE = const(10)
STATUS_ALREADY_LOGGED_IN = const(11)
STATUS_TIMEOUT = const(16)

# heart beat period
HB_FREQUENCY = const(10)

# time constants
NON_BLOCK_SOCKET = const(0)
MIN_SOCK_TIMEOUT = const(1) # 1 second
MAX_SOCK_TIMEOUT = const(5) # 5 seconds
WDT_TIMEOUT = const(9000) # 9 seconds
RECONNECT_DELAY = const(1) # 1 second
USER_TASK_PERIOD_MS_RES = const(50) # 50 ms
IDLE_TIME_MS = const(5) # 5 ms

# virtual pins
MAX_VIRTUAL_PINS = const(32)

# Blynk app state
DISCONNECTED = 0
CONNECTING = 1
AUTHENTICATING = 2
AUTHENTICATED = 3

# socket errno (define 'em here to save RAM)
EAGAIN = const(11)


def pack_header(type, id, ls):
    return b''.join([bytes([type]), bytes([(id >> 8) & 0xff]), bytes([id & 0xff]), 
                     bytes([(ls >> 8) & 0xff]), bytes([ls & 0xff])])


def unpack_header(hdr):
    return hdr[0], (int(hdr[1]) << 8) + hdr[2], (int(hdr[3]) << 8) + hdr[4]


def sleep_from_until (start, delay):
    while pyb.elapsed_millis(start) < delay:
        pyb.Sleep.idle()
    return start + delay


class HardwarePin:

    __ADCPinMap = {'GPIO2': 1, 'GPIO3': 2, 'GPIO4': 3, 'GPIO5': 4}
    __PWMPinMap = {'GPIO9': 3, 'GPIO10': 3, 'GPIO11': 3, 'GPIO24': 5, 'GPIO25': 9}
    __TimerPinMap = {'GPIO9': (3, pyb.Timer.B), 'GPIO10': (4, pyb.Timer.A), 'GPIO11': (4, pyb.Timer.B), 
                     'GPIO24': (1, pyb.Timer.A), 'GPIO25': (2, pyb.Timer.A)}

    __HeartBeatPinNum = 25 if 'WiPy' in os.uname().machine else 9

    def __init__(self, pin_num, mode, pull):
        # __mode can be either b'in' or b'out'
        # __pull can be one of b'pu, b'pd' or None
        # __function can be one of 'dig', 'ana', 'pwm' or None
        self.__mode = mode
        self.__pull = pull
        self.__function = ''
        self.__pin = None
        self.__adc = None
        self.__timer = None
        self.__pwm = None
        pin_num = int(pin_num)
        self.__name = 'GPIO' + str(pin_num)
        if pin_num == HardwarePin.__HeartBeatPinNum:
            # disable the heart beat
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
            self.__adc = pyb.ADC(HardwarePin.__ADCPinMap[self.__name])
        else:   # 'pwm'
            self.__pin = pyb.Pin(self.__name, af=HardwarePin.__PWMPinMap[self.__name], type=pyb.Pin.STD, strength=pyb.Pin.S6MA)
            self.__timer = pyb.Timer(HardwarePin.__TimerPinMap[self.__name][0], mode=pyb.Timer.PWM)
            self.__pwm = self.__timer.channel(HardwarePin.__TimerPinMap[self.__name][1], freq=20000, duty_cycle=_duty_cycle)

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


class VirtualRequest:

    READ = 0
    WRITE = 1

    def __init__(self, pin, type, args=None):
        self.pin = pin
        self.type = type
        self.args = args


class VirtualPin:

    def __init__(self, pin, handler):
        if isinstance(pin, int) and pin in range(0, MAX_VIRTUAL_PINS):
            self.pin = pin
            self.__handler = handler
        else:
            raise ValueError('virtual pin must be an integer between 0 and %d' % (MAX_VIRTUAL_PINS - 1))

    def request (self, request):
        if request.pin == self.pin:
            if self.__handler:
                return self.__handler(request)
        else:
            raise ValueError('virtual {:} on pin {:} with pin {:} request'.
                             format('read' if request.type == VirtualRequest.READ else 'write', request.pin, self.pin))


class Blynk:

    def __init__(self):
        self.__wdt = None
        self.__vr_pins = {}
        self.__connect = False
        self.__user_task = None
        self.__user_task_period = 0
        self.state = DISCONNECTED

    def __format_msg(self, msg_type, *args):
        # convert params to string and join using \0
        data = bytes('\0'.join(map(str, args)), 'ascii')
        # prepend the msg header
        return pack_header(msg_type, self.__new_msg_id(), len(data)) + data

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
                self.__hw_pins[pin] = HardwarePin(pin, mode, mode)
            self.__pins_configured = True
        elif cmd == b'vw':
                pin = int(params.pop(0))
                if pin in self.__vr_pins:
                    self.__vr_pins[pin].request(VirtualRequest(pin, VirtualRequest.WRITE, params))
                else:
                    print("Warning: Virtual write to unregistered pin %d" % pin)
        elif cmd == b'vr':
                pin = int(params.pop(0))
                if pin in self.__vr_pins:
                    val = self.__vr_pins[pin].request(VirtualRequest(pin, VirtualRequest.READ))
                else:
                    print("Warning: Virtual read from unregistered pin %d" % pin)
                    val = 'Error'
                self.conn.send(self.__format_msg(MSG_HW, 'vw', pin, val))
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
                self.conn.send(self.__format_msg(MSG_HW, 'dw', pin, val))
            elif cmd == b'ar':
                pin = params.pop(0)
                val = self.__hw_pins[pin].analogRead()
                self.conn.send(self.__format_msg(MSG_HW, 'aw', pin, val))
            else:
                raise ValueError("Unknown message cmd: %s" % cmd)

    def __new_msg_id(self):
        self.__msg_id += 1
        # msg id 0 is not allowed
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

    def __send(self, data):
        if self.__tx_count < BLYNK_MAX_MSG_PER_SEC:
            self.__tx_count += 1
            while (True):
                try:
                    self.conn.send(data)
                    break
                except socket.error as er:
                    if er.args[0] != EAGAIN:
                        raise

    def __close_connection(self, emsg=None):
        self.conn.close()
        self.state = DISCONNECTED
        time.sleep (RECONNECT_DELAY)
        if emsg:
            print('Error: %s, connection closed' % emsg)

    def __server_is_alive(self):
        # get the current time
        c_time = int(time.time())
        if self.__m_time != c_time:
            # reset the flooding avoidance variables
            self.__m_time = c_time
            self.__tx_count = 0
            # kick the watchdog
            if self.__wdt:
                self.__wdt.kick()
        # send a new heart beat
        if c_time - self.__hb_time >= HB_FREQUENCY and self.state == AUTHENTICATED:
            self.__hb_time = c_time
            self.__last_hb_id = self.__new_msg_id()
            self.conn.send(pack_header(MSG_PING, self.__last_hb_id, 0))
        # verify that the server is responding our heart beat messages
        if self.__last_hb_id != 0 and c_time - self.__hb_time > MAX_SOCK_TIMEOUT:
            return False
        return True

    def __run_user_task(self):
        if self.__user_task:
            c_millis = pyb.millis()
            if c_millis - self.__user_task_millis >= self.__user_task_period:
                # add self.__user_task_period to compensate for drifting
                self.__user_task_millis += self.__user_task_period
                self.__user_task()

    def tweet(self, msg):
        if self.state == AUTHENTICATED:
            self.conn.send(self.__format_msg(MSG_TWEET, msg))

    def email(self, to, subject, body):
        if self.state == AUTHENTICATED:
            self.conn.send(self.__format_msg(MSG_EMAIL, to, subject, body))

    def registerVirtualPin(self, pin):
        self.__vr_pins[pin.pin] = pin

    def registerUserTask(self, task, ms_period):
        if ms_period % USER_TASK_PERIOD_MS_RES != 0:
            raise ValueError('the user task period must be a multiple of %d ms' % USER_TASK_PERIOD_MS_RES)
        self.__user_task = task
        self.__user_task_period = ms_period

    def connect(self):
        self.__connect = True

    def disconnect(self):
        self.__connect = False

    def run(self, token, server='cloud.blynk.cc', port=8442, connect=True, enable_wdt=True):
        self.__start_time = pyb.millis()
        self.__connect = connect
        self.__hw_pins = {}
        self.__rx_data = b''
        self.__msg_id = 1
        self.__pins_configured = False
        self.__timeout = None
        self.__tx_count = 0 # counter to avoid flooding
        self.__m_time = 0 # time mark to avoid flooding
        self.__hb_time = 0
        self.__last_hb_id = 0
        self.__user_task_millis = pyb.millis()
        self.state = DISCONNECTED

        if enable_wdt:
            self.__wdt = pyb.WDT(WDT_TIMEOUT)

        while True:
            while self.state != AUTHENTICATED:
                self.__run_user_task()
                if self.__wdt:
                    self.__wdt.kick()
                if self.__connect:
                    try:
                        print('Connecting to %s:%d' % (server, port))
                        self.conn = socket.socket()
                        self.state = CONNECTING
                        self.conn.connect(socket.getaddrinfo(server, port)[0][4])
                    except:
                        self.__close_connection('connection to the Blynk servers failed!')
                        continue

                    # authenticate
                    self.state = AUTHENTICATING
                    hdr = pack_header(MSG_LOGIN, self.__new_msg_id(), len(token))
                    if isinstance (token, str):
                        token = bytes(token, 'ascii')
                    print('Blynk connection successful, authenticating...')
                    self.conn.send(hdr + token)
                    data = self.__receive(HDR_LEN, timeout=MAX_SOCK_TIMEOUT)
                    if not data:
                        self.__close_connection('Blynk authentication timed out')
                        continue

                    msg_type, msg_id, status = unpack_header(data)
                    if status != STATUS_SUCCESS or msg_id == 0:
                        self.__close_connection('Blynk authentication failed!')
                        continue

                    # log-in successful
                    self.state = AUTHENTICATED
                    print('Access granted, happy Blynking!')
                else:
                    self.__start_time = sleep_from_until(self.__start_time, USER_TASK_PERIOD_MS_RES)

            while self.__connect:
                data = self.__receive(HDR_LEN, NON_BLOCK_SOCKET)
                if data:
                    msg_type, msg_id, msg_len = unpack_header(data)
                    if msg_id == 0:
                        self.__close_connection('invalid msg id %d' % msg_id)
                        break
                    if msg_type == MSG_RSP:
                        if msg_id == self.__last_hb_id:
                            # heart beat response received
                            self.__last_hb_id = 0
                    elif msg_type == MSG_PING:
                        # got ping, send pong
                        self.conn.send(pack_header(MSG_RSP, msg_id, STATUS_SUCCESS))
                    elif msg_type == MSG_HW or msg_type == MSG_BRIDGE:
                        data = self.__receive(msg_len, MIN_SOCK_TIMEOUT)
                        if data:
                            self.__handle_hw(data)
                    else:
                        self.__close_connection('unknown message type %d' % msg_type)
                        break
                else:
                    self.__start_time = sleep_from_until(self.__start_time, IDLE_TIME_MS)
                if not self.__server_is_alive():
                    self.__close_connection('Blynk server is offline')
                    break
                self.__run_user_task()

            self.__close_connection()
            print('Blynk disconnection requested by the user')
