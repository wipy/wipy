#!/usr/bin/env python3

"""
Stub of the pyb module, useful to run the Blynk client using CPython.
"""

import time


"""
Some common utitlities
"""

try:
    float(1)
    floating_point = True
except:
    floating_point = False

start_time = time.time()
def pyblog(msg):
    if floating_point:
        print ("[{:7.3f}] {:}".format(float(time.time() - start_time), msg))
    else:
        print ("[{:}] {:}".format(time.time() - start_time, msg))


"""
Stub of the pyb.Pin class
"""
class Pin:

    # pin modes
    IN  = 'IN'
    OUT = 'OUT'

    # pin types
    STD     = 'STD'
    STD_PU  = 'STD_PU'
    STD_PD  = 'STD_PD'
    OD      = 'OD'
    OD_PU   = 'OD_PU'
    OD_PD   = 'OD_PD'

    # pin interrupt modes
    INT_FALLING         = 'INT_FALLING'
    INT_RISING          = 'INT_RISING'
    INT_RISING_FALLING  = 'INT_RISING_FALLING'
    INT_LOW_LEVEL       = 'INT_LOW_LEVEL'
    INT_HIGH_LEVEL      = 'INT_HIGH_LEVEL'

    # pin strength
    S2MA    = 'S2MA'
    S4MA    = 'S4MA'
    S6MA    = 'S6MA'

    def __init__(self, pin, af, mode=OUT, type=STD, strength=S4MA):
        self.pin = pin
        self.val = 0
        pyblog ('[Pin] Init %s with af=%s mode=%s type=%s strength=%s' % (pin, af, mode, type, strength))

    def value(self, val=None):
        if val is not None:
            self.val = val
            pyblog ('[Pin] %s value set to %d' % (self.pin, self.val))
        else:
            pyblog ('[Pin] %s value is %d' % (self.pin, self.val))
            return self.val


"""
Stub of the pyb.Timer class
"""
class Timer:

    A               = 'A'
    B               = 'B'
    ONE_SHOT        = 'ONE_SHOT'
    PERIODIC        = 'PERIODIC'
    EDGE_COUNT      = 'EDGE_COUNT'
    EDGE_TIME       = 'EDGE_TIME'
    PWM             = 'PWM'
    POSITIVE        = 'POSITIVE'
    NEGATIVE        = 'NEGATIVE'

    def __init__(self, timer, mode):
        self.timer = timer
        self.mode = mode
        pyblog ('[Timer] Init Timer %d with %s mode' % (timer, mode))

    def channel(self, channel, freq=None, period=None, polarity=POSITIVE, duty_cycle=0):
        if (freq is None and period is None) or (freq is not None and period is not None):
            raise ValueError('[Timer] Either frequency or period need to be given')
        self.channel = TimerChannel(channel, freq, period, polarity, duty_cycle)
        pyblog ('[Timer] New TimerChannel from Timer %d' % self.timer)
        return self.channel


class TimerChannel:

    def __init__(self, channel, freq=None, period=None, polarity=Timer.POSITIVE, duty_cycle=0):
        self.channel = channel
        self.freq = freq
        self.duty = duty_cycle
        pyblog ('[TimerChannel] New TimerChannel %s with freq=%d, period=%d, polarity=%s, duty_cycle=%d' % 
               (channel, freq if freq is not None else 0, period if period is not None else 0, polarity, duty_cycle))

    def duty_cycle(self, duty_cycle):
        self.duty = duty_cycle
        pyblog ('[TimerChannel] Channel %s duty cycle set to %d' % (self.channel, duty_cycle))


"""
Stub of the pyb.ADC class
"""
class ADC:

    def __init__(self, channel):
        self.channel = channel
        self.val = 1204
        pyblog ('[ADC] Init channel %d' % channel)

    def read(self):
        pyblog ('[ADC] Read channel %d value' % self.channel)
        return self.val


"""
Stub of the pyb.WDT class
"""
class WDT:

    def __init__(self, timeout):
        self.timeout = timeout
        pyblog ('[WDT] Init with %d timeout' % self.timeout)

    def kick(self):
        pyblog ('[WDT] Watchdog kicked just now')


"""
Stub of the pyb.HeartBeat class
"""
class HeartBeat:

    def __init__(self):
        pyblog ('[HeartBeat] Create instance')

    def enable(self):
        pyblog ('[HeartBeat] Enabled')

    def disable(self):
        pyblog ('[HeartBeat] Disabled')


"""
Stub of the pyb.Sleep class
"""
class Sleep:
    @staticmethod
    def idle():
        pass


"""
pyb functions
"""

def delay(msecs):
    if floating_point:
        time.sleep(msecs / 1000)
    else:
        time.sleep(msecs // 1000)

def millis():
    return int(time.time() * 1000)

def elapsed_millis(start):
    return millis() - start
