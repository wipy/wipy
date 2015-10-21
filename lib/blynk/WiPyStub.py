#!/usr/bin/env python3

"""
Stub of the wipy module, useful to run the Blynk client using CPython.
"""

def heartbeat(enable):
    if (enable):
        print('Heart beat LED enabled')
    else:
        print('Heart beat LED disabled')