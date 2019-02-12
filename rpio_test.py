#!/usr/bin/python3

"""
MIT LICENSE

Copyright (c) 2018-2019 David A. Krause, aka papamac

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

DESCRIPTION

"""
__author__ = 'papamac'
__version__ = '0.9.0'
__date__ = 'April 12, 2018'

from threading import Thread
from time import sleep

from rpio import RPIO
from rpio_global import REQUEST_LIST


class RPIOTest(Thread):

    # Private instance methods:

    def __init__(self):
        self.rpio = RPIO()
        self.output_channels = (12, 13, 14, 15, 28, 29, 30, 31)
        for channel in self.output_channels:
            self.rpio.configure(channel, 0)
        Thread.__init__(self, name='RPIOTest', target=self._rpio_test)
        self.running = False

    def _rpio_test(self):
        state = False
        while self.running:
            for channel in range(32):
                self.rpio.read(channel)
            sleep(2)
            for channel in self.output_channels:
                self.rpio.write(channel, int(state))
                state = not state
                if not self.running:
                    break
            state = not state
            self.rpio.read(101)
            sleep(10)

        # Public instance methods:

    def start(self):
        self.rpio.start()
        self.running = True
        Thread.start(self)

    def stop(self):
        self.running = False
        self.join()
        self.rpio.stop()


# RPIO Test main program:

# Requires the port definitions in "raspi3bp-ha_port_defs.xml"

if __name__ == '__main__':
    RPIO.enable_printing()
    print('\nBegin RPIO Interactive Test (RPIO instance 1)')
    rpio = RPIO()
    if rpio.init_error:
        print(rpio.init_error)
        rpio.stop()
        print('End RPIO Test')
        exit()
    print('\nBegin RPIO Background Test (RPIO instance 2)')
    rpiotest = RPIOTest()
    print('\nEnter a request from the following list:\n')
    print(REQUEST_LIST)
    rpio.start()
    rpiotest.start()
    while rpio.running:
        rpio.process_request(input())
    rpiotest.stop()
    print('End RPIO Test')
