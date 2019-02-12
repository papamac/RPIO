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

RPIO version 0.9.0 is an initial operational version that supports analog
input and digital input/output.  Documentation is incomplete.
"""
__author__ = 'papamac'
__version__ = '0.9.0'
__date__ = 'April 12, 2018'

from abc import ABCMeta, abstractmethod
from copy import copy
from datetime import datetime
from math import fabs, log2
from queue import *
from socket import gethostname
from textwrap import wrap
from threading import Thread, Lock
from time import sleep
from xml.etree.ElementTree import parse, ParseError

from I2Cbus import I2Cbus
import RPi.GPIO as GPIO

from rpio_global import MESSAGE_LENGTH, TEXT_LENGTH, REQUEST_LIST


# module rpio global constants:

# Timing constants:

NOMINAL_SLEEP_TIME = 1      # Nominal sleep time for port polling and
#                             status run loops (sec).
MINIMUM_SLEEP_TIME = 0.001  # Minimum sleep time needed to force equal
#                             allocation of processor time to multiple
#                             port threads (sec).
STATUS_INTERVAL = 60        # Interval for port status reporting (sec).
VALUE_INTERVAL = 600        # Interval for unforced value reporting (sec).

# Default configuration constants:

DEFAULT_DIRECTION = 1       # Digital input (1) or output (0).
DEFAULT_POLARITY = 0        # Normal logic polarity (0) or inverted (1).
DEFAULT_PULLUP = 0          # No pullup on digital input (0) or pullup to logic
#                             high through built-in 100K resistor (1).
DEFAULT_RESOLUTION = 12     # ADC resolution (12, 14, 16, or 18 bits).
DEFAULT_GAIN = 1            # ADC gain (1, 2, 4, or 8).
DEFAULT_CHANGE_LIMIT = 0.5  # ADC percentage limit for value change reporting.
DEFAULT_SCALING_FACTOR = (6.8 + 10) / 6.8  # Scaling factor for 5V single ended
#                                            channels on the AB Electronics ADC
#                                            Pi Plus board.


class Port(Thread, metaclass=ABCMeta):

    # Private instance methods:

    def __init__(self, rp, port_def):
        self.rpio = rp
        self.starting_channel = int(port_def['starting_channel'])
        self.ending_channel = int(port_def['ending_channel'])
        self.num_channels = self.ending_channel - self.starting_channel + 1
        self.channels = range(self.starting_channel, self.ending_channel + 1)
        self.polling = int(port_def['polling'])
        if self.polling < 0:
            self.sleep_time = NOMINAL_SLEEP_TIME
        elif self.polling == 0:
            self.sleep_time = MINIMUM_SLEEP_TIME
        elif self.polling > 0:
            self.sleep_time = min(self.polling, NOMINAL_SLEEP_TIME)
        self.report_changes = port_def['report_changes'] == 'True'
        self.zero = self.num_channels * [0]
        self.directions = self.resolutions = copy(self.zero)
        self.polarities = self.gains = copy(self.zero)
        self.pullups = self.scaling_factors = copy(self.zero)
        self.values = copy(self.zero)
        self.running = False
        self.lock = Lock()
        name = '%s[%i:%i]' % (port_def['type'], self.starting_channel,
                              self.ending_channel)
        Thread.__init__(self, name=name, target=self._polling_and_status)

    def _polling_and_status(self):
        poll_count = 0
        dt_poll = dt_status = dt_value = datetime.now()
        while self.running:
            dt_now = datetime.now()
            secs_since_last_poll = (dt_now - dt_poll).total_seconds()
            if secs_since_last_poll >= self.polling >= 0:
                self._poll()
                poll_count += 1
                dt_poll = dt_now
            secs_since_last_status = (dt_now - dt_status).total_seconds()
            if secs_since_last_status >= STATUS_INTERVAL:
                if self.polling >= 0 and poll_count > 0:
                    avg_polling_interval = secs_since_last_status / poll_count
                    avg_polling_rate = 1 / avg_polling_interval
                    self.rpio.queue_data('l', '"%s" port active %6.4f %6.1f'
                                         % (self.name, avg_polling_interval,
                                            avg_polling_rate))
                    poll_count = 0
                else:
                    self.rpio.queue_data('l', '"%s" port active' % self.name)
                dt_status = dt_now
            secs_since_last_value = (dt_now - dt_value).total_seconds()
            if secs_since_last_value >= VALUE_INTERVAL:
                for channel in self.channels:
                    channel_index = channel - self.starting_channel
                    value = self.values[channel_index]
                    self.rpio.queue_data('v', channel, value)
                dt_value = dt_now
            sleep(self.sleep_time)

    # Public instance methods:

    def start(self):
        self.running = True
        Thread.start(self)

    def stop(self):
        if self.running:
            self.running = False
            self.join()

    # Instance methods that may optionally be provided or extended by
    # subclasses:

    def _poll(self):
        pass

    def write(self, channel, value):
        pass

    def print(self):
        print('\nport: %s\tpolling: %i\t\treport_changes: %s'
              % (self.name, self.polling, self.report_changes))
        print('starting_channel: %i\tending_channel: %i\tnum_channels: %i' %
              (self.starting_channel, self.ending_channel, self.num_channels))
        inc = 3
        for i1 in range(self.starting_channel, self.ending_channel + 1, inc):
            i2 = min(i1 + inc, self.ending_channel + 1)
            for channel in range(i1, i2):
                channel_index = channel - self.starting_channel
                if isinstance(self.values[channel_index], float):
                    format_str = 'ch %i(%i %i): %7.5f'
                else:
                    format_str = 'ch %i(%i %i): %i  \t'
                print(format_str % (channel,
                                    self.directions[channel_index],
                                    self.polarities[channel_index],
                                    self.values[channel_index]), end='\t')
            print()

    # Abstract instance methods that must be provided by subclasses:

    @abstractmethod
    def configure(self, channel, *args):
        pass

    @abstractmethod
    def read(self, channel):
        pass


class BCM_DIO(Port):

    # Private instance methods:

    def __init__(self, rp, port_def):
        Port.__init__(self, rp, port_def)
        self.bcm_starting_channel = int(port_def['bcm_starting_channel'])
        if not (0 < self.num_channels <= 24):
            raise RuntimeError('"BCM_DIO" invalid number of channels in port '
                               'definition')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for channel in self.channels:
            self.configure(channel)
            self.read(channel)
        self.print()

    def _apply_polarity(self, channel_index, input_value):
        if self.polarities[channel_index]:
            value = 0 if input_value else 1
        else:
            value = input_value
        return value

    def _poll(self):
        for channel in self.channels:
            channel_index = channel - self.starting_channel
            bcm_channel = channel_index + self.bcm_starting_channel
            with self.lock:
                prior_value = self.values[channel_index]
                value = self._apply_polarity(channel_index,
                                             GPIO.input(bcm_channel))
                self.values[channel_index] = value
            if self.report_changes:
                if value != prior_value:
                    self.rpio.queue_data('n', channel, value)
            else:
                self.rpio.queue_data('v', channel, value)

    # Public instance methods:

    def configure(self, channel, direction=DEFAULT_DIRECTION,
                  polarity=DEFAULT_POLARITY, pullup=DEFAULT_PULLUP):
        channel_index = channel - self.starting_channel
        bcm_channel = channel_index + self.bcm_starting_channel
        bcm_direction = GPIO.IN if direction else GPIO.OUT
        if pullup > 0:
            bcm_pullup = GPIO.PUD_UP
        elif pullup < 0:
            bcm_pullup = GPIO.PUD_DOWN
        else:
            bcm_pullup = GPIO.PUD_OFF
        with self.lock:
            self.directions[channel_index] = direction
            self.polarities[channel_index] = polarity
            self.pullups[channel_index] = pullup
            GPIO.setup(bcm_channel, bcm_direction, bcm_pullup)

    def read(self, channel):
        channel_index = channel - self.starting_channel
        bcm_channel = channel_index + self.bcm_starting_channel
        with self.lock:
            value = self._apply_polarity(channel_index,
                                         GPIO.input(bcm_channel))
            self.values[channel_index] = value
        return value

    def write(self, channel, value):
        channel_index = channel - self.starting_channel
        if self.directions[channel_index]:
            raise RuntimeError('Channel not configured for output')
        bcm_channel = channel_index + self.bcm_starting_channel
        with self.lock:
            GPIO.output(bcm_channel, self._apply_polarity(channel_index,
                                                          value))
            self.values[channel_index] = value

    def print(self):
        if RPIO.printing:
            with RPIO.print_lock:
                Port.print(self)
                print('bcm_starting_channel: %i' % self.bcm_starting_channel)


class MCP_ADC(Port):

    # Class attribute:

    I2C_BASE_ADDRESS = 0x68

    # Private instance methodS:

    def __init__(self, rp, port_def):
        Port.__init__(self, rp, port_def)
        self.mcp_address = int(port_def['mcp_address'])
        self.I2C_address = self.I2C_BASE_ADDRESS + self.mcp_address
        if self.num_channels != 4:
            raise RuntimeError('"MCP_ADC" invalid number of channels in port '
                               'definition')
        self.change_limits = copy(self.zero)
        self.configs = copy(self.zero)
        self.num_bytes = copy(self.zero)
        self.prior_values = copy(self.zero)
        for channel in self.channels:
            self.configure(channel)
            self.read(channel)
        self.print()

    def _poll(self):
        for channel in self.channels:
            channel_index = channel - self.starting_channel
            with self.lock:
                value = self._read(channel_index)
                prior_value = self.prior_values[channel_index]
            if self.report_changes:
                if prior_value != 0.0:
                    percent_change = 100.0 * fabs((value - prior_value) /
                                                  prior_value)
                    if percent_change > self.change_limits[channel_index]:
                        self.rpio.queue_data('n', channel, value)
            else:
                self.rpio.queue_data('v', channel, value)

    def _read(self, channel_index):
        config = self.configs[channel_index]
        I2Cbus.write_byte(self.I2C_address, config)
        config = config & 0x7F
        while True:
            byts = I2Cbus.read_i2c_block_data(
                self.I2C_address, config,
                self.num_bytes[channel_index])
            if byts[-1] < 128:
                break
        counts = int.from_bytes(byts[:-1], byteorder='big', signed=True)
        if counts < 0:
            counts = 0
        lsb_value = 4.096 / 2 ** self.resolutions[channel_index]
        value = (counts * lsb_value / self.gains[channel_index] *
                 self.scaling_factors[channel_index])
        self.prior_values[channel_index] = self.values[channel_index]
        self.values[channel_index] = value
        return value

    # Public instance methods:

    def configure(self, channel, resolution=DEFAULT_RESOLUTION,
                  gain=DEFAULT_GAIN, change_limit=DEFAULT_CHANGE_LIMIT,
                  scaling_factor=DEFAULT_SCALING_FACTOR):
        if resolution not in (12, 14, 16, 18):
            raise RuntimeError('invalid ADC resolution')
        if gain not in (1, 2, 4, 8):
            raise RuntimeError('invalid ADC gain')
        channel_index = channel - self.starting_channel
        self.resolutions[channel_index] = resolution
        self.gains[channel_index] = gain
        self.change_limits[channel_index] = change_limit
        self.scaling_factors[channel_index] = scaling_factor
        resolution_index = int((resolution - 12) / 2)
        gain_index = int(log2(gain))
        self.configs[channel_index] = (0x80 + 32 * channel_index +
                                       4 * resolution_index + gain_index)
        self.num_bytes[channel_index] = 3 if resolution < 18 else 4

    def read(self, channel):
        channel_index = channel - self.starting_channel
        with self.lock:
            return self._read(channel_index)

    def print(self):
        if RPIO.printing:
            with RPIO.print_lock:
                Port.print(self)
                print('mcp_address: %i\t\tI2C_address: 0x%02X' %
                      (self.mcp_address, self.I2C_address))


class MCP_DIO(Port):

    # Class attributes:

    I2C_BASE_ADDRESS = 0x20
    REGISTER_BASE_ADDRESSES = dict(
        IODIR=0x00,  # IO direction register
        IPOL=0x01,   # Input polarity register
        IOCON=0x05,  # IO configuration register
        GPPU=0x06,   # GPIO pull-up resistor configuration register
        GPIO=0x09,   # GPIO port value register
        OLAT=0x0A    # Output latch register
        )

    # Private instance methods:

    def __init__(self, rp, port_def):
        Port.__init__(self, rp, port_def)
        self.mcp_address = int(port_def['mcp_address'])
        self.port_number = int(port_def['port_number'])
        self.I2C_address = self.I2C_BASE_ADDRESS + self.mcp_address
        if self.num_channels != 8:
            raise RuntimeError('"MCP_DIO" invalid number of channels in port '
                               'definition')
        self.register_addresses = {}
        self.register_values = {}
        for register in self.REGISTER_BASE_ADDRESSES:
            self.register_addresses[register] = (
                self.REGISTER_BASE_ADDRESSES[register] +
                self.port_number * 0x10)
            self.register_values[register] = 0

        # Set the port's IO configuration register:
        # IOCON = 0xA2 (bit 7 BANK = 1, bit 5 SEQOP = 1, and bit 1 INTPOL = 1).
        # This provides byte-oriented register addressing and disables
        # sequential operation.  It allows mixed use of MCP23008 and MCP23017
        # chips in the same gpio port definition.  See the MCP23017 Data
        # Sheet page 18 for details.

        iocon_value = self._read_register('IOCON')
        if iocon_value != 0xA2:
            self._write_register('OLAT', 0xA2)
        for channel in self.channels:
            self.configure(channel)
            self.read(channel)
        self._read_register('OLAT')
        self.print()

    def _poll(self):
        with self.lock:
            prior_gpio_value = self.register_values['GPIO']
            new_gpio_value = self._read_register('GPIO')
            changed_channels = new_gpio_value ^ prior_gpio_value
            for channel_index in range(8):
                mask = 1 << channel_index
                value = 1 if new_gpio_value & mask else 0
                self.values[channel_index] = value
                channel = self.starting_channel + channel_index
                if self.report_changes:
                    if changed_channels & mask:
                        self.rpio.queue_data('n', channel, value)
                else:
                    self.rpio.queue_data('v', channel, value)

    def _read_register(self, register):
        register_value = I2Cbus.read_byte_data(self.I2C_address,
                            self.register_addresses[register])
        self.register_values[register] = register_value
        return register_value

    def _write_register(self, register, register_value):
        I2Cbus.write_byte_data(self.I2C_address,
                               self.register_addresses[register],
                               register_value)
        self.register_values[register] = register_value

    def _update_register_value(self, register, channel_index, value):
        prior_register_value = self.register_values[register]
        mask = 1 << channel_index
        new_register_value = (prior_register_value | mask if value
                              else prior_register_value & ~mask)
        self._write_register(register, new_register_value)

    # Public instance methods:

    def configure(self, channel, direction=DEFAULT_DIRECTION,
                  polarity=DEFAULT_POLARITY, pullup=DEFAULT_PULLUP):
        channel_index = channel - self.starting_channel
        with self.lock:
            self._update_register_value('IODIR', channel_index, direction)
            self._update_register_value('IPOL', channel_index, polarity)
            self._update_register_value('GPPU', channel_index, pullup)

    def read(self, channel):
        channel_index = channel - self.starting_channel
        with self.lock:
            gpio_value = self._read_register('GPIO')
            value = 1 if gpio_value & (1 << channel_index) else 0
            self.values[channel_index] = value
            return value

    def write(self, channel, value):
        channel_index = channel - self.starting_channel
        if self.register_values['IODIR'] & (1 << channel_index):
            raise RuntimeError('Channel not configured for output')
        with self.lock:
            self._update_register_value('GPIO', channel_index, value)
            self.values[channel_index] = value

    def print(self):
        if RPIO.printing:
            with RPIO.print_lock:
                Port.print(self)
                print('mcp_address: %i\t\tI2C_address: 0x%02X\t'
                      'port_number: %i' % (self.mcp_address,
                       self.I2C_address, self.port_number))
                with self.lock:
                    register_list = [(self.register_addresses[reg], reg,
                                      self._read_register(reg))
                                     for reg in self.register_addresses]
                register_list.sort()
                num_regs = len(self.register_addresses)
                inc = 3
                for i1 in range(0, num_regs, inc):
                    i2 = min(i1 + inc, num_regs)
                    for j in range(i1, i2):
                        print('0x%02X %s: 0x%02X  ' % register_list[j],
                              end='\t')
                    print()


class RPIO:

    # Class attributes:

    INSTANTIATE_PORT = {'BCM_DIO': BCM_DIO,
                        'MCP_ADC': MCP_ADC,
                        'MCP_DIO': MCP_DIO
                        }
    printing = False
    print_lock = Lock()

    # Class methods:

    @staticmethod
    def enable_printing():
        RPIO.printing = True

    @staticmethod
    def disable_printing():
        RPIO.printing = False

    @staticmethod
    def _print(*args, **kwargs):
        if RPIO.printing:
            with RPIO.print_lock:
                print(*args, **kwargs)

    @staticmethod
    def print_request_list():
        RPIO._print(REQUEST_LIST)

    # Private instance methods:

    def __init__(self):
        self.PROCESS_REQUEST = {'c': self.configure,
                                'e': self.stop,
                                'h': self.print_request_list,
                                'l': self.log_text_data,
                                'p': self.print,
                                'q': self.stop,
                                'r': self.read,
                                'w': self.write,
                                '?': self.print_request_list
                                }
        self.ports = []
        self.data_queue = Queue()
        self.running = False
        self.init_error = None
        port_defs_xml = gethostname() + '_port_defs.xml'
        try:
            xml_tree = parse(port_defs_xml)
        except FileNotFoundError:
            self.init_error = 'Port definitions file "%s" not found'\
                         % port_defs_xml
            return
        except ParseError:
            self.init_error = 'Error in parsing port definitions file "%s"'\
                              % port_defs_xml
            return
        self._print('Using port definitions from "%s"' % port_defs_xml)
        port_defs = xml_tree.getroot()
        for pd in port_defs:
            port_def = pd.attrib
            port_type = u'unknown'
            try:
                port_type = port_def['type']
                port = self.INSTANTIATE_PORT[port_type](self, port_def)
            except KeyError:
                self.init_error = ('Invalid port type "%s" in port definition'
                                   % port_type)
                return
            except RuntimeError as err_msg:
                self.init_error = err_msg
                return
            self.ports.append(port)

    def _get_port(self, channel):
        for port in self.ports:
            if channel in port.channels:
                break
        else:
            raise RuntimeError('invalid channel number')
        return port

    # Public data queueing methods:

    def get_data(self):
        try:
            data = self.data_queue.get(timeout=1)
        except Empty:
            data = b''
        return data.decode().strip()

    def queue_data(self, data_id, *args):
        data = str(datetime.now()) + ' ' + data_id
        for arg in args:
            data = data + ' ' + str(arg)
        self.data_queue.put(data.ljust(MESSAGE_LENGTH).encode()
                            [:MESSAGE_LENGTH])
        RPIO._print(data[11:19], data[27:])

    # Public request processing methods:

    def process_request(self, request):
        try:
            request_id = request[:2].strip()
            if request_id == 'l':
                args = [request[2:]]
            else:
                args = [float(arg) if '.' in arg else int(arg)
                        for arg in request[2:].strip().split()]
            self.PROCESS_REQUEST[request_id](*args)
        except KeyError:
            self.queue_data('l', 'invalid request id "%s"' % request)
        except RuntimeError as err_msg:
            self.queue_data('l', '%s "%s"' % (err_msg, request))
        except:
            self.queue_data('l', 'invalid request "%s' % request)

    def configure(self, channel, *args):
        port = self._get_port(channel)
        port.configure(channel, *args)
        self.queue_data('c', channel, *args)
        self.read(channel)

    def read(self, channel):
        port = self._get_port(channel)
        value = port.read(channel)
        self.queue_data('r', channel, value)
        return value

    def write(self, channel, value):
        port = self._get_port(channel)
        port.write(channel, value)
        self.queue_data('w', channel, value)

    def print(self, channel):
        port = self._get_port(channel)
        port.print()

    def log_text_data(self, text):
        lines = wrap(text.strip(), TEXT_LENGTH)
        if not lines:
            raise RuntimeError('no text data')
        for line in lines:
            self.queue_data('l', line)

    def start(self):
        for port in self.ports:
            port.start()
        self.running = True

    def stop(self):
        for port in self.ports:
            port.stop()
        self.running = False


# RPIO Interactive main program:

if __name__ == '__main__':
    RPIO.enable_printing()
    print('\nBegin RPIO Interactive')
    rpio = RPIO()
    if rpio.init_error:
        print(rpio.init_error)
        rpio.stop()
        print('End RPIO Interactive')
        exit()
    print('\nEnter a request from the following list:\n')
    print(REQUEST_LIST)
    rpio.start()
    while rpio.running:
        rpio.process_request(input())
    print('End RPIO Interactive')
