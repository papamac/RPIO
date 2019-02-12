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

from os import _exit
from textwrap import wrap
from threading import Thread

from rpio_global import *


class Server(Thread):
    
    # Class attribute:

    num_instances = 0

    # Private instance methods:
    
    def __init__(self):
        if self.num_instances:
            raise RuntimeError('Cannot run multiple instances of Server class')
        self.num_instances += 1
        self.srv_addr = ''
        self.sock = None
        self.dt_recvd = None
        self._connect_to_server()
        self.running = False
        Thread.__init__(self, name='Server ' + self.srv_addr,
                        target=self._print_server_data)

    def _connect_to_server(self):
        print('Enter server address:port, ret for defaults, or e, q to exit')
        self.srv_addr = DEFAULT_SRV_ADDR
        port = DEFAULT_PORT
        response = input()
        if response in ('e', 'q'):
            print('End RPIO Client')
            exit()
        if response:
            if ':' in response:
                self.srv_addr, port = response.split(':')
                if not port.isdigit():
                    print('Port number not an unsigned integer')
                    self._connect_to_server()
                    return
                port = int(port)
                if port not in DYNAMIC_PORT_RANGE:
                    print('Port number is not in dynamic port range')
                    self._connect_to_server()
                    return
            else:
                self.srv_addr = response
        try:
            ipv4_addr = gethostbyname(self.srv_addr)
        except OSError as err_msg:
            print('Address resolution error "%s" %s' % (self.srv_addr,
                                                        err_msg))
            self._connect_to_server()
            return
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        try:
            self.sock.connect((ipv4_addr, port))
        except OSError as err_msg:
            print('Connection error "%s" %s' % (self.srv_addr, err_msg))
            self._connect_to_server()
            return
        sock_id = '%s:%i' % self.sock.getsockname()
        print('Connected "%s" via socket "%s"' % (self.srv_addr, sock_id))

    def _print_server_data(self):
        while self.running:
            data = ''
            try:
                data = recv_msg(self, self.sock, self.dt_recvd)
                if not self.running:
                    break
            except OSError as err_msg:
                err_msg = 'Recv error "%s" %s' % (self.srv_addr, err_msg)
                self._terminal_error(err_msg)
            except BrokenPipe:
                err_msg = 'Server disconnected "%s"' % self.srv_addr
                self._terminal_error(err_msg)
            except ServerTimeout:
                err_msg = 'Server timeout "%s"' % self.srv_addr
                self._terminal_error(err_msg)
            self.dt_recvd = datetime.now()
            print(data[11:19] + data[DATETIME_LENGTH:])
            try:
                dt_sent = datetime.strptime(data[:26], '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                print('******** invalid datetime "%s" ********' % data[:26])
                continue
            latency = (self.dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY_LIMIT:
                print('******** late data %5.3f ********' % latency)

    def _terminal_error(self, err_msg):
        print(err_msg)
        self.running = False
        print('End RPIO Client')
        _exit(0)

    # Public instance methods:

    def send_request(self, *args):
        req = str(datetime.now())
        for arg in args:
            req = req + ' ' + str(arg)
        try:
            send_msg(self.sock, req)
        except OSError as err_msg:
            err_msg = 'Send error "%s" %s' % (self.srv_addr, err_msg)
            self._terminal_error(err_msg)
        except BrokenPipe:
            err_msg = 'Broken pipe to server "%s"' % self.srv_addr
            self._terminal_error(err_msg)

    def start(self):
        self.dt_recvd = datetime.now()
        self.running = True
        Thread.start(self)

    def stop(self):
        self.running = False
        self.join()
        self.sock.close()


# RPIO Interactive Client main program:"

print('\nBegin RPIO Interactive Client')
server = Server()
print('\nEnter a request from the following list:\n')
print(REQUEST_LIST)
server.start()
while server.running:
    request = input()
    if request:
        request_id = request[:2].strip()
        if request_id in ('e', 'q'):
            server.stop()
        elif request_id in ('h', '?'):
            print(REQUEST_LIST)
        else:
            if request_id == 'l':
                lines = wrap(request[1:].strip(), TEXT_LENGTH)
                if not lines:
                    lines = ['']
                requests = ['l ' + line for line in lines]
            else:
                requests = [request.strip()[:MESSAGE_LENGTH]]
            for request in requests:
                server.send_request(request)
print('End RPIO Interactive Client')
