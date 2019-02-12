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

from rpio import RPIO
from rpio_global import *


class Client(Thread):

    # Private instance methods:

    def __init__(self, rp, sock, sock_id):
        self.rpio = rp
        self.sock = sock
        self.sock_id = sock_id
        self.running = False
        Thread.__init__(self, name='Client' + self.sock_id,
                        target=self._process_client_requests)

    def _process_client_requests(self):
        while self.running:
            req = ''
            try:
                req = recv_msg(self, self.sock)
            except OSError as err_msg:
                self.rpio.process_request('l recv error "%s"' % self.sock_id)
                self.rpio.process_request('l %s' % err_msg)
                self.running = False
            except BrokenPipe:
                self.rpio.process_request(
                    'l client disconnected "%s"' % self.sock_id)
                self.running = False
            if not self.running:
                break
            dt_recvd = datetime.now()
            try:
                dt_sent = datetime.strptime(req[:DATETIME_LENGTH],
                                            '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                self.rpio.process_request('l invalid datetime "%s" %s'
                                          % (self.sock_id, req))
                continue
            latency = (dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY_LIMIT:
                self.rpio.process_request('l late request "%s" %s %5.3f'
                                          % (self.sock_id, req, latency))
            self.rpio.process_request(req[DATETIME_LENGTH + 1:])

    # Public instance methods:

    def start(self):
        self.running = True
        Thread.start(self)

    def stop(self):
        self.running = False
        self.join()
        self.sock.close()


class Server(Thread):

    # Private instance methods:

    def __init__(self, rp, port=DEFAULT_PORT):
        self.rpio = rp
        self.address_tuple = '', port
        self.sock = None
        self.clients = []
        self.running = False
        self.accept = Thread(name='Server_accept_client_connections',
                             target=self._accept_client_connections)
        Thread.__init__(self, name='Server_serve_rpio_data',
                        target=self._serve_data_to_clients)
    
    def _accept_client_connections(self):
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        self.sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.sock.bind(self.address_tuple)
        self.sock.listen(5)
        while self.running:
            try:
                sock, address_tuple = self.sock.accept()
            except timeout:
                continue
            sock_id = '%s:%i' % address_tuple
            self.rpio.process_request('l connected "%s"' % sock_id)
            sock.settimeout(SOCKET_TIMEOUT)
            client = Client(self.rpio, sock, sock_id)
            self.clients.append(client)
            client.start()

    def _serve_data_to_clients(self):
        while self.running:
            data = self.rpio.get_data()
            if data:
                for client in self.clients:
                    if client.running:
                        try:
                            send_msg(client.sock, data)
                        except OSError as err_msg:
                            self.rpio.process_request(
                                'l send error "%s"' % client.sock_id)
                            self.rpio.process_request('l %s ' % err_msg)
                        except BrokenPipe:
                            err_msg = 'broken pipe "%s"' % client.sock_id
                            self.rpio.process_request('l %s ' % err_msg)
                        else:
                            continue
                        self.rpio.process_request('l closed "%s" '
                                                  % client.sock_id)
                        client.stop()

    # Public instance methods:

    def start(self):
        self.rpio.start()
        self.running = True
        self.accept.start()
        Thread.start(self)

    def stop(self):
        self.running = False
        for client in self.clients:
            client.stop()
        self.accept.join()
        self.sock.shutdown(SHUT_RDWR)
        self.sock.close()
        self.rpio.stop()
        self.join()


# RPIO Server main program with interactive client:

if __name__ == '__main__':
    RPIO.enable_printing()
    print('\nBegin RPIO Server with interactive client capability')
    rpio = RPIO()
    if rpio.init_error:
        print(rpio.init_error)
        rpio.stop()
        print('End RPIO Server')
        exit()
    server = Server(rpio)
    print('\nEnter a request from the following list:\n')
    print(REQUEST_LIST)
    server.start()
    while True:
        request = input()
        if request in ('e', 'q'):
            break
        server.rpio.process_request(request)
    server.stop()
    print('End RPIO Server')
