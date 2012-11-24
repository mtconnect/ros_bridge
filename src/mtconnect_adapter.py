from SocketServer import ThreadingMixIn, TCPServer, BaseRequestHandler
import threading
from datetime import datetime
import re


class Adapter(ThreadingMixIn, TCPServer):
    def __init__(self, address, heartbeat_interval = 10000):
        TCPServer.__init__(self, address, BaseRequestHandler, False)
        self._clients = dict()
        self._lock = threading.RLock()
        self._data_items = []
        self._running = False
        self._heartbeat_interval = heartbeat_interval
        self._ping_pat = re.compile('\\* PING')

    def add_data_item(self, item):
        self._data_items.append(item)

    def start(self):
        self.server_bind()
        self._running = True
        self.server_activate()

    def finish_request(self, request, client_address):
        print "Connected to %s: %d" % (client_address[0], client_address[1])
        self._lock.acquire()
        self._clients[client_address] = request
        self._lock.release()

        self.send_initial(request)
        self.heartbeat(request)

        self.remove_client(client_address)

    def heartbeat(self, client):
        try:
            client.settimeout(None)
            while self._running:
                line = client.recv(256)
                if self._ping_pat.match(line):
                    if not client.gettimeout():
                        client.settimeout(self._heartbeat_interval / 500.0)

                    self._lock.acquire()
                    client.send("* PONG " + str(self._heartbeat_interval))
                    self._lock.release()
                else:
                    break
        except:
            print "Exception in heartbeat thread"

        print "Headbeat thread stopped"


    def remove_client(self, client_address):
        self._lock.acquire()
        try:
            if client_address in self._clients:
                socket = self._clients[client_address]
                del self._clients[client_address]
                socket.shutdown(socket.SHUT_RDWR)
        except:
            print "Exception closing socket for " + client_address[0]

        self._lock.release()

    def begin(self):
        for di in self._data_items:
            di.begin()

    def complete(self):
        for di in self._data_items:
            di.complete()

    def sweep(self):
        for di in self._data_items:
            di.sweep()

    def unavailable(self):
        for di in self._data_items:
            di.unavailable()

    def send_initial(self, request):
        self.send_changed([request], True)


    def format_time(self):
        time = datetime.utcnow()
        return time.strftime("%Y-%m-%dT%H:%M:%S.%f") + 'Z'

    def send_changed(self, clients, force = False):
        text = ''
        time = self.format_time()
        separate = [item for item in self._data_items if item.separate_line()]
        combined = [item for item in self._data_items if not item.separate_line()]
        for item in combined:
            if force or item.changed():
                text += ''.join(item.values(force))

        if len(text) > 0:
            self.send(time, text, clients)

        for item in separate:
          if force or item.changed():
            for line in item.values(force):
              self.send(time, line, clients)

    def format_line(self, time, text):
        return time + text + "\n"

    def send_to_client(self, client, line):
        try:
            socket = self._clients[client]
            socket.send(line)
            socket.flush()
        except:
            self.remove_client(client)

    def send(self, time, text, clients):
        line =  self.format_line(time, text)
        for client in clients.keys():
          self.send_to_client(client, line)

    def gather(self, function):
      self.begin()

      function()

      self.complete()
      self.send_changed(self._clients)
      self.sweep()
