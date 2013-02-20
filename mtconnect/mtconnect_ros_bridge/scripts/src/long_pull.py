"""Copyright 2012, System Insights, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License."""
    
import socket, select, re
import scanner
import httplib

class LongPullException(Exception):
    pass

class LongPull:
    def __init__(self, response):
        self._response = response
        self._buffer = ''

    def _read_all(self, size):
        globs = [self._buffer]
        amt = size - len(self._buffer)


        while amt > 0:
            ready, d1, d2 = select.select([self._socket], [], [])
            if len(ready) > 0:
                glob = self._socket.recv(amt)
                if not glob:
                    raise LongPullException(''.join(globs), amt)
                amt -= len(glob)
                globs.append(glob)

        return ''.join(globs)


    def _read_chunk(self):
        # This should always occur on a chunk boundry, so we should never
        # need to store a lot of text in the buffer.

        text = ''
        chunk_size = None

        if len(self._buffer) < 32:
            ready, d1, d2 = select.select([self._socket], [], [])
            if len(ready) == 0:
                raise httplib.NotConnected()

            text = self._socket.recv(32)
            if len(text) == 0:
                self._socket.close()
                raise httplib.NotConnected()

        self._buffer += text
        eol = self._buffer.find('\r\n')
        if eol >= 0:
            line = self._buffer[:eol]
            i = line.find(';')
            if i >= 0:
                line = line[:i] # strip chunk-extensions
            try:
                chunk_size = int(line, 16) + 2
            except ValueError:
                # close the connection as protocol synchronisation is
                # probably lost
                self._socket.close()
                raise httplib.IncompleteRead(line)
            self._buffer = self._buffer[(eol + 2):]

        chunk = self._read_all(chunk_size)
        self._buffer = chunk[chunk_size:]

        return chunk[:(chunk_size - 2)]


    def long_pull(self, callback, user_data = None):
        fileno = self._response.fileno()
        self._socket = socket.fromfd(fileno, socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setblocking(False)
        content_type = dict(self._response.getheaders())['content-type']
        match = re.search('boundary=([0-9A-Fa-f]+)', content_type)
        if not match:
            raise LongPullException('Cannot find boundary in content-type')

        boundary = '--' + match.group(1)
        boundary_pat = re.compile('^' + boundary)
        header = True
        length = len(boundary)
        document = scanner.Scanner('')
        while True:
            chunk = self._read_chunk()
            document.string += chunk

            while document.rest_len() >= length:
                if header:
                    if not document.check(boundary_pat):
                        print "Framing error!"
                        raise LongPullException('Framing error')

                    head = document.scan_until('\r\n\r\n')
                    mime_headers = head.split('\r\n')
                    values = dict([(v.lower().split(':')) for v in mime_headers[1:] if v.find(':') > 0])
                    header = False
                    try:
                        length = int(values['content-length'])
                    except ValueError:
                        raise LongPullException('Cannot get length from mime header: ' + mime_headers)

                else:
                    rest = document.rest()
                    body = rest[:length]

                    document.reset()
                    document.string = rest[length:]

                    callback(body)

                    length = len(boundary)
                    header = True

if __name__ == "__main__":
    conn = httplib.HTTPConnection('agent.mtconnect.org')
    conn.request("GET", "/sample?interval=1000&count=1000")
    response = conn.getresponse()

    lp = LongPull(response)
    def callback(chunk):
        print chunk

    lp.long_pull(callback)
