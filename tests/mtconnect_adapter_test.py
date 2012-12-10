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
    

from mtconnect_adapter import Adapter
from data_item import Event
from nose.tools import eq_
from mock import MagicMock
import socket
import os

def create_server():
    fname = os.tmpnam()
    adapter = Adapter(fname, 10000, socket.AF_UNIX)

    return (fname, adapter)

def connect_client(fname):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(fname)
    client.settimeout(5.0)

    return client

def basic_adapter_connection_test():
    fname, adapter = create_server()
    adapter.start()
    client = connect_client(fname)

    client.send('* PING\n')
    line = client.recv(256)
    eq_(line, '* PONG 10000\n')

    adapter.stop()
    os.remove(fname)


def single_event_communications_test():
    fname, adapter = create_server()
    event = Event('foo')
    adapter.add_data_item(event)
    event.set_value('hello')
    adapter.start()

    adapter.format_time = MagicMock(return_value="TIME")

    client = connect_client(fname)
    line = client.recv(256)
    eq_(line, 'TIME|foo|hello\n')

    adapter.stop()
    os.remove(fname)

def multiple_events_communitcation_test():
    fname, adapter = create_server()
    event = Event('foo')
    adapter.add_data_item(event)
    event.set_value('hello')
    event = Event('bar')
    adapter.add_data_item(event)
    event.set_value('there')
    adapter.start()

    adapter.format_time = MagicMock(return_value="TIME")

    client = connect_client(fname)
    line = client.recv(256)
    eq_(line, 'TIME|foo|hello|bar|there\n')

    adapter.stop()
    os.remove(fname)
