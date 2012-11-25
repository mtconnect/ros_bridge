import sys, os, time

path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/../src')

from data_item import Event
from mtconnect_adapter import Adapter

if __name__ == "__main__":
    adapter = Adapter(('localhost', 7878))
    event = Event('e1')
    adapter.add_data_item(event)
    adapter.start()

    for value in range(100):
        adapter.begin_gather()
        print "Setting event to " + str(value)
        event.set_value(value)
        adapter.complete_gather()
        time.sleep(1.0)

