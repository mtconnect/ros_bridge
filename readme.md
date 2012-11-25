Python Adapter for MTConnect C++ Agent
-------

This is a simple adapter library written in Python to support the ROS
robotics integration project. The adapter supports multiple connections,
heartbeats and the following MTConnect data item types:

* Event  - A simple data item that usually contains with either a controlled
           vocabulary like ControllerMode or Execution, or text like Program.

* Sample - A numeric valued data item for values that are continuously changing
           like position, angle, or temperatures

* ThreeDSample - A sample that represents a three dimensional position like
            a path position. The path is represented by a three space list or
            tuple: (1.0, 2.0, 3.0)

* Condition - A condition is a data item that describes the current normal, fault, or
            warning state of a component for a certain type of problem. A warning is
            a situation that may self correct, whereas a fault needs manual intervention.
            Conditions assume a sweep of a set of error codes where the errors will be
            checked each time the adapter polls. This is usually how most PLCs operate
            and there is no specific message to say when an alarm becomes active or
            clears. If there is an alarm messaging mechanism, use the SimpleCondition.

            Conditions rely on a native code to distinguish themselves. Other systems
            can be used, but this is the current assumption of the agent for its
            condition management.

* SimpleCondition - Similar to Condition, except that when the alarm is cleared, use
            the remove method with the code.


Adapter Creation
------

To create an adapter, import the Adapter from the mtconnect_adapter and add
data items. You need to tell it what host and port it should bind to. The default
port is 7878:

    adapter = Adapter(('0.0.0.0', 7878))
    di = Event('event_1')
    adapter.add_data_item(di)
    adapter.start()

This creates an adapter, adds an Event and then starts the adapter. To change
values of the data item, you need to begin gathering data and then complete the
data gathering process to send all the data.

    adapter.begin_gather()
    di.set_value('hello')
    adapter.complete_gather()

Working with conditions are very similar to the events:

    c1 = SimpleCondition('c1')
    adapter.add_data_item(c1)

    ...

Add a fault

    adapter.begin_gather()
    c1.add('fault', 'A fault', '123')
    adapter.complete_gather()

And to clear the condition:

    adapter.begin_gather()
    c1.remove('123')
    adapter.complete_gather()

There are two additional arguments that are options, qualifier and severity.
The qualifier represents extra information like HIGH or LOW and the severity
is a native severity if available.

There are more examples in the simple_adapter.py.