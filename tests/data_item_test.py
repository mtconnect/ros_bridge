
from data_item import Event, Condition
from nose.tools import ok_, eq_

def init_test():
    event = Event("foo")
    eq_(event.changed(), False)
    eq_(event._name, "foo")

    event.set_value("blat")
    eq_(event.changed(), True)
    eq_(event.value(), "blat")

    lines = event.values()
    eq_(lines[0], "|foo|blat")

    event.sweep()
    eq_(event.changed(), False)

def change_test():
    event = Event("foo")
    eq_(event.changed(), False)

    event.set_value("blat")
    eq_(event.changed(), True)
    eq_(event.value(), "blat")

    event.sweep()
    eq_(event.changed(), False)

    event.set_value("blat")
    eq_(event.changed(), False)
    eq_(event.value(), "blat")


def condition_is_unavailable_at_start_test():
    cond = Condition("foo")
    values = cond.values()
    eq_(len(values), 1)
    eq_(values[0], '|foo|UNAVAILABLE||||')

def should_have_one_fault_when_a_fault_is_added_test():
    cond = Condition("foo")
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.complete()
    ok_(cond.changed())
    values = cond.values()
    eq_(len(values), 1)
    eq_(values[0], '|foo|fault|123|||something failed')

def should_be_normal_when_a_fault_is_added_and_then_not_added_again_test():
    cond = Condition("foo")
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.complete()
    ok_(cond.changed())
    cond.sweep()
    cond.begin()
    values = cond.values()
    eq_(len(values), 1)
    eq_(values[0], '|foo|normal||||')

