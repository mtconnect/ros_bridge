
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

def should_have_two_faults_when_two_are_added_test():
    cond = Condition("foo")
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.add('fault', 'something else failed', '124')
    cond.complete()
    ok_(cond.changed())
    lines = cond.values()
    eq_(len(lines), 2)
    eq_(lines[0], '|foo|fault|123|||something failed')
    eq_(lines[1], '|foo|fault|124|||something else failed')

def should_have_one_changed_normal_when_one_fault_is_cleared_test():
    cond = Condition("foo")
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.add('fault', 'something else failed', '124')
    cond.sweep()
    ok_(not cond.changed())
    cond.begin()
    cond.add('fault', 'something failed', '123')
    lines = cond.values()
    eq_(len(lines), 1)
    eq_(lines[0], '|foo|normal|124|||')

def cond_with_two_faults():
    cond = Condition("foo")
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.add('fault', 'something else failed', '124')
    cond.complete()
    cond.sweep()
    return cond

def should_have_one_normal_and_one_fault_when_a_fault_is_cleared_test():
    cond = cond_with_two_faults()
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.complete()
    ok_(cond.changed())
    lines = cond.values(True)
    eq_(len(lines), 1)
    eq_(lines[0], '|foo|fault|123|||something failed')

def should_have_no_faults_when_the_same_fault_is_readded_test():
    cond = cond_with_two_faults()
    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.complete()
    ok_(cond.changed())
    cond.sweep()

    cond.begin()
    cond.add('fault', 'something failed', '123')
    cond.complete()
    ok_(not cond.changed())

    lines = cond.values()
    eq_(len(lines), 0)

    lines = cond.values(True)
    eq_(len(lines), 1)
    eq_(lines[0], '|foo|fault|123|||something failed')

def should_go_back_to_normal_when_no_faults_are_added_test():
    condition = cond_with_two_faults()
    condition.begin()
    condition.add('fault', 'something failed', '123')
    condition.complete()

    ok_(condition.changed())
    condition.sweep()

    condition.begin()
    condition.complete()
    ok_(condition.changed())

    lines = condition.values()
    eq_(lines[0], '|foo|normal||||')
