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
    
from collections import OrderedDict

class DataItem:
    """A Representation of an MTConnect Data Item"""

    def __init__(self, name):
        self._name = name
        self._changed = False
        self._value = None
        self._separate_line = False

    def value(self):
        return self._value

    def set_value(self, value):
        if self._value != value:
            self._value = value
            self._changed = True

    def changed(self):
        return self._changed

    def separate_line(self):
        return self._separate_line

    def name(self):
        return self._name

    def begin(self):
        return

    def complete(self):
        return

    def sweep(self):
        self._changed = False

    def unavailable(self):
        if self._value:
            self._value = False
            self._changed = True

    def values(self, all = False):
        if self._value:
            v = self._value
        else:
            v = "UNAVAILABLE"
        return ["|" + self._name + "|" + str(v)]


class Event(DataItem):
    """An Event"""
    pass

class Sample(DataItem):
    """A Sample"""
    pass

class ThreeDSample(DataItem):
    def values(self, all = False):
        if self._value:
            v = ' '.join([str(i) for i in self._value])
        else:
            v = "UNAVAILABLE"
        return ["|" + self._name + "|" + str(v)]


class ConditionActivation:
    def __init__(self, level, text, code, qualifier, severity):
        self._level = level
        self._text = text
        self._code = code
        self._qualifier = qualifier
        self._severity = severity
        self._changed = True
        self._marked = True

    def mark(self):
        self._marked = True

    def clear(self):
        self._marked = False
        self._changed = False

    def clear_changed(self):
        self._changed = False

    def marked(self):
        return self._marked

    def changed(self):
        return self._changed

    def code(self):
        return self._code

    def __eq__(self, other):
        return self._code == other.code

    def __str__(self):
        return self._level + "|" + self._code + "|" + self._severity + "|" + self._qualifier + "|" + self._text


class Condition(DataItem):
    def __init__(self, name):
        DataItem.__init__(self, name)
        self._active = OrderedDict()
        self._separate_line = True

    def add(self, level, text, code, qualifier = "", severity = ""):
        self._value = True
        if code in self._active:
            cond = self._active[code]
            cond.mark()
        else:
            cond = ConditionActivation(level,text, code, qualifier, severity)
            self._active[code] = cond
            self._changed = True

        return cond

    def normal(self):
        if not self._value:
            self._value = True
            self._changed = True

    def begin(self):
        for code, cond in self._active.items():
            cond.clear()

    def complete(self):
        marked = len([cond for code, cond in self._active.items() if not cond.marked()])
        if marked > 0:
            self._changed = True

    def sweep(self):
        DataItem.sweep(self)
        for code in self._active.keys():
            if not self._active[code].marked():
                del self._active[code]


    def values(self, all = False):
        if self._value:
            active = [cond for code, cond in self._active.items() if cond.marked()]
            if len(active) == 0:
                return ["|" + self._name + "|normal||||"]
            elif all:
                return ["|" + self._name + "|" + str(cond) for cond in active]
            else:
                cleared = [cond for code, cond in self._active.items() if not cond.marked()]
                return  ["|" + self._name + "|normal|" + cond.code() + "|||" for cond in cleared] + \
                    ["|" + self._name + "|" + str(cond) for cond in active if cond.changed()]
        else:
            return ["|" + self._name + "|UNAVAILABLE||||"]


class SimpleCondition(Condition):
    def begin(self):
        for code, cond in self._active.items():
            cond.clear_changed()

    def remove(self, code):
        if code in self._active:
            cond = self._active[code]
            cond.clear()