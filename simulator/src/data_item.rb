# Copyright 2012, System Insights, Inc.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

module MTConnect
  class DataItem
    attr_reader :name, :value

    def initialize(name)
      @name = name
      @changed = false
      @value = nil
      @separate_line = false
    end

    def value=(v)
      unless @value == v
        @value = v
        @changed = true
      end
    end

    def changed?; @changed end

    def separate_line?; false end

    def begin; end
    def complete; end
    def sweep; @changed = false end
    def unavailable
      if @value
        @value = false
        @changed = true
      end
    end

    def values(all = false)
      v = @value ? @value : 'UNAVAILABLE'
      ["|#{@name}|#{v}"]
    end
  end

  class Event < DataItem

  end

  class Sample < DataItem

  end

  class Condition < DataItem
    class Activation
      attr_reader :level, :text, :code, :qualifier, :severity

      def initialize(level, text, code, qualifier, severity)
        @level, @text, @code, @qualifier, @severity =
                level, text, code, qualifier, severity
        @changed = true
        @marked = true
      end

      def mark
        @marked = true
      end

      def clear
        @marked = false
        @changed = false
      end

      def clear_changed
        @changed = false
      end

      def marked?; @marked end
      def changed?; @changed end

      def to_s
        "#{@level}|#{@code}|#{@severity}|#{@qualifier}|#{@text}"
      end
    end

    attr_reader :active

    def initialize(name)
      super(name)
      @active = []
    end

    def separate_line?; true end

    def add(level, text, code, qualifier = "", severity = "")
      @value = true
      cond = @active.find { |a| a.code == code }
      if cond
        cond.mark
      else
        cond = Activation.new(level, text, code, qualifier, severity)
        @active << cond
        @changed = true
      end

      cond
    end

    def normal
      unless @value
        @value = true
        @changed = true
      end
    end

    def normal?
      @active.empty?
    end

    def begin
      @active.each { |a| a.clear }
    end

    def complete
      @changed ||= @active.any? { |a| !a.marked? }
    end

    def sweep
      super
      @active.delete_if { |a| !a.marked? }
    end

    def changed?
      @changed
    end

    def values(all = false)
      if @value
        active, cleared = @active.partition { |a| a.marked? }
        if active.empty?
          ["|#{@name}|normal||||"]
        elsif all
          active.map do |a|
            "|#{@name}|#{a}"
          end
        else
          cleared.map do |a|
            "|#{@name}|normal|#{a.code}|||"
          end.concat(active.map do |a|
            "|#{@name}|#{a}" if a.changed?
          end).delete_if { |e| e.nil? }
        end
      else
        ["|#{@name}|UNAVAILABLE||||"]
      end
    end
  end

  class SimpleCondition < Condition
    def begin
      @active.each { |a| a.clear_changed }
    end

    def remove(code)
      cond = @active[code]
      cond.clear if cond
    end

    def normal
      super
      @active.clear
      @changed = true
    end
  end
end