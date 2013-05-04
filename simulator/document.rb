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

$: << '.'
$: << './src'

require 'cnc'

dir = File.dirname(__FILE__) + '/graph'
Dir.mkdir dir unless File.exist?(dir)
context = Cnc::CncContext.new('')
context.statemachine.to_dot(:output => 'graph')
Dir.chdir('graph') do
  system('dot -Tpng -o main.png main.dot')
  Dir['*.dot'].each do |f|
    system("dot -Tpng -o #{File.basename(f, '.dot')}.svg #{f}")
  end

  Dir.mkdir('response') unless File.exist?('response')
  resp = Cnc::OpenDoor.new(context)
  resp.create_statemachine
  resp.statemachine.to_dot(:output => 'response')
  Dir.chdir('response') do
    system('dot -Tpng -o main.png main.dot')
    Dir['*.dot'].each do |f|
      system("dot -Tpng -o #{File.basename(f, '.dot')}.svg #{f}")
    end
  end

  Dir.mkdir('request') unless File.exist?('request')
  resp = Cnc::MaterialLoad.new(context)
  resp.create_statemachine
  resp.statemachine.to_dot(:output => 'request')
  Dir.chdir('request') do
    system('dot -Tpng -o main.png main.dot')
    Dir['*.dot'].each do |f|
      system("dot -Tpng -o #{File.basename(f, '.dot')}.svg #{f}")
    end
  end
end
