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

require 'adapter'
require 'data_item'

include MTConnect

adapter = Adapter.new(7878)

adapter.data_items << (availability_di = DataItem.new('avail'))

adapter.data_items << (mode_di = DataItem.new('mode'))
adapter.data_items << (exec_di = DataItem.new('exec'))


adapter.data_items << (material_load_di = DataItem.new('material_load'))
adapter.data_items << (material_unload_di = DataItem.new('material_unload'))

adapter.data_items << (open_chuck_di = DataItem.new('open_chuck'))
adapter.data_items << (close_chuck_di = DataItem.new('close_chuck'))

adapter.data_items << (open_door_di = DataItem.new('open_door'))
adapter.data_items << (close_door_di = DataItem.new('close_door'))

adapter.start

adapter.gather do
  availability_di.value = "AVAILABLE"
  mode_di.value = "AUTOMATIC"
  exec_di.value = "ACTIVE"
  [material_load_di, material_unload_di, open_chuck_di, close_chuck_di,
   open_door_di, close_door_di].each { |di| di.value = "READY" }
end

sleep 5

while true
  adapter.gather do
    material_load_di.value = "ACTIVE"
  end
  sleep 0.5
  adapter.gather do
    close_chuck_di.value = "ACTIVE"
  end
  sleep 1.5
  adapter.gather do
    close_chuck_di.value = "READY"
  end
  sleep 1
  adapter.gather do
    close_door_di.value = "ACTIVE"
  end
  sleep 1.5
  adapter.gather do
    close_door_di.value = "READY"
  end
  sleep 0.5
  adapter.gather do
    material_load_di.value = "COMPLETE"
  end
  sleep 0.5
  adapter.gather do
    material_load_di.value = "READY"
  end
  sleep 6
  adapter.gather do
    material_unload_di.value = "ACTIVE"
  end
  sleep 0.5
  adapter.gather do
    open_door_di.value = "ACTIVE"
  end
  sleep 1.5
  adapter.gather do
    open_door_di.value = "READY"
  end
  sleep 1
  adapter.gather do
    open_chuck_di.value = "ACTIVE"
  end
  sleep 1.5
  adapter.gather do
    open_chuck_di.value = "READY"
  end
  sleep 0.5
  adapter.gather do
    material_unload_di.value = "COMPLETE"
  end
  sleep 0.5
  adapter.gather do
    material_unload_di.value = "READY"
  end

  puts "Done with one part..."
  sleep 5
end