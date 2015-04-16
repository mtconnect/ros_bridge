
require 'socket'

server = TCPServer.new(3000)
client = server.accept
while line = client.readline
  puts line
end
