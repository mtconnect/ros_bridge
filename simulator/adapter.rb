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

require 'socket'
require 'thread'
require 'logger'
require 'data_item'

$logger = Logger.new(STDOUT)
$logger.level= Logger::DEBUG

module MTConnect
  class Adapter
    attr_reader :data_items

    def initialize(port = 7878, heartbeat_interval = 1000)
      @port = port
      @heartbeat_interval = heartbeat_interval
      @running = false
      @mutex = Mutex.new
      @cond = ConditionVariable.new
      @clients = []
      @data_items = []
      @started = ConditionVariable.new
      @heartbeat = @acceptor = nil
    end

    def start
      @acceptor = Thread.new do
        begin
          server = TCPServer.new(@port)
          @mutex.synchronize do
            @running = true
            @started.signal
          end
          while @running
            $logger.info('start') { "Waiting on 0.0.0.0 #{@port}" }
            sock = server.accept
            next unless sock
            $logger.info('start') { "Client connected: #{sock.peeraddr.inspect}" }
            begin
              @mutex.synchronize do
                @clients << sock
              end
              # Send initial data
              $logger.debug('start') { "Sending initial data" }
              send_initial(sock)
              heartbeat(sock)
            rescue
              $logger.error('start') { "Exception occurred while adding client: #{$!}\n#{$!.backtrace.join("\n")}" }
              remove_client(sock)
            end
          end
        rescue
          $logger.error('start') { "Exception occurred: #{$!}\n#{$!.backtrace.join("\n")}" }
        end
      end
      @acceptor = nil
    end

    def heartbeat(client)
      @heartbeat = Thread.new do
        $logger.debug('heartbeat') { "Starting heartbeat thread for #{client.peeraddr.inspect}" }
        begin
          timeout = nil
          while @running
            readables, = select([client], nil, nil, timeout)
            if readables != nil
              if (r = client.read_nonblock(256)) =~ /\* PING/
                timeout = @heartbeat_interval / 500.0
                # $logger.debug('heartbeat') { "Received #{r.strip}, responding with pong" }
                @mutex.synchronize do
                  client.puts "* PONG #{@heartbeat_interval}"
                end
              else
                $logger.debug('heartbeat') { "Received '#{r.strip}'" }
              end
            else
              $logger.warn('heartbeat') {  "#{client.peeraddr.inspect} did not ping after #{timeout} seconds" }
              break
            end
          end
        rescue Exception
          $logger.error('heartbeat') { "An error occurred: #{$!}\n#{$!.backtrace.join("\n")}" }

        ensure
          $logger.info('heartbeat') { "Heartbeat thread exiting, removing client #{client.peeraddr.inspect}" }
          remove_client(client)
        end
      end
      @heartbeat = nil
    end

    def stop
      unavailable
      send_changed
      sweep
      @running = false
      $logger.info('stop') { "stopping adapter" }
      @mutex.synchronize do
        @clients.each { |client| client.close }
        @clients.clear
      end
      if @acceptor
        @acceptor.raise RuntimeException, 'Stopped'
        @acceptor.join if @acceptor
      end
      if @heartbeat
        @heartbeat.raise RuntimeException, 'Stopped'
        @heartbeat.join if @heartbeat
      end
    end

    def remove_client(client)
      @mutex.synchronize do
        client.close
        @clients.delete(client)
      end
    end

    def format_time
      time = Time.now.utc
      time.strftime("%Y-%m-%dT%H:%M:%S.") + ("%06d" % time.usec) + 'Z'
    end


    def format_line(time, text)
      line =  "#{time}#{text}\n"
    end

    def begin
      @data_items.each { |d| d.begin }
    end

    def complete
      @data_items.each { |d| d.complete }
    end

    def sweep
      @data_items.each { |d| d.sweep }
    end

    def unavailable
      @data_items.each { |d| d.unavailable }
    end

    def send_changed(clients = @clients, force = false)
      text = ''
      time = format_time
      separate, combined = @data_items.partition { |i| i.separate_line? }
      combined.each do |item|
        text << item.values(force).join if force or item.changed?
      end
      send(time, text, clients) unless text.empty?
      separate.each do |item|
        if force or item.changed?
          item.values(force).each do |line|
            send(time, line, clients)
          end
        end
      end
    end

    def gather
      self.begin

      yield

      self.complete
      self.send_changed
      self.sweep
    end

    def send_to_client(client, text)
      @mutex.synchronize do
        $logger.debug('send_to_client') { "Sending #{text} to #{client.inspect}" }
        client.write text
        client.flush
      end
    rescue
      $logger.error('send_to_client') { "Error occurred #{$!}, removing client #{client.inspect}\n#{$!.backtrace.join("\n")}" }
      remove_client(client)
    end

    def send(time, text, clients)
      line =  format_line(time, text)
      clients.reverse_each do |sock|
        send_to_client(sock, line)
      end
    end

    def send_initial(client)
      send_changed([client], true)
    end
  end
end