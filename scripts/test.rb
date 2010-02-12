#! /usr/bin/ruby

require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.spawn 'test' do |p|
    driver = p.task 'dynamixel_Task'
    Orocos.log_all_ports

    driver.device = ARGV[0]
    driver.scanner_tilt_id = 1
    driver.scanner_tilt_min = 0
    driver.scanner_tilt_max = 2*Math::PI

    driver.configure
    driver.start

    angle = 180.0
    #reader = driver.scanner_tilt_angle.reader(:type => :buffer, :size => 10)
    reader = driver.scanner_tilt_angle.reader

    loop do
	driver.set_scanner_tilt_angle( angle/180.0*Math::PI )

	if sample = reader.read
	    print("#{sample/Math::PI*180.0} \r")
	end
	sleep 0.1

	cmd = $stdin.readline.chomp
	if cmd.to_i > 0
	    angle = cmd.to_i
	elsif cmd == "q"
	    exit
	end
    end
end

