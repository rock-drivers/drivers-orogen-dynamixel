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
    driver.servo_id = 1
    driver.zero_offset = 4.58660255577807

    driver.configure
    driver.start

    angle = 0.0
    #reader = driver.scanner_tilt_angle.reader(:type => :buffer, :size => 10)
    reader = driver.angle.reader()
  
    sample = nil
    while(!(sample = reader.read))
      sleep 0.1
    end

    puts("#{sample/Math::PI*180.0} \r")
  
    read_angle = 0.0

    loop do
	driver.setAngle( angle/180.0*Math::PI )



        while((read_angle/Math::PI * 180.0 - angle).abs > 2.0)
          if sample = reader.read
            read_angle = sample
	    print("Cur Angle: #{sample/Math::PI*180.0} \r")
          end
          sleep 0.01
        end

	sleep 0.1
        puts("")

        puts('Enter next angle')
	cmd = $stdin.readline.chomp
	if cmd == "q"
	    exit
	else
	    angle = cmd.to_i
          puts("Setting angle to #{angle}")
	end


    end
end

