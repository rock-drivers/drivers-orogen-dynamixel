#! /usr/bin/ruby
require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize
Orocos.load_typekit("dynamixel")

DynamixelType = Orocos.registry.get("dynamixel/DynamixelDaisyChain")

Orocos.run 'test' do
    driver = TaskContext.get 'dynamixel_Task'

    driver.device = ARGV[0]
    driver.servo_id = 1
    driver.zero_offset = 4.58660255577807
    driver.moving_speed = 0.1
    driver.lower_sweep_angle = 1.75
    driver.upper_sweep_angle = 3.32
    driver.torque_enable = true
    
    dyn_type_cam = DynamixelType.new(:mId => 1, :mMode => 0)
    dyn_type_head = DynamixelType.new(:mId => 2, :mMode => 1)
    array = Array.new
    array << dyn_type_cam
    array << dyn_type_head

    driver.dynamixels = array

    puts "drivers: #{driver.dynamixels.size}" 

    driver.dynamixels.each do |element|
         puts "#{element}"
    end
    
    driver.num_resend = 2

    driver.configure
    driver.start

    reader_interface = driver.angle.reader()
    reader_dyn_cam = driver.angle_dyn1.reader()
    reader_dyn_head = driver.angle_dyn2.reader()

    # Get angles of the head.
    puts "Read angles"
    sample = nil
    for i in 0..10
        sample = reader_interface.read
        puts "Current head angle (requested via interface port) is %4.2f" % [sample]
        sample = reader_dyn_head.read
        puts "Current head angle (requested via dynamixel port) is %4.2f" % [sample]
        sample = reader_dyn_cam.read
        puts "Current cam angle (requested via dynamixel port) is %4.2f" % [sample]
        sleep 1    
    end
    
    # Control cam dynamixel.
    puts "Activate cam servo and read position"
    driver.setDynamixelActive(1)
    for i in 0..4
        sample = reader_interface.read
        puts "Current cam angle is (requested via interface port) %4.2f" % [sample]
        sleep 1        
    end

    # Change angle of cam.
    puts "Change position of the cam servo"
    writer = driver.cmd_angle.writer()
    writer.write(sample + 10/180.0*Math::PI)
    sleep 2
    writer.write(sample - 10/180.0*Math::PI)
    sleep 2
    writer.write(sample)
    sleep 2
end

