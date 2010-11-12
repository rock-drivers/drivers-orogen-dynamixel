#include "Task.hpp"

using namespace dynamixel;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

bool Task::setAngle(double angle)
{
    uint16_t pos_= angleToDynamixel( angle );
    if(!dynamixel_.setGoalPosition(pos_))
    {
	std::cerr << "setGoalPosition" << std::endl;
	perror("errno is");
	fatal(IO_ERROR);
	return false;
    }

    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if( _angle_to_servo_factor.value() == 0.0 )
    {
	std::cerr << "factor can not be 0.0" << std::endl;
	return false;
    }

    if( _cw_slope.value() < 0 ||  _cw_slope.value() > 254)
    {
	std::cerr << "cw_slope range is 0-254" << std::endl;
	return false;
    }

    if( _cw_margin.value() < 0 ||  _cw_margin.value() > 254 )
    {
	std::cerr << "cw_margin range is 0-254" << std::endl;
	return false;
    }

    if( _ccw_margin.value() < 0 ||  _ccw_margin.value() > 254)
    {
	std::cerr << "ccw_margin range is 0-254" << std::endl;
	return false;
    }

    if( _ccw_slope.value() < 0 ||  _ccw_slope.value() > 254)
    {
	std::cerr << "ccw_slope range is 0-254" << std::endl;
	return false;
    }

    if( _punch.value() < 32 ||  _punch.value() > 1023)
    {
	std::cerr << "punch range is 32-1023" << std::endl;
	return false;
    }

    // get the id of the tilt servo
    int id = _servo_id.value(); 

    dynamixel_.addServo(id);
    dynamixel_.setServoActive(id);

    dynamixel_config.mFilename= ( _device.value() );
    dynamixel_config.mBaudrate = 57600;
    dynamixel_.setTimeout(10000);




    if(!dynamixel_.init(&dynamixel_config))
    {
        std::cerr << "cannot open device '" << _device.value() << "'." << std::endl;
        perror("errno is");
	
        return false;
    }

    // set control value A,B,C,D,E (see RX-28 manual)
    dynamixel_.setControlTableEntry("CW Compliance Slope", _cw_slope.value());
    dynamixel_.setControlTableEntry("CW Compliance Margin", _cw_margin.value());
    dynamixel_.setControlTableEntry("CCW Compliance Margin", _ccw_margin.value());
    dynamixel_.setControlTableEntry("CCW Compliance Slope", _ccw_slope.value());
    dynamixel_.setControlTableEntry("Punch", _punch.value());



    if(!dynamixel_.readControlTable())
    {
        std::cerr << "readControlTable" << std::endl;
        perror("errno is");
        return 2;
    } 

    return true;
}

bool Task::startHook()
{
    return true;
}

void Task::updateHook()
{
    if( _cmd_angle.connected() )
    {
        double angle;
	if (_cmd_angle.read( angle ) == RTT::NewData)
            setAngle(angle);
    }

    uint16_t present_pos_ = 0;
    if(!dynamixel_.getPresentPosition(&present_pos_) )
    {
	std::cerr << "getPresentPosition failed" << std::endl;
	perror("errno is");
	fatal(IO_ERROR);
	return;
    }
    _angle.write( dynamixelToAngle(present_pos_) );
}

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }


uint16_t Task::angleToDynamixel( double angle ) 
{
    int pos_ = angle * _angle_to_servo_factor.value() + _angle_to_servo_offset.value();
    return std::min( 0x3ff, std::max( 0, pos_ ) );
}

double Task::dynamixelToAngle( uint16_t pos )
{
    double angle = (static_cast<double>(pos) - _angle_to_servo_offset.value() ) / _angle_to_servo_factor.value();
    return angle;
}

