#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace dynamixel;


RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
}




bool Task::set_scanner_tilt_angle(double angle)
{
    wanted_scanner_tilt_angle = angle;
    return true;
}

bool Task::isset_scanner_tilt_angleCompleted(double angle)
{
    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if( _scanner_tilt_max.value() <= _scanner_tilt_min.value() )
    {
	std::cerr << "allowable servo range is not valid." << std::endl;
	return false;
    }

    // get the id of the tilt servo
    int id = _scanner_tilt_id.value(); 

    dynamixel_.addServo(id);
    dynamixel_.setServoActive(id);

    dynamixel_config.mFilename= ( _device.value() );
    dynamixel_config.mBaudrate = 57600;
    dynamixel_.setTimeout(1000);

    if(!dynamixel_.init(&dynamixel_config))
    {
        std::cerr << "cannot open device '" << _device.value() << "'." << std::endl;
        perror("errno is");
	
        return false;
    }

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
    if( _scanner_tilt_angle_set.connected() )
    {
	_scanner_tilt_angle_set.read( wanted_scanner_tilt_angle );
    }

    if( _scanner_tilt_min > wanted_scanner_tilt_angle 
	    || _scanner_tilt_max.value() < wanted_scanner_tilt_angle )
    {
	std::cerr << "wanted servo pos " << wanted_scanner_tilt_angle << " is out of allowed range." << std::endl;
	error();
    }

    uint16_t pos_ = angleToDynamixel( wanted_scanner_tilt_angle );
    if(!dynamixel_.setGoalPosition(pos_))
    {
	std::cerr << "setGoalPosition" << std::endl;
	perror("errno is");
	error();
    }

    uint16_t present_pos_ = 0;
    if(!dynamixel_.getPresentPosition(&present_pos_) )
    {
	std::cerr << "getPresentPosition failed" << std::endl;
	perror("errno is");
	error();
    }
    //std::cout << pos_ << ":" << present_pos_ << std::endl;
    _scanner_tilt_angle.write( dynamixelToAngle(present_pos_) );
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
    //uint16_t pos_ = (uint16_t) (angle-30*M_PI/180) / (2.0*M_PI) * (0x3ff*360/300);
    int pos_ = 0x3ff * ( 6.0*angle / (10.0*M_PI) - 1.0/10.0 );
    return std::min( 0x3ff, std::max( 0, pos_ ) );
}

double Task::dynamixelToAngle( uint16_t pos )
{
    //double angle = pos * (2.0*M_PI) / (0x3ff*360/300) + 30*M_PI/180;
    double angle = (static_cast<double>(pos) / static_cast<double>(0x3ff) + 1.0/10.0) * (10.0*M_PI/6.0);
    return angle;
}

