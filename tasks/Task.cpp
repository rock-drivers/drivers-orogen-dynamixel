#include "Task.hpp"

#include <base/samples/rigid_body_state.h>


using namespace dynamixel;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{}

bool Task::setAngle(double angle)
{
    uint16_t pos_= radToTicks( angle );
    if(!dynamixel_.setGoalPosition(pos_))
    {
	std::cerr << "setGoalPosition" << std::endl;
	perror("errno is");
	exception(IO_ERROR);
	return false;
    }

    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
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

    zeroOffset = _zero_offset.value();

    if(!dynamixel_.readControlTable())
    {
        std::cerr << "readControlTable" << std::endl;
        perror("errno is");
        return 2;
    } 

    lowerDynamixel2UpperDynamixel.initSane();
    lowerDynamixel2UpperDynamixel.sourceFrame = _lowerFrameName.value();
    lowerDynamixel2UpperDynamixel.targetFrame = _upperFrameName.value();
    lowerDynamixel2UpperDynamixel.position.setZero();
    
    return true;
}

bool Task::startHook()
{
    return true;
}

void Task::updateHook()
{
    //first read position and report 
    uint16_t present_pos_ = 0;
    if(!dynamixel_.getPresentPosition(&present_pos_) )
    {
	std::cerr << "getPresentPosition failed" << std::endl;
	perror("errno is");
	exception(IO_ERROR);
	return;
    }
    
    double present_angle = ticksToRad(present_pos_) - zeroOffset;
    _angle.write( present_angle );
    
    //also report as transformation
    lowerDynamixel2UpperDynamixel.orientation = Eigen::AngleAxisd(present_angle, Eigen::Vector3d::UnitX());
    _lowerDynamixel2UpperDynamixel.write(lowerDynamixel2UpperDynamixel);
    
	    //return if no angle was set and in mode 0
	    if(_cmd_angle.readNewest( wanted_scanner_tilt_angle ) == RTT::NoData)
		return;

    uint16_t pos = radToTicks(wanted_scanner_tilt_angle + zeroOffset);
    if(!dynamixel_.setGoalPosition(pos))
    {
	std::cerr << "setGoalPosition failed" << std::endl;
	perror("errno is");
	exception(IO_ERROR);
	return;
    }    
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


uint16_t Task::radToTicks(double angle) const
{
    if(angle < 0.0)
        return 0;

    if(angle > (300.0/360.0*M_PI*2))
        return 1024;

    //(300.0/360.0*M_PI*2) == maximum moving area of servo (300 degees)
    return angle * 1024.0 / (300.0/360.0*M_PI*2);  
}

double Task::ticksToRad(uint16_t ticks) const
{
    return ticks * (300.0/360.0*M_PI*2) /1024.0;
}
