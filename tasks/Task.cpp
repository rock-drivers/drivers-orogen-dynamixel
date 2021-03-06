#include "Task.hpp"

#include <base/samples/rigid_body_state.h>


using namespace dynamixel;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _rotation_axis.value() = base::Vector3d::UnitX();
}

Task::~Task()
{}

base::Time Task::getTime()
{
    return base::Time::now();
}

bool Task::set_angle(double angle)
{
    target_angle = angle;
    uint16_t pos = radToTicks(target_angle + _zero_offset.value());
    
    //std::cout << "Setting Angle: " << angle/M_PI*180 << " in PositionMode " << (_mode.value() == POSITION) << std::endl;

    if(!dynamixel_.setGoalPosition(pos))
    {
	std::cerr << "setGoalPosition failed angle : " << angle << " offset: " << _zero_offset.value() << " pos:" << pos << std::endl;
	perror("errno is");
	exception(IO_ERROR);
	return false;
    }    
    return true;
}

double Task::get_angle()
{
    //first read position and report 
    uint16_t present_pos_ = 0;
    if(!dynamixel_.getPresentPosition(&present_pos_) )
    {
	std::cerr << "getPresentPosition failed" << std::endl;
	perror("errno is");
	exception(IO_ERROR);
	return 0.0;
    }
    double present_angle = ticksToRad(present_pos_) - _zero_offset.value();

    return present_angle;
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
    if (!dynamixel_.setControlTableEntry("CW Compliance Slope", _cw_slope.value()))
	return false;
    if (!dynamixel_.setControlTableEntry("CW Compliance Margin", _cw_margin.value()))
	return false;
    if (!dynamixel_.setControlTableEntry("CCW Compliance Margin", _ccw_margin.value()))
	return false;
    if (!dynamixel_.setControlTableEntry("CCW Compliance Slope", _ccw_slope.value()))
	return false;
    if (!dynamixel_.setControlTableEntry("Punch", _punch.value()))
	return false;

    if (!dynamixel_.setControlTableEntry("CW Angle Limit", 0))
	return false;
    if(!dynamixel_.setControlTableEntry("CCW Angle Limit", 1023))
	return false;
    if(!dynamixel_.setControlTableEntry("Torque Enable", 0))
	return false;

    uint16_t present_pos_ = 0;
    //set current position, so the servo won't move to old values from previous runs
    if(!dynamixel_.getPresentPosition(&present_pos_) )
    {
	return false;
    }

    if(!dynamixel_.setGoalPosition(present_pos_))
    {
	return false;
    }

    //set speed
    //0.111 is the factor for converting to RPM according to the manual
    //RPM / 0.111 = dyna
    int speed_dyna = _moving_speed.value() / M_PI  * 60  / 0.111;
    std::cout << "Moving speed is " << _moving_speed.value() << " " << speed_dyna << std::endl;
    if(speed_dyna <= 0)
	speed_dyna = 1;

    if(speed_dyna > (1<<23))
	speed_dyna = (1<<23);

    if(!dynamixel_.setControlTableEntry("Moving Speed", speed_dyna))
	return false;

    if(!dynamixel_.readControlTable())
    {
        std::cerr << "readControlTable" << std::endl;
        perror("errno is");
        return false;
    } 

    lowerDynamixel2UpperDynamixel.initSane();
    lowerDynamixel2UpperDynamixel.sourceFrame = _lowerFrameName.value();
    lowerDynamixel2UpperDynamixel.targetFrame = _upperFrameName.value();
    lowerDynamixel2UpperDynamixel.position.setZero();
    
    return true;
}

bool Task::startHook()
{
    if( !TaskBase::startHook() )
	return false;
    
    upper2lower.initSane();
    upper2lower.position.setZero();
    upper2lower.sourceFrame = _upper_frame.value();
    upper2lower.targetFrame = _lower_frame.value();

    if (!dynamixel_.setControlTableEntry("Torque Limit", _torque_limit.value()))
	return false;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    double present_angle = get_angle();
    _angle.write( present_angle );
    
    //also report as transformation
    upper2lower.time = getTime();
    upper2lower.orientation = Eigen::AngleAxisd( -present_angle, _rotation_axis.value() );
    _upper2lower.write(upper2lower);

    switch(_mode.value())
    {
	case servo::POSITION:
	    //return if no angle was set and in mode 0
	    if(_cmd_angle.readNewest( target_angle ) == RTT::NoData)
	    {
		return;
	    }
	    set_angle(target_angle);

	    break;
	case servo::SWEEP:
	    if(fabs(last_angle - present_angle) < .2/180*M_PI)
	    {
		if(fabs(present_angle - _upper_sweep_angle.value()) < 4.0/180*M_PI 
		   && target_angle == _upper_sweep_angle.value())
		{
		    target_angle = _lower_sweep_angle.value();
		    set_angle(target_angle);
		}
   
		if(fabs(present_angle - _lower_sweep_angle.value()) < 4.0/180*M_PI 
		   && target_angle == _lower_sweep_angle.value())
		{
		    target_angle = _upper_sweep_angle.value();
		    set_angle(target_angle);
		}
	    }
	    
	    if(target_angle != _lower_sweep_angle.value() &&
	       target_angle != _upper_sweep_angle.value())
	    {
		target_angle = _upper_sweep_angle.value();
		set_angle(target_angle);
	    }
	    //note sending target angle over and over results in 
	    //non fluent movement of the servo in speed mode
	    break;
    }
    last_angle = present_angle;

    // TODO: this is just for backward compatibility and should be removed soon
    lowerDynamixel2UpperDynamixel.time = base::Time::now();
    lowerDynamixel2UpperDynamixel.orientation = Eigen::AngleAxisd(-last_angle, Eigen::Vector3d::UnitX());
    _lowerDynamixel2UpperDynamixel.write(lowerDynamixel2UpperDynamixel);
}

void Task::stopHook()
{
    TaskBase::stopHook();

    //turn off servo
    dynamixel_.setControlTableEntry("Torque Limit", 0);
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
