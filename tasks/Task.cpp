#include "Task.hpp"

#include <sstream>

#include <base/samples/rigid_body_state.h>


using namespace dynamixel;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{}

base::Time Task::getTime()
{
    return base::Time::now();
}

bool Task::set_angle(double angle)
{
    std::map<int, struct DynamixelDaisyChain>::iterator it = mDynamixels.find((int)dynamixel_.getActiveServo());
    if(it == mDynamixels.end()) {
        RTT::log(RTT::Warning) << "Servo with ID " << (int)dynamixel_.getActiveServo() 
                << "not available" << RTT::endlog();
        return false;
    }

    it->second.mTargetAngle = angle;
    uint16_t pos = radToTicks(target_angle + _zero_offset.value());
    
    //std::cout << "Setting Angle: " << angle/M_PI*180 << " in PositionMode " << (_mode.value() == POSITION) << std::endl;

    if(!dynamixel_.setGoalPosition(pos))
    {
	RTT::log(RTT::Error) << "setGoalPosition failed angle : " << angle << " offset: " 
            << _zero_offset.value() << " pos:" << pos << RTT::endlog();
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
	RTT::log(RTT::Error) << "getPresentPosition failed" << RTT::endlog();
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
	RTT::log(RTT::Warning) << "cw_slope range is 0-254" << RTT::endlog();
	return false;
    }

    if( _cw_margin.value() < 0 ||  _cw_margin.value() > 254 )
    {
	RTT::log(RTT::Warning) << "cw_margin range is 0-254" << RTT::endlog();
	return false;
    }

    if( _ccw_margin.value() < 0 ||  _ccw_margin.value() > 254)
    {
	RTT::log(RTT::Warning) << "ccw_margin range is 0-254" << RTT::endlog();
	return false;
    }

    if( _ccw_slope.value() < 0 ||  _ccw_slope.value() > 254)
    {
	RTT::log(RTT::Warning) << "ccw_slope range is 0-254" << RTT::endlog();
	return false;
    }

    if( _punch.value() < 32 ||  _punch.value() > 1023)
    {
	RTT::log(RTT::Warning) << "punch range is 32-1023" << RTT::endlog();
	return false;
    }

    dynamixel_config.mFilename= ( _device.value() );
    dynamixel_config.mBaudrate = 57600;
    dynamixel_.setTimeout(10000);

    if(!dynamixel_.init(&dynamixel_config))
    {
        RTT::log(RTT::Error) << "cannot open device '" << _device.value() << "'." << RTT::endlog();
        perror("errno is");

        return false;
    }

    // The single ID OR the list of IDs will be used. 
    std::vector<struct DynamixelDaisyChain> dynamixels_tmp = _dynamixels.get();
    if(dynamixels_tmp.empty()) {
        struct DynamixelDaisyChain dyn_tmp(_servo_id.get(), _mode.get());
        mDynamixels.insert( std::pair<int, struct DynamixelDaisyChain>(_servo_id.get(), dyn_tmp) );
        RTT::log(RTT::Info) << "Single dynamixel " << _servo_id.get() << " in mode " 
                << ((int)_mode.get() ? "SWEEP" : "POSITION") << " will be used" << RTT::endlog();
    } else { 
        // Fills the internal 'mDynamixels' map using the property 'dynamixels'.
        std::stringstream oss;
        oss << "Following daisy-chained dynamixels will be used:" << std::endl;
        for(unsigned int i=0; i < dynamixels_tmp.size(); ++i) {
            mDynamixels.insert( std::pair<int, struct DynamixelDaisyChain>(dynamixels_tmp[i].mId, dynamixels_tmp[i]) );
            oss << i << ": ID " << dynamixels_tmp[i].mId << ", Mode " << (dynamixels_tmp[i].mMode ? "SWEEP" : "POSITION") << std::endl;
            
            // Create ports upper2lower_dynX and angle_dynX where X is the ID of the dynamixel
            /*
            std::string angle_port_name = 
            RTT::OutputPort<double>* outputPort = new RTT::OutputPort<double>();
            taskContextSender->ports()->addPort(outputPortName, *outputPort);
            log(RTT::Info) << "CorbaConnection: created port: " << outputPortName << RTT::endlog();
            */
        } 
        RTT::log(RTT::Info) << oss.str() << RTT::endlog();
    }

    std::map<int, struct DynamixelDaisyChain>::iterator it = mDynamixels.begin();
    for(; it != mDynamixels.end(); ++it) {

    dynamixel_.addServo(it->first);
    dynamixel_.setServoActive(it->first);

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
        RTT::log(RTT::Error) << "readControlTable" << RTT::endlog();
        perror("errno is");
        return false;
    } 

    lowerDynamixel2UpperDynamixel.initSane();
    lowerDynamixel2UpperDynamixel.sourceFrame = _lowerFrameName.value();
    lowerDynamixel2UpperDynamixel.targetFrame = _upperFrameName.value();
    lowerDynamixel2UpperDynamixel.position.setZero();

    } // for-loop

    return true;
}

bool Task::startHook()
{
    if( !TaskBase::startHook() )
	return false;

    if (!dynamixel_.setControlTableEntry("Torque Limit", _torque_limit.value()))
	return false;

    return true;
}

void Task::updateHook()
{
    // Publish the present angle and the transformation (just) for the ACTIVE dynamixel!
    /////////////////// Start: copied from TaskBase::updateHook(); //////////////////
    double present_angle = get_angle();
    _angle.write( present_angle );
    
    //also report as transformation
    upper2lower.time = getTime();
    upper2lower.orientation = Eigen::AngleAxisd( - present_angle, _rotation_axis.value() );
    _upper2lower.write(upper2lower);
    /////////////////// Stop: copied from TaskBase::updateHook(); //////////////////

    // TODO: this is just for backward compatibility and should be removed soon
    lowerDynamixel2UpperDynamixel.time = base::Time::now();
    lowerDynamixel2UpperDynamixel.orientation = Eigen::AngleAxisd(-last_angle, Eigen::Vector3d::UnitX());
    _lowerDynamixel2UpperDynamixel.write(lowerDynamixel2UpperDynamixel);

    // Updates position for all dynamixels.
    int id_tmp = dynamixel_.getActiveServo();
    enum servo::MODE mode_tmp = _mode.get();

    std::map<int, struct DynamixelDaisyChain>::iterator it = mDynamixels.begin();
    // Calls the interface update hook for every sensor.
    for(; it != mDynamixels.end(); ++it) {
        target_angle = it->second.mTargetAngle;
        last_angle = it->second.mLastAngle;
        _mode.set((enum servo::MODE)it->second.mMode);
        dynamixel_.setServoActive(it->first);
        /////////////////// Start: copied from TaskBase::updateHook(); //////////////////
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
        /////////////////// Stop: copied from TaskBase::updateHook(); //////////////////

        it->second.mTargetAngle = target_angle;
        it->second.mLastAngle = last_angle;
    }
    // Restores the id which have been set while entering the update-hook.
    dynamixel_.setServoActive(id_tmp);
    _mode.set(mode_tmp);

    ServoBase::updateHook();
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

bool Task::setDynamixelActive(boost::int32_t servo_id) {
    // Returns pointer to servo if available, otherwise NULL.
    return (dynamixel_.setServoActive(servo_id) != NULL); 
    // TODO: Clear ports required?
}
