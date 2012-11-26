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
    std::map<int, struct DynamixelDaisyChainInternal>::iterator it = mDynamixels.find((int)dynamixel_.getActiveServo());
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
    
    if(_num_resend.get() > 0) {
        dynamixel_.setNumberRetries(_num_resend.get());
    }

    // Fills the dynamixel-map according to the properties servo_id, mode and dynamixels.
    std::vector<struct DynamixelDaisyChain> dynamixels_tmp = _dynamixels.get();
    
    if(dynamixels_tmp.empty()) { // Just a single servo is used, the default behaviour if the servo interface will be used.
        struct DynamixelDaisyChainInternal dyn_tmp(_servo_id.get(), _mode.get());
        mDynamixels.insert( std::pair<int, struct DynamixelDaisyChainInternal>(_servo_id.get(), dyn_tmp) ); 
        RTT::log(RTT::Info) << "Single dynamixel with ID " << _servo_id.get() << " will be used" << RTT::endlog();
    } else { // Several chained up dynamixels will be used, additional ports for the current angle and transformation will be created.
        std::stringstream oss;
        oss << "The following daisy-chained dynamixels will be used:" << std::endl;
        for(unsigned int i=0; i < dynamixels_tmp.size(); ++i) {
            struct DynamixelDaisyChainInternal dyn_tmp(dynamixels_tmp[i]);
            dyn_tmp.addOutputPorts(this); // Adds additional output ports for every dynamixel.
            mDynamixels.insert( std::pair<int, struct DynamixelDaisyChainInternal>(dyn_tmp.mId, dyn_tmp));
            oss << dyn_tmp.mId << " ";
        } 
        oss << std::endl;
        RTT::log(RTT::Info) << oss.str() << RTT::endlog();
    }

    std::map<int, struct DynamixelDaisyChainInternal>::iterator it = mDynamixels.begin();
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

    int active_id = dynamixel_.getActiveServo();
    std::map<int, struct DynamixelDaisyChainInternal>::iterator it = mDynamixels.begin();
    for(; it != mDynamixels.end(); ++it) {
        dynamixel_.setServoActive(it->first);
        if (!dynamixel_.setControlTableEntry("Torque Limit", _torque_limit.value())) {
    	    return false;
        }
    }
    dynamixel_.setServoActive(active_id);
    return true;
}

void Task::updateHook()
{
    _lowerDynamixel2UpperDynamixel.write(lowerDynamixel2UpperDynamixel);

    // Updates position for all dynamixels.
    int id_tmp = dynamixel_.getActiveServo();
    enum servo::MODE mode_tmp = _mode.get();

    std::map<int, struct DynamixelDaisyChainInternal>::iterator it = mDynamixels.begin();
    // Calls the interface update hook for every sensor.
    for(; it != mDynamixels.end(); ++it) {
        target_angle = it->second.mTargetAngle;
        last_angle = it->second.mLastAngle;
        _mode.set((enum servo::MODE)it->second.mMode);
        dynamixel_.setServoActive(it->first);
   
        TaskBase::updateHook(); // Sets last_angle to present_angle.

        // Updates the dynamixel ports.
        lowerDynamixel2UpperDynamixel.time = base::Time::now();
        lowerDynamixel2UpperDynamixel.orientation = Eigen::AngleAxisd(-last_angle, _rotation_axis.value());
        it->second.writeRigidBodyState(lowerDynamixel2UpperDynamixel);
        it->second.writeAngle(last_angle);
        
        it->second.mTargetAngle = target_angle;
        it->second.mLastAngle = last_angle;
    }
    // Restores the id which have been set while entering the update-hook.
    dynamixel_.setServoActive(id_tmp);
    _mode.set(mode_tmp);
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // Turn off servos.
    int active_id = dynamixel_.getActiveServo();
    std::map<int, struct DynamixelDaisyChainInternal>::iterator it = mDynamixels.begin();
    for(; it != mDynamixels.end(); ++it) {
        dynamixel_.setServoActive(it->first);
        dynamixel_.setControlTableEntry("Torque Limit", 0);
    }
    dynamixel_.setServoActive(active_id);
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
