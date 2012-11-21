#ifndef DYNAMIXEL_TASK_TASK_HPP
#define DYNAMIXEL_TASK_TASK_HPP

#include "dynamixel/TaskBase.hpp"
#include <dynamixel/dynamixel.h>

#include <map>

namespace dynamixel {

    /**
     * Required for the workaround to use several chained up servos together with the
     * servo interface.
     */
    struct ServoInformations {
     public:
        ServoInformations() :
            mTargetAngle(0.0),
            mLastAngle(0.0) 
        {
        }

        ServoInformations(double target_angle, double last_angle) : 
                mTargetAngle(target_angle),
                mLastAngle(last_angle) 
        {
        }

        double mTargetAngle;
    	double mLastAngle;
    };

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
    
	struct Dynamixel::Configuration dynamixel_config;
	Dynamixel dynamixel_;
    
	bool set_angle(double angle);
	double get_angle();
	base::Time getTime();

	uint16_t radToTicks(double angle) const;
	double ticksToRad(uint16_t ticks) const;

	base::samples::RigidBodyState lowerDynamixel2UpperDynamixel;

    std::map<int, struct DynamixelDaisyChain> mDynamixels; // Contains the daisy chained dynamixels.

    public:
        Task(std::string const& name = "dynamixel::Task");
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** 
         * Unfortunately the content of TaskBase::updateHook() had to be copied.
         * Otherwise the daisy chained dynamixels would publish their angle and their
         * transformation one after another on the same port.
         * Now only the active dynamixel uses the ports.
         */
        void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();

        bool setDynamixelActive(boost::int32_t servo_id);
    };
}

#endif

