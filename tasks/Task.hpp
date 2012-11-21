#ifndef DYNAMIXEL_TASK_TASK_HPP
#define DYNAMIXEL_TASK_TASK_HPP

#include "dynamixel/TaskBase.hpp"
#include <dynamixel/dynamixel.h>

#include <map>

namespace dynamixel {

    /**
     * Required for the workaround to use several chained-up servos together with the
     * servo interface. The DynamixelDaisyChain structure contains all the informations
     * which should differ between the dynamixels.
     */
    struct DynamixelDaisyChainInternal : public DynamixelDaisyChain {
     public:
        double mTargetAngle;
    	double mLastAngle; 
        RTT::OutputPort<double>* mAngleOutputPort; 
        RTT::OutputPort<base::samples::RigidBodyState>* mUpper2LowerOutputPort;

        DynamixelDaisyChainInternal() : DynamixelDaisyChain(),
            mTargetAngle(0.0),
            mLastAngle(0.0),
            mAngleOutputPort(NULL),
            mUpper2LowerOutputPort(NULL)
        {
        }

        DynamixelDaisyChainInternal(int id, int mode) : DynamixelDaisyChain(id, mode),
            mTargetAngle(0.0),
            mLastAngle(0.0),
            mAngleOutputPort(NULL),
            mUpper2LowerOutputPort(NULL)
        {
        }

        DynamixelDaisyChainInternal(DynamixelDaisyChain& dyn) : DynamixelDaisyChain(dyn.mId, dyn.mMode),
            mTargetAngle(0.0),
            mLastAngle(0.0),
            mAngleOutputPort(NULL),
            mUpper2LowerOutputPort(NULL)
        {
        }

        void addOutputPorts(RTT::TaskContext* task_context) {
            std::stringstream port_name;
            if(mAngleOutputPort == NULL) {
                port_name << "angle_dyn" << mId;
                mAngleOutputPort = new RTT::OutputPort<double>(port_name.str());
                task_context->ports()->addPort(port_name.str(), *mAngleOutputPort);
                log(RTT::Info) << "Created output port: " << port_name.str() << RTT::endlog();
            }

            if(mUpper2LowerOutputPort == NULL) {
                port_name.clear();
                port_name << "upper2lower_dyn" << mId;
                mUpper2LowerOutputPort = new RTT::OutputPort<base::samples::RigidBodyState>(port_name.str());
                task_context->ports()->addPort(port_name.str(), *mUpper2LowerOutputPort);
                log(RTT::Info) << "Created output port: " << port_name.str() << RTT::endlog();
            }
        }

        bool writeAngle(double angle) {
            if(mAngleOutputPort != NULL) {
                mAngleOutputPort->write(angle); 
            } else {
                log(RTT::Warning) << "Angle have not been written, use addPorts() first" << RTT::endlog();
                return false;
            }   
            return true;
        }

        bool writeRigidBodyState(base::samples::RigidBodyState& rigid_body_state) {
            if(mUpper2LowerOutputPort != NULL) {
                mUpper2LowerOutputPort->write(rigid_body_state);
            } else {
                log(RTT::Warning) << "RigidBodyState have not been written, use addPorts() first" << RTT::endlog();
                return false;                
            }
            return true;
        }
    };

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	    struct Dynamixel::Configuration dynamixel_config;
	    Dynamixel dynamixel_;
	    base::samples::RigidBodyState lowerDynamixel2UpperDynamixel;
        std::map<int, DynamixelDaisyChainInternal> mDynamixels; // Contains the daisy-chained dynamixel informations.
        
        /**
         * Sets the passed position for the active dynamixel.
         */
	    bool set_angle(double angle);

        /**
         * Returns the position of the active dynamixel.
         */
	    double get_angle();
	    base::Time getTime();

	    uint16_t radToTicks(double angle) const;
	    double ticksToRad(uint16_t ticks) const;

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

