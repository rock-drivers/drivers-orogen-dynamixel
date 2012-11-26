#ifndef DYNAMIXEL_DAISY_CHAIN_HPP
#define DYNAMIXEL_DAISY_CHAIN_HPP

#include <orocos/interfaces/types/interfaces/interfacesTypes.hpp>

namespace dynamixel
{
    /**
     * Workaround to use several chained up dynamixels together with the servo interface,
     * which is designed to represent just a single servo.
     */
    struct DynamixelDaisyChain{
     public:
        DynamixelDaisyChain() :
            mId(0)
        {
            mMode = (int)servo::POSITION;
        }

        /**
         * \param id ID which is set on the dynamixel.
         * \param mode Use servo::POSITION or servo::SWEEP.
         */
        DynamixelDaisyChain(int id, int mode) :
            mId(id),
            mMode(mode)
        {
        }
    
        int mId;
        int mMode; // servo::POSITION=0 or servo::SWEEP=1
    };
}

#endif

