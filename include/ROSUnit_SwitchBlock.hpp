#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include <std_msgs/Bool.h>
#include "BoolMessage.hpp"
#include "Vector3D.hpp"
#include "SwitchBlockMsg.hpp"
#include "positioning_system/SwitchBlock.h"

class ROSUnit_SwitchBlock :  public ROSUnit{

    private:
        static ROSUnit_SwitchBlock* _instance_ptr;
        static SwitchBlockMsg _switch_msg; 
        ros::ServiceServer _srv_switch;
        static bool callbackSwitchBlocks(positioning_system::SwitchBlock::Request  &req,
                                         positioning_system::SwitchBlock::Response &res);
    
    public:
        void receive_msg_data(DataMessage* t_msg); 
        ROSUnit_SwitchBlock(ros::NodeHandle&);
        ~ROSUnit_SwitchBlock();
};