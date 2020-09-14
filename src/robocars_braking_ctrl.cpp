/**
 * @file robocars_braking_ctrl.cpp
 * @brief drive ESC speed accoridingly to current mode and orders received.
 * 
 * Copyright (c) 2020 Benoit TRINITE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Topic subscribed : 
 *  - /radio_channels : current braking value from radio
 *  - /robocars_brain_state : current car state
 *  - /robocars_actuator_ctrl_mode : nominal/calibration mode
 *  - /autopilot/braking : current braking decision from autopilot
 * Topic published :
 *  - /robocars_braking_ctrl/output : value sent actuator 
 * 
 * Parameters :
 *  - command_input_min : expected lowest value from radio channel : used to map value to -1
 *  - command_input_max : expected highest value from radio channel : used to map value to 1
 *  - command_output_min : expected lowest value for actuators : usually : 1000
 *  - command_output_max : expected highest value for actuators : usually : 2000
 *  - use_brake : use brake cycle to actively slow down car, ESC must support it
 *  - brake_cycle_ms : duration for brake cycle
 *  - loop_hz : main loop refresh freq.
 */


#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <algorithm> 
#include <cmath>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_actuator_ctrl_mode.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_autopilot_output.h>

#include <robocars_braking_ctrl.hpp>

RosInterface * ri;

static int command_input_min;
static int command_input_max;
static int command_output_min;
static int command_output_max;
static int loop_hz;

class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;
class onQualibtrateMode;

class onRunningMode
: public RobocarsStateMachine
{
    public:
        onRunningMode() : RobocarsStateMachine("onRunningMode"),__tick_count(0) {};
        onRunningMode(const char * subStateName) : RobocarsStateMachine(subStateName),__tick_count(0) {};


    protected:

        uint32_t __tick_count;
        
        void entry(void) override {
            RobocarsStateMachine::entry();
        };

        void react(ManualDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( AutonomousDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( IdleStatusEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(EnterQualibrateModeEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(TickEvent const & e) override {
            __tick_count++;
            if ((__tick_count%loop_hz)==0) {
                //Update param each second
                ri->updateParam(); 
            }
        };

};

class onQualibtrateMode
: public RobocarsStateMachine
{
    public:
        onQualibtrateMode() : RobocarsStateMachine("onQualibtrateMode") {};

    private:

        void entry(void) override {
            RobocarsStateMachine::entry();
            ri->initQualibration();
        };

        void react (LeaveQualibrateModeEvent const & e) override {
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        }

        void react (RadioChannelEvent const & e) override {
            ri->qualibrate(e.radio_channel_value); 
        };

        void react (TickEvent const & e) override {
            RobocarsStateMachine::react(e);
        };

};

class onIdle
: public onRunningMode
{
    public:
        onIdle() : onRunningMode("onArm") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };
  
        void react(ManualDrivingEvent const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            ri->maintainIdleActuator(); 
            onRunningMode::react(e);
        };

};

class onManualDriving
: public onRunningMode
{
    public:
        onManualDriving() : onRunningMode("onManualDriving") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
            transit<onAutonomousDriving>();
        }

        void react(IdleStatusEvent const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (RadioChannelEvent const & e) override {
            ri->controlActuatorFromRadio(e.radio_channel_value); 
        }

        void react (TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onAutonomousDriving
: public onRunningMode
{
    public:
        onAutonomousDriving() : onRunningMode("onAutonomousDriving") {};

    protected:

        virtual void entry(void) { 
            onRunningMode::entry();
        };  

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
        };

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (AutopilotEvent const & e) override {
            ri->controlActuatorFromAutopilot(e.autopilot_value); 
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle);


uint32_t mapRange(uint32_t in1,uint32_t in2,uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

_Float32 mapRange(_Float32 in1,_Float32 in2,_Float32 out1,_Float32 out2,_Float32 value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

void RosInterface::initParam() {
    if (!nh.hasParam("command_input_min")) {
        nh.setParam ("command_input_min", 363);       
    }
    if (!nh.hasParam("command_input_max")) {
        nh.setParam ("command_input_max", 1641);       
    }
    if (!nh.hasParam("command_output_min")) {
        nh.setParam ("command_output_min", 1000);       
    }
    if (!nh.hasParam("command_output_max")) {
        nh.setParam ("command_output_max", 2000);       
    }
    if (!nh.hasParam("loop_hz")) {
        nh.setParam ("loop_hz", 30);       
    }
}
void RosInterface::updateParam() {
    nh.getParam("command_input_min", command_input_min);
    nh.getParam("command_input_max", command_input_max);
    nh.getParam("command_output_min", command_output_min);
    nh.getParam("command_output_max", command_output_max);
    nh.getParam("loop_hz", loop_hz);
}

void RosInterface::initPub () {
    act_braking_pub = nh.advertise<robocars_msgs::robocars_actuator_output>("output", 1);
}

void RosInterface::initSub () {
    channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("/radio_channels", 1, &RosInterface::channels_msg_cb, this);
    state_sub = nh.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 1, &RosInterface::state_msg_cb, this);
    mode_sub = nh.subscribe<robocars_msgs::robocars_actuator_ctrl_mode>("/robocars_actuator_ctrl_mode", 1, &RosInterface::mode_msg_cb, this);
    autopilot_sub = nh.subscribe<robocars_msgs::robocars_autopilot_output>("/autopilot/braking", 1, &RosInterface::autopilot_msg_cb, this);

}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){    
    send_event(RadioChannelEvent(msg->channels[1]));
}

void RosInterface::autopilot_msg_cb(const robocars_msgs::robocars_autopilot_output::ConstPtr& msg) {
        send_event(AutopilotEvent(msg->norm));
}

void RosInterface::state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg) {
    static u_int32_t last_state = -1;
    if (msg->state != last_state) {
        switch (msg->state) {
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
                send_event(IdleStatusEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
                send_event(ManualDrivingEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
                send_event(AutonomousDrivingEvent());        
            break;
        }
        last_state=msg->state;
    }
    
}

void RosInterface::mode_msg_cb(const robocars_msgs::robocars_actuator_ctrl_mode::ConstPtr& msg) {
    if (msg->mode == robocars_msgs::robocars_actuator_ctrl_mode::ACTUATOR_MODE_QUALIBRATE) {
        send_event(EnterQualibrateModeEvent());        
    } else {
        send_event(LeaveQualibrateModeEvent());        
    }
}


void RosInterface::controlActuatorFromRadio (uint32_t braking_value) {

    robocars_msgs::robocars_actuator_output brakingMsg;
    float norm=0.0;
    brakingMsg.header.stamp = ros::Time::now();
    brakingMsg.header.seq=1;
    brakingMsg.header.frame_id = "mainBraking";
    norm = std::fmin((_Float32)0.0,mapRange((_Float32)command_input_min,(_Float32)command_input_max,-1.0,1.0,(_Float32)braking_value));
    if (norm>0.2) {
        brakingMsg.pwm = std::min((uint32_t)1500,mapRange(command_input_min,command_input_max,command_output_min,command_output_max,braking_value));
        brakingMsg.norm = norm;
    } else {
        brakingMsg.pwm = 1500;
        brakingMsg.norm = 0
    }
    act_braking_pub.publish(brakingMsg);
}

void RosInterface::controlActuatorFromAutopilot (_Float32 braking_value) {

    robocars_msgs::robocars_actuator_output brakingMsg;

    brakingMsg.header.stamp = ros::Time::now();
    brakingMsg.header.seq=1;
    brakingMsg.header.frame_id = "mainBraking";
    brakingMsg.pwm = std::min((uint32_t)1500,(uint32_t)mapRange(-1.0,1.0,(_Float32)command_output_min,(_Float32)command_output_max,braking_value));
    brakingMsg.norm = braking_value;

    act_braking_pub.publish(brakingMsg);
}

void RosInterface::maintainIdleActuator () {

    robocars_msgs::robocars_actuator_output brakingMsg;

    brakingMsg.header.stamp = ros::Time::now();
    brakingMsg.header.seq=1;
    brakingMsg.header.frame_id = "mainBraking";
    brakingMsg.pwm = 1500;
    brakingMsg.norm = 0.0;

    act_braking_pub.publish(brakingMsg);
}

void RosInterface::brakeActuator () {

    robocars_msgs::robocars_actuator_output brakingMsg;

    brakingMsg.header.stamp = ros::Time::now();
    brakingMsg.header.seq=1;
    brakingMsg.header.frame_id = "mainBraking";
    brakingMsg.pwm = 1000;
    brakingMsg.norm = -1.0;

    act_braking_pub.publish(brakingMsg);
}

void RosInterface::initQualibration() {
    command_input_min = 1500;
    command_input_max = 1500;
}

void RosInterface::qualibrate (uint32_t braking_value) {
    if (braking_value < command_input_min) {
        nh.setParam ("command_input_min", (int)braking_value);
        command_input_min = braking_value;
    }
    if (braking_value > command_input_max) {
        nh.setParam ("command_input_max", (int)braking_value);      
        command_input_max = braking_value;   
    }
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_braking_ctrl_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();
    ROS_INFO("Braking Ctrl: Starting");

    // wait for FCU connection
    ros::Rate rate(loop_hz);
    while(ros::ok()){
        ros::spinOnce();
        send_event (TickEvent());
        rate.sleep();
    }
}

