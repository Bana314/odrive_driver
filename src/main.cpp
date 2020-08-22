#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include "odrive_diff.h"

uint8_t drive_state = AXIS_STATE_IDLE;
int32_t prv_btn_state = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{   
    // If it was low and now its high
    if(prv_btn_state == 0 && joy->buttons[2] == 1){
        if(drive_state == AXIS_STATE_IDLE)
        {
            printf("[ODRIVE] Motors armed.\n");
            drive_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        }else
        {
            printf("[ODRIVE] Motors disarmed.\n");
            drive_state = AXIS_STATE_IDLE;
        }
        
    }
    prv_btn_state = joy->buttons[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odrive_driver");
    
    // Create and initialize driver
    odrive_diff robot;

    controller_manager::ControllerManager cm(&robot);

    ros::NodeHandle n;
    ros::Subscriber sub       = n.subscribe("joy", 1000, joyCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0);


    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        robot.read();
        cm.update(time, period);
        robot.write(drive_state);
        rate.sleep();
        robot.updateWD();
    }

    return 0;
}
