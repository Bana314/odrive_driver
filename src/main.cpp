#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include "odrive_diff.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "odrive_driver");
    
    // Create and initialize driver
    odrive_diff robot;

    controller_manager::ControllerManager cm(&robot);

    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        robot.read();
        cm.update(time, period);
        robot.write();
        rate.sleep();
        robot.updateWD();
    }

    return 0;
}
