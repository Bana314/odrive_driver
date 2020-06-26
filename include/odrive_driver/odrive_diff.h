#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include "odrive_driver//OdriveConfig.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ros_odrive/odrive.hpp"

#include <string> 


class odrive_diff : public hardware_interface::RobotHW {
public:
    odrive_diff();
    ~odrive_diff();

    int init();

    void read();
    void write();
    void updateWD();

    std::string od_cfg;

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::Float64 pos;
        std_msgs::Float64 vel;
        std_msgs::Float64 eff;
        std_msgs::Float64 cmd;
    } joints[2];



    /******************************/
    // ODRIVE USB SETUP VARIABLES //
    Json::Value odrive_json;
    bool targetJsonValid = false;
    odrive_endpoint *endpoint = NULL;

    /******************************/

    double wheel_radius;
    double wheel_separation;

    uint16_t u16val;
    uint8_t u8val;
    float fval;

    uint8_t connected;

    std::string cmd;

    std_msgs::Float64 vbus;     // Bus voltage
    int32_t error0; // Axis 0 error
    uint8_t state0; // Axis 0 state
    float vel0;     // Axis 0 velocity 
    float pos0;     // Axis 0 encoder position
    float curr0B;   // Axis 0 motor channel B current
    float curr0C;   // Axis 0 motor channel C current
    float temp0;    // Axis 0 invertr temperature

    int32_t error1; // Axis 1 error
    uint8_t state1; // Axis 1 state
    float vel1;     // Axis 1 velocity 
    float pos1;     // Axis 1 encoder position
    float curr1B;   // Axis 1 motor channel B current
    float curr1C;   // Axis 1 motor channel C current
    float temp1;    // Axis 1 inverter temperature

    std::string od_sn;

    double m_Dc;
    double m_Dr;
    double m_Dl;
    double m_x;
    double m_y;
    double m_phi;

    double m_tickr;
    double m_tickl;

    ros::Time current_time;
    ros::Time last_read;
    ros::Time startTime;

    const int encoder_CPR = 90;

    // For debug purposes only
    ros::NodeHandle nh;
    // ros::Publisher left_pos_pub, right_pos_pub;
    // ros::Publisher left_vel_pub, right_vel_pub;
    // ros::Publisher left_eff_pub, right_eff_pub;
    // ros::Publisher left_cmd_pub, right_cmd_pub;
    
    
    ros::Publisher vbus_pub;
    ros::Publisher odom_encoder_pub;

    tf::TransformBroadcaster odom_broadcaster;

    // Supporting dynamic reconfigure for PID control
    dynamic_reconfigure::Server<odrive_driver::OdriveConfig> *dsrv;
    void reconfigure_callback(odrive_driver::OdriveConfig& config, uint32_t level);
    odrive_driver::OdriveConfig config;
    bool have_config = false;
};