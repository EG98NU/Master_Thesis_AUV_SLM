#ifndef ZENO_MODEL_H
#define ZENO_MODEL_H

#include <ros/ros.h>
#include <cmath>

namespace zeno_model
{

// Vehicle physical parameters
struct VehicleParams
{
  double mass;              // [kg] Vehicle mass
  double linear_damping;    // [N*s/m] Surge damping coefficient
  double lateral_damping;   // [N*s/m] Sway damping coefficient
  double inertia;           // [kg*m^2] Moment of inertia (yaw axis)
  double yaw_damping;       // [N*m*s/rad] Yaw damping coefficient

  VehicleParams() :
    mass(44.0),
    linear_damping(1.0),
    lateral_damping(5.0),
    inertia(6.4),
    yaw_damping(1.0)
  {}

  void loadFromROS(ros::NodeHandle& nh)
  {
    nh.param("mass", mass, 44.0);
    nh.param("linear_damping", linear_damping, 1.0);
    nh.param("lateral_damping", lateral_damping, 5.0);
    nh.param("inertia", inertia, 6.4);
    nh.param("yaw_damping", yaw_damping, 1.0);
  }
};

// Control parameters
struct ControlParams
{
  double K_yaw_P;      // Yaw proportional gain
  double K_yaw_D;      // Yaw derivative gain
  double K_surge_P;    // Surge proportional gain
  double K_sway_P;     // Sway proportional gain
  double K_depth_P;    // Depth proportional gain

  ControlParams() :
    K_yaw_P(30.0),
    K_yaw_D(30.0),
    K_surge_P(100.0),
    K_sway_P(100.0),
    K_depth_P(0.5)
  {}

  void loadFromROS(ros::NodeHandle& nh)
  {
    nh.param("K_yaw_P", K_yaw_P, 30.0);
    nh.param("K_yaw_D", K_yaw_D, 30.0);
    nh.param("K_surge_P", K_surge_P, 100.0);
    nh.param("K_sway_P", K_sway_P, 100.0);
    nh.param("K_depth_P", K_depth_P, 0.5);
  }
};

// Initial state parameters
struct InitialState
{
  double latitude;   // [deg] Initial latitude
  double longitude;  // [deg] Initial longitude
  double depth;      // [m] Initial depth
  double yaw;        // [rad] Initial yaw angle

  InitialState() :
    latitude(44.09588219),
    longitude(9.86493751),
    depth(0.0),
    yaw(0.0)
  {}

  void loadFromROS(ros::NodeHandle& nh)
  {
    nh.param("start_latitude", latitude, 44.09588219);
    nh.param("start_longitude", longitude, 9.86493751);
    nh.param("start_depth", depth, 0.0);
    nh.param("start_yaw", yaw, 0.0);
  }
};

// Simulation parameters
struct SimulationParams
{
  double integration_step;  // [s] Time step for integration
  double rate;              // [Hz] Publishing rate

  SimulationParams() :
    integration_step(0.1),
    rate(10.0)
  {}

  void loadFromROS(ros::NodeHandle& nh)
  {
    nh.param("integration_step", integration_step, 0.1);
    nh.param("rate", rate, 10.0);
  }
};

} // namespace zeno_model

#endif // ZENO_MODEL_H

//////////////////////////////////////////////////////////////////////////////////////////////////
// // ZENO JOYSTICK

// #ifndef MM_BT_ZENO_JOYSTICK_H
// #define MM_BT_ZENO_JOYSTICK_H

// #include <ros/ros.h>
// #include <raasl_toolbox/template.h>
// #include <raasl_toolbox/geodetic.h>
// #include <raasl_toolbox/angles.h>
// #include <sensor_msgs/Joy.h>
// #include <marta_msgs/NavStatus.h>
// #include <geometry_msgs/Vector3.h>
// #include <Eigen/Dense>

// class ZenoJoystick : public RAASL::NodeHandler::NodeTopicClass {
// public:
//     // Constructor
//     ZenoJoystick(float rate, bool verbose = false);

//     // Subscriber for joystick commands
//     ros::Subscriber sub_joystick;

//     // Publisher for navigation status
//     ros::Publisher pub_nav_status;

//     // Flags for node management
//     bool subscribed {false};
//     bool joystick_received {false};
//     bool verbose {false};
//     bool initialized {false};

//     // Node name variables
//     std::string node_name {getNodeName()};
//     std::string node_name_complete {getNodeNameComplete()};
//     std::string node_namespace {getNodeNamespace()};

//     // Joystick callback
//     void joystickCallback(const sensor_msgs::JoyConstPtr& msg);

// private:
//     // State management functions (must be implemented)
//     bool on_activate() override;
//     void on_deactivate() override;
//     void on_active() override;

//     // Vehicle state variables
//     double current_latitude;   // [deg]
//     double current_longitude;  // [deg]
//     double current_depth;      // [m]
//     double current_roll;       // [rad]
//     double current_pitch;      // [rad]
//     double current_yaw;        // [rad]

//     // Vehicle velocity in NED frame
//     Eigen::Vector3d ned_speed;  // [m/s]
//     Eigen::Vector3d omega_body; // [rad/s]

//     // Initial vehicle state (from parameters)
//     double start_latitude;
//     double start_longitude;
//     double start_depth;
//     double start_yaw;

//     // Control parameters
//     double surge_speed_multiplier;  // Speed for each axis increment
//     double sway_speed_multiplier;
//     double yaw_rate;               // [rad/s]
//     double depth_rate;             // [m/s]
//     double pitch_rate;             // [rad/s]

//     // Last joystick message
//     sensor_msgs::Joy last_joy_msg;
//     ros::Time last_update_time;

//     // Helper functions
//     void loadParameters();
//     void initializeVehicleState();
//     void updateVehicleState(double dt);
//     void publishNavStatus();
// };

// #endif // MM_BT_ZENO_JOYSTICK_H