#include <ros/ros.h>
#include <angles/angles.h>

// Custom header
#include "mm_bt/zeno_model.h"

// RAASL Toolbox - Geodetic conversions
#include <raasl_toolbox/geodetic.h>
#include <raasl_toolbox/angles.h>

// Message includes
#include <marta_msgs/NavStatus.h>
#include <joystick_command/Rel_error_joystick.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

class ZenoJoystickModel
{
public:
  ZenoJoystickModel() : nh_("~")
  {
    // Load all parameters from ROS parameter server
    vehicle_params_.loadFromROS(nh_);
    control_params_.loadFromROS(nh_);
    initial_state_.loadFromROS(nh_);
    sim_params_.loadFromROS(nh_);


    // Load NED origin from parameters
    nh_.param("ned_origin_latitude", ned_origin_lat_, 44.09613064612619);
    nh_.param("ned_origin_longitude", ned_origin_lon_, 9.865068083462063);
    nh_.param("ned_origin_depth", ned_origin_depth_, 0.0);

    // Store NED origin as Eigen vector
    ned_origin_ = Eigen::Vector3d(ned_origin_lat_, ned_origin_lon_, ned_origin_depth_);

    // Initialize state
    initializeState();

    // Publishers
    nav_status_pub_ = nh_.advertise<marta_msgs::NavStatus>("/nav_status", 10);

    // Subscribers - subscribe to /relative_error (same as joystick_command)
    joystick_sub_ = nh_.subscribe("/relative_error", 10,
                                  &ZenoJoystickModel::joystickCallback, this);

    // Timer for dynamics update
    timer_ = nh_.createTimer(ros::Duration(sim_params_.integration_step),
                             &ZenoJoystickModel::updateDynamics, this);

    ROS_INFO("Zeno Joystick Model initialized");
    ROS_INFO("  NED Origin: lat=%.8f, lon=%.8f, depth=%.2f",
             ned_origin_lat_, ned_origin_lon_, ned_origin_depth_);
    ROS_INFO("  Start position: lat=%.6f, lon=%.6f, depth=%.2f, yaw=%.2f rad",
             initial_state_.latitude, initial_state_.longitude,
             initial_state_.depth, initial_state_.yaw);
    printParameters();
  }

private:
  void initializeState()
  {
    // Convert initial lat/lon/depth to NED using RAASL::Geo
    Eigen::Vector3d initial_pos(initial_state_.latitude, initial_state_.longitude, initial_state_.depth);
    Eigen::Vector3d ned_pos = RAASL::Geo::LatLonDepth2NED(ned_origin_, initial_pos);

    // Position in NED local frame (meters)
    x_ = ned_pos.x();
    y_ = ned_pos.y();
    depth_ = initial_state_.depth;

    // Orientation (radians)
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = initial_state_.yaw;

    // Velocities in body frame
    u_ = 0.0;  // surge
    v_ = 0.0;  // sway
    w_ = 0.0;  // heave

    // Angular velocities in body frame
    p_ = 0.0;  // roll rate
    q_ = 0.0;  // pitch rate
    r_ = 0.0;  // yaw rate

    // Desired velocities from joystick
    u_des_ = 0.0;
    v_des_ = 0.0;
    yaw_des_ = initial_state_.yaw;
    depth_des_ = initial_state_.depth;

    ROS_INFO("Initial NED position: x=%.2f m, y=%.2f m, depth=%.2f m", x_, y_, depth_);
  }

  void printParameters()
  {
    ROS_INFO("Vehicle Parameters:");
    ROS_INFO("  Mass: %.2f kg", vehicle_params_.mass);
    ROS_INFO("  Linear damping: %.2f N*s/m", vehicle_params_.linear_damping);
    ROS_INFO("  Lateral damping: %.2f N*s/m", vehicle_params_.lateral_damping);
    ROS_INFO("  Inertia: %.2f kg*m^2", vehicle_params_.inertia);
    ROS_INFO("  Yaw damping: %.2f N*m*s/rad", vehicle_params_.yaw_damping);
    ROS_INFO("Control Gains:");
    ROS_INFO("  K_yaw_P: %.2f", control_params_.K_yaw_P);
    ROS_INFO("  K_yaw_D: %.2f", control_params_.K_yaw_D);
    ROS_INFO("  K_surge_P: %.2f", control_params_.K_surge_P);
    ROS_INFO("  K_sway_P: %.2f", control_params_.K_sway_P);
  }

  void joystickCallback(const joystick_command::Rel_error_joystick::ConstPtr& msg)
  {
    // Extract desired velocities from joystick command
    u_des_ = msg->error_surge_speed;  // [m/s] desired surge velocity
    v_des_ = msg->error_sway_speed;   // [m/s] desired sway velocity

    // Desired yaw is the relative yaw error (convert from deg to rad)
    double yaw_error_rad = msg->error_yaw * M_PI / 180.0;
    yaw_des_ = angles::normalize_angle(yaw_ + yaw_error_rad);

    // Desired depth from error
    depth_des_ = depth_ + msg->error_depth;

    // Clamp depth to reasonable values
    depth_des_ = std::max(0.0, std::min(depth_des_, 50.0));
  }

  void updateDynamics(const ros::TimerEvent&)
  {
    double dt = sim_params_.integration_step;

    // CONTROL: Calculate forces/moments
    double error_surge = u_des_ - u_;
    double tau_u = control_params_.K_surge_P * error_surge;

    double error_sway = v_des_ - v_;
    double tau_v = control_params_.K_sway_P * error_sway;

    double error_yaw = angles::normalize_angle(yaw_des_ - yaw_);
    double tau_r = control_params_.K_yaw_P * error_yaw -
                   control_params_.K_yaw_D * r_;

    double error_depth = depth_des_ - depth_;
    w_ = control_params_.K_depth_P * error_depth;

    // DYNAMICS: Update velocities
    double u_dot = (-vehicle_params_.linear_damping * u_ + tau_u) / vehicle_params_.mass;
    u_ += u_dot * dt;

    double v_dot = (-vehicle_params_.lateral_damping * v_ + tau_v) / vehicle_params_.mass;
    v_ += v_dot * dt;

    double r_dot = (-vehicle_params_.yaw_damping * r_ + tau_r) / vehicle_params_.inertia;
    r_ += r_dot * dt;

    // KINEMATICS: Update position and orientation
    yaw_ += r_ * dt;
    yaw_ = angles::normalize_angle(yaw_);

    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);

    double x_dot = u_ * cos_yaw - v_ * sin_yaw;
    double y_dot = u_ * sin_yaw + v_ * cos_yaw;

    x_ += x_dot * dt;
    y_ += y_dot * dt;

    depth_ += w_ * dt;
    depth_ = std::max(0.0, depth_);

    publishNavStatus();
  }

  void publishNavStatus()
  {
    marta_msgs::NavStatus nav_msg;
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.frame_id = "map";

    // Convert NED position to lat/lon/depth using RAASL::Geo
    Eigen::Vector3d ned_pos(x_, y_, depth_);
    Eigen::Vector3d lat_lon_depth = RAASL::Geo::NED2LatLonDepth(ned_origin_, ned_pos);

    nav_msg.position.latitude = lat_lon_depth.x();
    nav_msg.position.longitude = lat_lon_depth.y();
    nav_msg.position.depth = depth_;

    nav_msg.orientation.roll = roll_;
    nav_msg.orientation.pitch = pitch_;
    nav_msg.orientation.yaw = yaw_;

    tf::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    nav_msg.quaternion.x = q.x();
    nav_msg.quaternion.y = q.y();
    nav_msg.quaternion.z = q.z();
    nav_msg.quaternion.w = q.w();

    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    nav_msg.ned_speed.x = u_ * cos_yaw - v_ * sin_yaw;
    nav_msg.ned_speed.y = u_ * sin_yaw + v_ * cos_yaw;
    nav_msg.ned_speed.z = w_;

    nav_msg.omega_body.x = p_;
    nav_msg.omega_body.y = q_;
    nav_msg.omega_body.z = r_;

    nav_msg.gps_status = 1;
    nav_msg.initialized = true;

    nav_status_pub_.publish(nav_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher nav_status_pub_;
  ros::Subscriber joystick_sub_;
  ros::Timer timer_;

  zeno_model::VehicleParams vehicle_params_;
  zeno_model::ControlParams control_params_;
  zeno_model::InitialState initial_state_;
  zeno_model::SimulationParams sim_params_;

  // NED origin stored as Eigen vector
  Eigen::Vector3d ned_origin_;
  double ned_origin_lat_, ned_origin_lon_, ned_origin_depth_;

  // State variables
  double x_, y_, depth_;
  double roll_, pitch_, yaw_;
  double u_, v_, w_;
  double p_, q_, r_;
  double u_des_, v_des_;
  double yaw_des_, depth_des_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zeno_joystick");
  try
  {
    ZenoJoystickModel model;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in zeno_joystick: %s", e.what());
    return 1;
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // ZENO JOYSTICK

// #include <mm_bt/zeno_joystick.h>
// #include <tf/tf.h>

// ZenoJoystick::ZenoJoystick(float rate, bool verbose_)
//     : RAASL::NodeHandler::NodeTopicClass(rate, verbose_)
// {
//     // Constructor implementation
//     if (verbose_) ROS_INFO("[%s]: Initializing ZenoJoystick", RAASL::ROS::getNodeName().c_str());
    
//     verbose = verbose_;
//     if (verbose) ROS_INFO("[%s]: Verbose mode is ON", node_name.c_str());

//     // Initialize vehicle state vectors
//     ned_speed = Eigen::Vector3d::Zero();
//     omega_body = Eigen::Vector3d::Zero();

//     // Load parameters from YAML file
//     loadParameters();
    
//     // Initialize vehicle state with starting position
//     initializeVehicleState();
// }

// /**
//  * @brief Load parameters from the parameter server
//  */
// void ZenoJoystick::loadParameters() {
//     // Load initial vehicle state
//     nh_.param<double>("start_latitude", start_latitude, 44.09588219);
//     nh_.param<double>("start_longitude", start_longitude, 9.86493751);
//     nh_.param<double>("start_depth", start_depth, 0.0);
//     nh_.param<double>("start_yaw", start_yaw, 0.0);

//     // Load control parameters (based on multiplier = 4 from joystick_params.yaml)
//     nh_.param<double>("surge_speed_multiplier", surge_speed_multiplier, 0.75);  // [m/s]
//     nh_.param<double>("sway_speed_multiplier", sway_speed_multiplier, 0.75);    // [m/s]
//     nh_.param<double>("yaw_rate", yaw_rate, 30.0 * M_PI / 180.0);              // [rad/s] (30 deg/s)
//     nh_.param<double>("depth_rate", depth_rate, 0.375);                         // [m/s]
//     nh_.param<double>("pitch_rate", pitch_rate, 15.0 * M_PI / 180.0);          // [rad/s] (15 deg/s)

//     if (verbose) {
//         ROS_INFO("[%s]: Loaded parameters:", node_name.c_str());
//         ROS_INFO("[%s]:   Start Latitude: %.8f deg", node_name.c_str(), start_latitude);
//         ROS_INFO("[%s]:   Start Longitude: %.8f deg", node_name.c_str(), start_longitude);
//         ROS_INFO("[%s]:   Start Depth: %.2f m", node_name.c_str(), start_depth);
//         ROS_INFO("[%s]:   Start Yaw: %.2f rad", node_name.c_str(), start_yaw);
//     }
// }

// /**
//  * @brief Initialize vehicle state with starting values
//  */
// void ZenoJoystick::initializeVehicleState() {
//     current_latitude = start_latitude;
//     current_longitude = start_longitude;
//     current_depth = start_depth;
//     current_roll = 0.0;
//     current_pitch = 0.0;
//     current_yaw = start_yaw;

//     ned_speed.setZero();
//     omega_body.setZero();

//     last_update_time = ros::Time::now();
//     initialized = true;

//     if (verbose) {
//         ROS_INFO("[%s]: Vehicle state initialized", node_name.c_str());
//     }
// }

// /**
//  * @brief on_activate implementation
//  */
// bool ZenoJoystick::on_activate() {
//     if (!subscribed) {
//         // Subscribe to joystick commands
//         sub_joystick = nh_.subscribe("/drivers/joystick", 1, &ZenoJoystick::joystickCallback, this);
        
//         // Advertise navigation status
//         pub_nav_status = nh_.advertise<marta_msgs::NavStatus>("/nav_status", 10);
        
//         if (verbose) ROS_INFO("[%s]: Subscribed to /drivers/joystick", node_name.c_str());
//         if (verbose) ROS_INFO("[%s]: Publishing to /nav_status", node_name.c_str());
        
//         subscribed = true;
//     }

//     // Node is ready to activate
//     if (verbose) ROS_INFO("[%s]: Node activated", node_name.c_str());
//     return true;
// }

// /**
//  * @brief on_active implementation - main loop
//  */
// void ZenoJoystick::on_active() {
//     // Calculate time since last update
//     ros::Time current_time = ros::Time::now();
//     double dt = (current_time - last_update_time).toSec();
//     last_update_time = current_time;

//     // Update vehicle state based on joystick commands
//     updateVehicleState(dt);

//     // Publish navigation status
//     publishNavStatus();

//     if (verbose) {
//         ROS_INFO_THROTTLE(2.0, "[%s]: Position: Lat=%.8f, Lon=%.8f, Depth=%.2f, Yaw=%.2f deg", 
//                           node_name.c_str(), current_latitude, current_longitude, 
//                           current_depth, current_yaw * 180.0 / M_PI);
//     }
// }

// /**
//  * @brief on_deactivate implementation
//  */
// void ZenoJoystick::on_deactivate() {
//     if (verbose) ROS_INFO("[%s]: Deactivation started", node_name.c_str());
    
//     // Shutdown subscribers and publishers
//     sub_joystick.shutdown();
//     pub_nav_status.shutdown();
    
//     // Reset flags
//     subscribed = false;
//     joystick_received = false;
    
//     if (verbose) ROS_INFO("[%s]: Deactivation completed", node_name.c_str());
// }

// /**
//  * @brief Joystick callback
//  */
// void ZenoJoystick::joystickCallback(const sensor_msgs::JoyConstPtr& msg) {
//     last_joy_msg = *msg;
//     joystick_received = true;
    
//     if (verbose) {
//         ROS_INFO_THROTTLE(5.0, "[%s]: Joystick command received", node_name.c_str());
//     }
// }

// /**
//  * @brief Update vehicle state based on joystick commands
//  */
// void ZenoJoystick::updateVehicleState(double dt) {
//     if (dt <= 0.0 || dt > 1.0) {
//         return;
//     }

//     // Extract joystick commands
//     // AXES: [SWAY, SURGE, PITCH, ROLL, -, -]
//     // BUTTONS: [MULTIPLIER MAX, MULTIPLIER MEDIUM +, MULTIPLIER MEDIUM -, 
//     //           MULTIPLIER MIN, YAW_SX, YAW_DX, DEPTH-, DEPTH+, ...]
    
//     double sway_cmd = 0.0, surge_cmd = 0.0, pitch_cmd = 0.0;
//     int yaw_left = 0, yaw_right = 0, depth_down = 0, depth_up = 0;

//     if (joystick_received && last_joy_msg.axes.size() >= 4 && last_joy_msg.buttons.size() >= 12) {
//         // Gli assi sono già normalizzati [-1, 1] dal joystick_command
//         sway_cmd = last_joy_msg.axes[0];   // Sway axis (negativo nel joystick_command)
//         surge_cmd = last_joy_msg.axes[1];  // Surge axis
//         pitch_cmd = last_joy_msg.axes[2];  // Pitch axis
        
//         yaw_left = last_joy_msg.buttons[4];   // YAW_SX
//         yaw_right = last_joy_msg.buttons[5];  // YAW_DX
//         depth_down = last_joy_msg.buttons[6]; // DEPTH-
//         depth_up = last_joy_msg.buttons[7];   // DEPTH+
//     }

//     // Calcola velocità nel body frame
//     // Gli assi sono normalizzati, quindi moltiplica per le velocità massime
//     double surge_vel = surge_cmd * surge_speed_multiplier;  // m/s in body x
//     double sway_vel = sway_cmd * sway_speed_multiplier;     // m/s in body y
//     double heave_vel = (depth_up - depth_down) * depth_rate; // m/s in body z

//     // Update angular velocities in body frame
//     omega_body(0) = 0.0;  // Roll rate (not controlled)
//     omega_body(1) = pitch_cmd * pitch_rate;  // Pitch rate
//     omega_body(2) = (yaw_right - yaw_left) * yaw_rate;  // Yaw rate

//     // Update orientation (integrate angular velocities)
//     current_roll += omega_body(0) * dt;
//     current_pitch += omega_body(1) * dt;
//     current_yaw += omega_body(2) * dt;

//     // Wrap angles
//     current_yaw = RAASL::Angles::wrapToPi(current_yaw);
//     current_pitch = RAASL::Angles::wrapToPi(current_pitch);
//     current_roll = RAASL::Angles::wrapToPi(current_roll);

//     // Convert body velocities to NED frame using rotation matrix
//     // La matrice di rotazione da Body a NED usa solo yaw (assumendo roll e pitch piccoli)
//     double cos_yaw = std::cos(current_yaw);
//     double sin_yaw = std::sin(current_yaw);

//     // Velocità NED = R_yaw * velocità_body
//     ned_speed(0) = surge_vel * cos_yaw - sway_vel * sin_yaw;  // North
//     ned_speed(1) = surge_vel * sin_yaw + sway_vel * cos_yaw;  // East
//     ned_speed(2) = heave_vel;  // Down (positivo verso il basso)

//     // Converti posizione attuale in coordinate NED rispetto all'origine
//     Eigen::Vector3d origin_lla(start_latitude, start_longitude, 0.0);
//     Eigen::Vector3d current_lla(current_latitude, current_longitude, current_depth);
    
//     // Calcola NED displacement usando le funzioni geodetiche
//     Eigen::Vector3d current_ned = RAASL::Geo::LatLonDepth2NED(origin_lla, current_lla);
    
//     // Integra la velocità NED per ottenere il nuovo displacement
//     Eigen::Vector3d new_ned = current_ned + ned_speed * dt;
    
//     // Converti il nuovo NED displacement in coordinate geografiche
//     Eigen::Vector3d new_lla = RAASL::Geo::NED2LatLonDepth(origin_lla, new_ned);
    
//     current_latitude = new_lla(0);
//     current_longitude = new_lla(1);
//     current_depth = new_lla(2);

//     // Ensure depth is non-negative (surface is at depth=0)
//     if (current_depth < 0.0) {
//         current_depth = 0.0;
//         ned_speed(2) = 0.0;
//         heave_vel = 0.0;
//     }
// }

// /**
//  * @brief Publish navigation status message
//  */
// void ZenoJoystick::publishNavStatus() {
//     marta_msgs::NavStatus nav_msg;

//     // Fill header
//     nav_msg.header.stamp = ros::Time::now();
//     nav_msg.header.frame_id = "world";

//     // Fill position (lat, lon, depth)
//     nav_msg.position.latitude = current_latitude;
//     nav_msg.position.longitude = current_longitude;
//     nav_msg.position.depth = current_depth;

//     // Fill orientation (Euler angles in radians)
//     nav_msg.orientation.roll = current_roll;
//     nav_msg.orientation.pitch = current_pitch;
//     nav_msg.orientation.yaw = current_yaw;

//     // Fill quaternion
//     tf::Quaternion q;
//     q.setRPY(current_roll, current_pitch, current_yaw);
//     nav_msg.quaternion.x = q.x();
//     nav_msg.quaternion.y = q.y();
//     nav_msg.quaternion.z = q.z();
//     nav_msg.quaternion.w = q.w();

//     // Fill NED velocity
//     nav_msg.ned_speed.x = ned_speed(0);
//     nav_msg.ned_speed.y = ned_speed(1);
//     nav_msg.ned_speed.z = ned_speed(2);

//     // Fill body angular velocity
//     nav_msg.omega_body.x = omega_body(0);
//     nav_msg.omega_body.y = omega_body(1);
//     nav_msg.omega_body.z = omega_body(2);

//     // GPS status and initialization
//     nav_msg.gps_status = 3;  // Simulated - always good fix
//     nav_msg.initialized = initialized;

//     // Publish
//     pub_nav_status.publish(nav_msg);
// }

// /**
//  * @brief Main function
//  */
// int main(int argc, char** argv) {
//     // Initialize ROS node
//     ros::init(argc, argv, "zeno_joystick");

//     // Node parameters
//     float rate = 50.0;  // 50 Hz update rate
//     bool verbose = true;

//     // Create node instance
//     ZenoJoystick zeno_joystick(rate, verbose);

//     // Run the node
//     zeno_joystick.run();

//     return 0;
// }



