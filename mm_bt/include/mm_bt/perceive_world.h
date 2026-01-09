#ifndef MM_BT_PERCEIVE_WORLD_H
#define MM_BT_PERCEIVE_WORLD_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <rami_msgs/OPIInfo.h>
#include <rami_msgs/NodeStatus.h>
#include <marta_msgs/NavStatus.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <raasl_toolbox/geodetic.h>
#include <raasl_toolbox/angles.h>

namespace perception {

struct BuoyData {
    std::string name;
    Eigen::Vector3d gps;      // [lat, lon, depth]
    Eigen::Vector3d ned;      // [N, E, D]
    std::string color;
    std::string label;
    int fiducial_id;
};

class PerceiveWorld {
public:
    PerceiveWorld(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~PerceiveWorld() = default;

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Publishers
    ros::Publisher pub_opi_;
    ros::Publisher pub_status_;
    
    // Subscribers
    ros::Subscriber sub_nav_status_;
    ros::Subscriber sub_odom_;
    
    // Timers
    ros::Timer timer_detection_;
    ros::Timer timer_status_;
    
    // Parameters
    Eigen::Vector3d ned_origin_;  // [lat, lon, depth]
    double detection_range_;
    double detection_fov_;
    double detection_timeout_;
    std::string world_file_path_;
    std::string mission_memory_file_;
    bool generate_world_on_startup_;
    
    // Robot state
    Eigen::Vector3d robot_ned_;   // [N, E, D]
    Eigen::Vector3d robot_gps_;   // [lat, lon, depth]
    double robot_yaw_;
    marta_msgs::NavStatus nav_status_;
    bool nav_status_received_;
    
    // Buoy database
    std::map<int, BuoyData> buoys_;  // fiducial_id -> BuoyData
    
    // Detection tracking
    std::map<int, ros::Time> detected_buoys_;  // fiducial_id -> last_detection_time
    std::set<std::string> discovered_buoys_memory_;  // Persistent memory
    
    // Member functions
    void loadParameters();
    void loadBuoyDatabase();
    void generateStageWorld();
    void initializeMissionMemory();
    void loadDiscoveredBuoys();
    void saveDiscoveredBuoy(int buoy_id);
    
    // Callbacks
    void navStatusCallback(const marta_msgs::NavStatus::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void detectionLoop(const ros::TimerEvent& event);
    void publishStatus(const ros::TimerEvent& event);
    
    // Detection logic
    void publishOPIMessage();
};

} // namespace perception

#endif // MM_BT_PERCEIVE_WORLD_H
