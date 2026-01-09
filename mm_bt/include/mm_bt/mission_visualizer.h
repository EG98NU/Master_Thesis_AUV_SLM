#ifndef MM_BT_MISSION_VISUALIZER_H
#define MM_BT_MISSION_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <marta_msgs/NavStatus.h>
#include <rami_msgs/Goal.h>
#include <rami_msgs/Area.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <raasl_toolbox/geodetic.h>
#include <mm_bt/rviz_util.h>
#include <set>
#include <map>
#include <string>
#include <tuple>

namespace visualization {

struct BuoyInfo {
    std::string name;
    double lat, lon, depth;
    std::string color;
    std::string label;
};

struct MapCorners {
    Eigen::Vector2d northeast;
    Eigen::Vector2d northwest;
    Eigen::Vector2d southeast;
    Eigen::Vector2d southwest;
};

class MissionVisualizer {
public:
    MissionVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~MissionVisualizer() = default;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Publishers
    ros::Publisher pub_zeno_mesh_;
    ros::Publisher pub_goal_marker_;
    ros::Publisher pub_survey_area_;
    ros::Publisher pub_map_boundaries_;
    ros::Publisher pub_path_;
    ros::Publisher pub_static_buoys_;
    ros::Publisher pub_discovered_buoys_;
    ros::Publisher pub_compass_rose_;
    
    // Subscribers
    ros::Subscriber sub_nav_status_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_survey_area_;
    
    // Timers
    ros::Timer timer_map_boundaries_;
    ros::Timer timer_static_buoys_;
    ros::Timer timer_reload_discovered_;
    ros::Timer timer_compass_rose_;
    
    // Map parameters
    MapCorners map_corners_;
    double lat_min_, lon_min_;
    double lat_max_, lon_max_;
    
    // Robot state
    marta_msgs::NavStatus nav_data_;
    bool nav_data_received_;
    
    // Goal and survey area
    rami_msgs::Goal::ConstPtr goal_data_;
    rami_msgs::Area::ConstPtr survey_area_data_;
    
    // Path tracking
    nav_msgs::Path path_;
    Eigen::Vector2d last_position_;
    bool has_last_position_;
    int max_path_length_;
    double min_distance_threshold_;
    
    // Buoys
    std::map<int, BuoyInfo> all_buoys_;
    std::set<std::string> discovered_buoys_;
    std::string mission_memory_file_;
    
    // Member functions
    void loadParameters();
    void loadAllBuoys();
    void loadDiscoveredBuoysFromFile();
    void reloadDiscoveredBuoys(const ros::TimerEvent& event);
    
    // Callbacks
    void navStatusCallback(const marta_msgs::NavStatus::ConstPtr& msg);
    void goalCallback(const rami_msgs::Goal::ConstPtr& msg);
    void surveyAreaCallback(const rami_msgs::Area::ConstPtr& msg);
    
    // Publishers
    void publishZenoMesh();
    void publishGoalMarker();
    void publishSurveyArea();
    void publishMapBoundaries(const ros::TimerEvent& event);
    void publishPath();
    void publishStaticBuoys(const ros::TimerEvent& event);
    void publishDiscoveredBuoys();
    void publishCompassRose(const ros::TimerEvent& event);
    
    // Utilities
    void updatePath(double x, double y, double depth);
};

} // namespace visualization

#endif // MM_BT_MISSION_VISUALIZER_H


