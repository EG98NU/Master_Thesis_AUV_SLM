#include "mm_bt/mission_visualizer.h"
#include <tf/transform_datatypes.h>
#include <cmath>

namespace visualization {

static const double BUOY_SCALE = 0.01;

MissionVisualizer::MissionVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), nav_data_received_(false), has_last_position_(false) {
    
    ROS_INFO("Waiting for map parameters...");
    ros::Duration(2.0).sleep();
    
    loadParameters();
    loadAllBuoys();
    loadDiscoveredBuoysFromFile();
    
    // Publishers
    pub_zeno_mesh_ = nh_.advertise<visualization_msgs::Marker>("/visualization/zeno_mesh", 1);
    pub_goal_marker_ = nh_.advertise<visualization_msgs::Marker>("/visualization/goal_marker", 1);
    pub_survey_area_ = nh_.advertise<visualization_msgs::Marker>("/visualization/survey_area", 1);
    pub_map_boundaries_ = nh_.advertise<visualization_msgs::Marker>("/visualization/map_boundaries", 1);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/visualization/robot_path", 10);
    pub_static_buoys_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/static_buoys", 10);
    pub_discovered_buoys_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/discovered_buoys", 10);
    pub_compass_rose_ = nh_.advertise<visualization_msgs::Marker>("/visualization/compass_rose", 1);
    
    // Subscribers
    sub_nav_status_ = nh_.subscribe("/nav_status", 1, &MissionVisualizer::navStatusCallback, this);
    sub_goal_ = nh_.subscribe("/raasl_stack/guidance/send_goal", 1, &MissionVisualizer::goalCallback, this);
    sub_survey_area_ = nh_.subscribe("/raasl_stack/guidance/survey_area", 1, &MissionVisualizer::surveyAreaCallback, this);
    
    // Timers
    timer_map_boundaries_ = nh_.createTimer(ros::Duration(1.0), &MissionVisualizer::publishMapBoundaries, this);
    timer_static_buoys_ = nh_.createTimer(ros::Duration(0.5), &MissionVisualizer::publishStaticBuoys, this);
    timer_reload_discovered_ = nh_.createTimer(ros::Duration(1.0), &MissionVisualizer::reloadDiscoveredBuoys, this);
    timer_compass_rose_ = nh_.createTimer(ros::Duration(1.0), &MissionVisualizer::publishCompassRose, this);
    
    path_.header.frame_id = "my_frame_id";
    
    ROS_INFO("Mission Visualizer initialized successfully");
    ROS_INFO("Loaded %lu static buoys for visualization", all_buoys_.size());
}

void MissionVisualizer::loadParameters() {
    // Map corners
    nh_.param("/area_corners/northeast/latitude", map_corners_.northeast(0), 44.09611115206862);
    nh_.param("/area_corners/northeast/longitude", map_corners_.northeast(1), 9.865054589742114);
    nh_.param("/area_corners/northwest/latitude", map_corners_.northwest(0), 44.09601351852949);
    nh_.param("/area_corners/northwest/longitude", map_corners_.northwest(1), 9.864608338509868);
    nh_.param("/area_corners/southeast/latitude", map_corners_.southeast(0), 44.095752729346444);
    nh_.param("/area_corners/southeast/longitude", map_corners_.southeast(1), 9.86520762383199);
    nh_.param("/area_corners/southwest/latitude", map_corners_.southwest(0), 44.095651845671284);
    nh_.param("/area_corners/southwest/longitude", map_corners_.southwest(1), 9.864767936334957);
    
    lat_min_ = map_corners_.southwest(0);
    lon_min_ = map_corners_.northwest(1);
    lat_max_ = map_corners_.northeast(0);
    lon_max_ = map_corners_.southeast(1);
    
    nh_private_.param("max_path_length", max_path_length_, 1000);
    nh_private_.param("min_distance_threshold", min_distance_threshold_, 0.05);
    
    std::string default_memory = std::string(getenv("HOME")) + "/btree_ws/src/mm_bt/config/mission_memory.yaml";
    nh_private_.param("mission_memory_file", mission_memory_file_, default_memory);
    
    ROS_INFO("Map boundaries loaded:");
    ROS_INFO("  Southwest: lat=%.8f, lon=%.8f", lat_min_, lon_min_);
    ROS_INFO("  Northeast: lat=%.8f, lon=%.8f", lat_max_, lon_max_);
}

void MissionVisualizer::loadAllBuoys() {
    XmlRpc::XmlRpcValue buoys_config;
    if (!nh_.getParam("/mission_objects/buoys", buoys_config)) {
        ROS_ERROR("Failed to load static buoys");
        return;
    }
    
    for (XmlRpc::XmlRpcValue::iterator it = buoys_config.begin(); it != buoys_config.end(); ++it) {
        std::string buoy_name = it->first;
        XmlRpc::XmlRpcValue& config = it->second;
        
        BuoyInfo buoy;
        buoy.name = buoy_name;
        buoy.lat = static_cast<double>(config["latitude"]);
        buoy.lon = static_cast<double>(config["longitude"]);
        buoy.depth = static_cast<double>(config["depth"]);
        buoy.color = static_cast<std::string>(config["color"]);
        buoy.label = static_cast<std::string>(config["label"]);
        
        int fid = static_cast<int>(config["fiducial_id"]);
        all_buoys_[fid] = buoy;
    }
    
    ROS_INFO("Loaded %lu static buoys from mission_objects.yaml", all_buoys_.size());
}

void MissionVisualizer::loadDiscoveredBuoysFromFile() {
    try {
        YAML::Node memory = YAML::LoadFile(mission_memory_file_);
        if (memory["discovered_buoys"]) {
            for (size_t i = 0; i < memory["discovered_buoys"].size(); ++i) {
                discovered_buoys_.insert(memory["discovered_buoys"][i]["label"].as<std::string>());
            }
            ROS_INFO("Loaded %lu discovered buoys from mission memory", discovered_buoys_.size());
        }
    } catch (const std::exception& e) {
        ROS_INFO("Mission memory file does not exist yet");
    }
}

void MissionVisualizer::reloadDiscoveredBuoys(const ros::TimerEvent& event) {
    try {
        YAML::Node memory = YAML::LoadFile(mission_memory_file_);
        if (memory["discovered_buoys"]) {
            size_t old_count = discovered_buoys_.size();
            discovered_buoys_.clear();
            for (size_t i = 0; i < memory["discovered_buoys"].size(); ++i) {
                discovered_buoys_.insert(memory["discovered_buoys"][i]["label"].as<std::string>());
            }
            size_t new_count = discovered_buoys_.size();
            if (new_count != old_count) {
                ROS_INFO("Updated discovered buoys: %lu -> %lu", old_count, new_count);
            }
        }
    } catch (...) {}
}

void MissionVisualizer::navStatusCallback(const marta_msgs::NavStatus::ConstPtr& msg) {
    nav_data_ = *msg;
    nav_data_received_ = true;
    
    // Convert lat/lon to local coordinates using raasl_toolbox
    Eigen::Vector2d origin(lat_min_, lon_min_);
    Eigen::Vector2d pos(msg->position.latitude, msg->position.longitude);
    Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, pos);
    
    double zeno_x = local(0);
    double zeno_y = local(1);
    
    // Broadcast transforms using rviz_util
    // RvizUtil::transformBroadcast("map", "my_frame_id", 0, 0, 0, 0, M_PI, -M_PI/2);
    RvizUtil::transformBroadcast("map", "my_frame_id", 0, 0, 0, M_PI, 0, -M_PI/2);
    RvizUtil::transformBroadcast("my_frame_id", "body_frame", zeno_x, zeno_y, 0, 0, 0, msg->orientation.yaw);
    
    updatePath(zeno_x, zeno_y, msg->position.depth);
    publishZenoMesh();
    publishPath();
    publishDiscoveredBuoys();
}

void MissionVisualizer::goalCallback(const rami_msgs::Goal::ConstPtr& msg) {
    goal_data_ = msg;
    publishGoalMarker();
    ROS_INFO("New goal received at lat=%.6f, lon=%.6f", msg->latitude, msg->longitude);
}

void MissionVisualizer::surveyAreaCallback(const rami_msgs::Area::ConstPtr& msg) {
    survey_area_data_ = msg;
    publishSurveyArea();
    ROS_INFO("Survey area received at lat=%.6f, lon=%.6f, radius=%.2f m",
             msg->latitude, msg->longitude, msg->radius);
}

void MissionVisualizer::updatePath(double x, double y, double depth) {
    if (has_last_position_) {
        double dx = x - last_position_(0);
        double dy = y - last_position_(1);
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance_threshold_) {
            return;
        }
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "my_frame_id";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = depth;
    pose.pose.orientation.w = 1.0;
    
    path_.poses.push_back(pose);
    
    if (static_cast<int>(path_.poses.size()) > max_path_length_) {
        path_.poses.erase(path_.poses.begin());
    }
    
    last_position_ = Eigen::Vector2d(x, y);
    has_last_position_ = true;
}

void MissionVisualizer::publishPath() {
    if (!path_.poses.empty()) {
        path_.header.stamp = ros::Time::now();
        pub_path_.publish(path_);
    }
}

void MissionVisualizer::publishZenoMesh() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "body_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "zeno_robot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = -1.0;
    marker.pose.position.y = -0.73;
    marker.pose.position.z = -0.8;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = marker.scale.y = marker.scale.z = 0.003;
    marker.color.a = 1.0;
    marker.color.r = marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    marker.mesh_resource = "package://mm_bt/config/rviz/mesh_zeno.stl";
    pub_zeno_mesh_.publish(marker);
}

void MissionVisualizer::publishGoalMarker() {
    if (!goal_data_) return;
    
    Eigen::Vector2d origin(lat_min_, lon_min_);
    Eigen::Vector2d goal_pos(goal_data_->latitude, goal_data_->longitude);
    Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, goal_pos);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "my_frame_id";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = local(0);
    marker.pose.position.y = local(1);
    marker.pose.position.z = goal_data_->depth;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.a = 0.8;
    marker.color.g = 1.0;
    
    pub_goal_marker_.publish(marker);
}

void MissionVisualizer::publishSurveyArea() {
    if (!survey_area_data_) return;
    
    Eigen::Vector2d origin(lat_min_, lon_min_);
    Eigen::Vector2d area_pos(survey_area_data_->latitude, survey_area_data_->longitude);
    Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, area_pos);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "my_frame_id";
    marker.header.stamp = ros::Time::now();
    marker.ns = "survey_area";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = local(0);
    marker.pose.position.y = local(1);
    marker.pose.position.z = survey_area_data_->depth + 0.5;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = marker.scale.y = survey_area_data_->radius * 2.0;
    marker.scale.z = 0.1;
    marker.color.a = 0.3;
    marker.color.g = 1.0;
    
    pub_survey_area_.publish(marker);
}

void MissionVisualizer::publishMapBoundaries(const ros::TimerEvent& event) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "my_frame_id";
    marker.header.stamp = ros::Time::now();
    marker.ns = "map_boundaries";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    Eigen::Vector2d origin(lat_min_, lon_min_);
    std::vector<Eigen::Vector2d> corners;
    corners.push_back(map_corners_.northwest);
    corners.push_back(map_corners_.northeast);
    corners.push_back(map_corners_.southeast);
    corners.push_back(map_corners_.southwest);
    corners.push_back(map_corners_.northwest);
    
    for (size_t i = 0; i < corners.size(); ++i) {
        Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, corners[i]);
        geometry_msgs::Point p;
        p.x = local(0);
        p.y = local(1);
        p.z = 0.0;
        marker.points.push_back(p);
    }
    
    marker.scale.x = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    
    pub_map_boundaries_.publish(marker);
}

void MissionVisualizer::publishStaticBuoys(const ros::TimerEvent& event) {
    visualization_msgs::MarkerArray marker_array;
    
    std::map<std::string, std::tuple<double, double, double> > color_map;
    color_map["yellow"] = std::make_tuple(1.0, 1.0, 0.0);
    color_map["white"] = std::make_tuple(1.0, 1.0, 1.0);
    color_map["black"] = std::make_tuple(0.1, 0.1, 0.1);
    color_map["red"] = std::make_tuple(1.0, 0.0, 0.0);
    
    Eigen::Vector2d origin(lat_min_, lon_min_);
    
    for (std::map<int, BuoyInfo>::const_iterator it = all_buoys_.begin(); it != all_buoys_.end(); ++it) {
        int fid = it->first;
        const BuoyInfo& buoy = it->second;
        
        // Skip discovered buoys
        if (discovered_buoys_.count(buoy.label)) continue;
        
        Eigen::Vector2d buoy_pos(buoy.lat, buoy.lon);
        Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, buoy_pos);
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "my_frame_id";
        marker.header.stamp = ros::Time::now();
        marker.ns = "static_buoys";
        marker.id = fid;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = local(0);
        marker.pose.position.y = local(1);
        marker.pose.position.z = buoy.depth;

        tf2::Quaternion q;
        q.setRPY(0, M_PI, 0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        marker.scale.x = marker.scale.y = marker.scale.z = BUOY_SCALE;
        
        double r = 0.5, g = 0.5, b = 0.5;
        if (color_map.find(buoy.color) != color_map.end()) {
            std::tuple<double, double, double> rgb = color_map[buoy.color];
            r = std::get<0>(rgb);
            g = std::get<1>(rgb);
            b = std::get<2>(rgb);
        }
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.3;  // Semi-transparent
        
        marker.mesh_resource = "package://mm_bt/config/rviz/buoy.stl";
        marker_array.markers.push_back(marker);
    }
    
    pub_static_buoys_.publish(marker_array);
}

void MissionVisualizer::publishDiscoveredBuoys() {
    visualization_msgs::MarkerArray marker_array;
    
    std::map<std::string, std::tuple<double, double, double> > color_map;
    color_map["yellow"] = std::make_tuple(1.0, 1.0, 0.0);
    color_map["white"] = std::make_tuple(1.0, 1.0, 1.0);
    color_map["black"] = std::make_tuple(0.1, 0.1, 0.1);
    color_map["red"] = std::make_tuple(1.0, 0.0, 0.0);
    
    Eigen::Vector2d origin(lat_min_, lon_min_);
    int marker_id = 0;
    
    for (std::set<std::string>::const_iterator label_it = discovered_buoys_.begin(); 
         label_it != discovered_buoys_.end(); ++label_it) {
        const std::string& label = *label_it;
        
        const BuoyInfo* buoy_data = NULL;
        for (std::map<int, BuoyInfo>::const_iterator it = all_buoys_.begin(); it != all_buoys_.end(); ++it) {
            if (it->second.label == label) {
                buoy_data = &(it->second);
                break;
            }
        }
        
        if (!buoy_data) continue;
        
        Eigen::Vector2d buoy_pos(buoy_data->lat, buoy_data->lon);
        Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, buoy_pos);
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "my_frame_id";
        marker.header.stamp = ros::Time::now();
        marker.ns = "discovered_buoys";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = local(0);
        marker.pose.position.y = local(1);
        marker.pose.position.z = buoy_data->depth;

        tf2::Quaternion q;
        q.setRPY(0, M_PI, 0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        marker.scale.x = marker.scale.y = marker.scale.z = BUOY_SCALE;
        
        double r = 0.5, g = 0.5, b = 0.5;
        if (color_map.find(buoy_data->color) != color_map.end()) {
            std::tuple<double, double, double> rgb = color_map[buoy_data->color];
            r = std::get<0>(rgb);
            g = std::get<1>(rgb);
            b = std::get<2>(rgb);
        }
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;  // Fully opaque
        
        marker.mesh_resource = "package://mm_bt/config/rviz/buoy.stl";
        marker_array.markers.push_back(marker);
    }
    
    pub_discovered_buoys_.publish(marker_array);
}

void MissionVisualizer::publishCompassRose(const ros::TimerEvent& event) {
    // Calcola il centro della RAMI area 1
    double center_lat = (map_corners_.northeast(0) + map_corners_.northwest(0) + 
                         map_corners_.southeast(0) + map_corners_.southwest(0)) / 4.0;
    double center_lon = (map_corners_.northeast(1) + map_corners_.northwest(1) + 
                         map_corners_.southeast(1) + map_corners_.southwest(1)) / 4.0;
    
    // Converti in coordinate locali
    Eigen::Vector2d origin(lat_min_, lon_min_);
    Eigen::Vector2d center_pos(center_lat, center_lon);
    Eigen::Vector2d local = RAASL::Geo::LatLon2NE(origin, center_pos);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "my_frame_id";
    marker.header.stamp = ros::Time::now();
    marker.ns = "compass_rose";
    marker.id = 100;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Posizione al centro della mappa, 1 metro sotto il piano (z = -1.0)
    marker.pose.position.x = local(0);
    marker.pose.position.y = local(1);
    marker.pose.position.z = 1.0;
    
    // Orientamento: ruota di 90 gradi in senso orario attorno all'asse Z
    // per allineare il nord del compass rose con il nord reale
    // Il sistema "my_frame_id" ha una rotazione di M_PI attorno a Y e -M_PI/2 attorno a Z
    // Dobbiamo compensare con una rotazione che punti effettivamente a nord
    tf2::Quaternion q;
    // q.setRPY(0, 0, -M_PI/2);  // Rotazione di -90° attorno a Z per allineare a nord
    q.setRPY(0, M_PI, -M_PI/2);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // Scala: 5 metri di raggio significa 10 metri di diametro
    // Adatta la scala in base alle dimensioni originali dell'STL
    marker.scale.x = marker.scale.y = marker.scale.z = 4e-2;
    
    // Colore bianco trasparente
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.3;  // Trasparente (30% opacità)
    
    // Path del file STL
    marker.mesh_resource = "package://mm_bt/config/rviz/compass_rose.stl";
    
    pub_compass_rose_.publish(marker);
}
  

} // namespace visualization

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    visualization::MissionVisualizer node(nh, nh_private);
    ros::spin();
    
    return 0;
}
