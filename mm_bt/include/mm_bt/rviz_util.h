#ifndef MM_BT_RVIZ_UTIL_H
#define MM_BT_RVIZ_UTIL_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

namespace RvizUtil {

/**
 * @brief Creates and broadcasts a roto-translational transform between frames
 * @param frame_1 Starting frame name
 * @param frame_2 Arrival frame name
 * @param dx, dy, dz Translational part (meters)
 * @param roll, pitch, yaw Rotational part (radians)
 */
inline void transformBroadcast(const std::string& frame_1, const std::string& frame_2,
                               double dx, double dy, double dz,
                               double roll, double pitch, double yaw) {
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = frame_1;
    transform.child_frame_id = frame_2;
    
    transform.transform.translation.x = dx;
    transform.transform.translation.y = dy;
    transform.transform.translation.z = dz;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    broadcaster.sendTransform(transform);
}

/**
 * @brief Creates and returns a PoseStamped message
 * @param x, y, z Position coordinates
 * @param roll, pitch, yaw Orientation angles (radians)
 * @param frame_id Reference frame name
 * @return PoseStamped message
 */
inline geometry_msgs::PoseStamped poseCreate(double x, double y, double z,
                                              double roll, double pitch, double yaw,
                                              const std::string& frame_id = "my_frame_id") {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now();
    
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;
}

/**
 * @brief Creates and returns a PointStamped message
 * @param x, y, z Position coordinates
 * @param frame_id Reference frame name
 * @return PointStamped message
 */
inline geometry_msgs::PointStamped pointCreate(double x, double y, double z,
                                                const std::string& frame_id = "my_frame_id") {
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.header.stamp = ros::Time::now();
    
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    
    return point;
}

/**
 * @brief Creates an OccupancyGrid message for visualization
 * @param n_rows Number of rows
 * @param n_cols Number of columns
 * @param res_meters Grid cell dimension (meters)
 * @param grid Grid data (Eigen matrix, values 0 or 1)
 * @param frame_id Reference frame name
 * @return OccupancyGrid message
 */
inline nav_msgs::OccupancyGrid createOccupancyGrid(int n_rows, int n_cols, double res_meters,
                                                    const Eigen::MatrixXi& grid,
                                                    const std::string& frame_id = "my_frame_id") {
    nav_msgs::OccupancyGrid v_map;
    v_map.header.stamp = ros::Time::now();
    v_map.header.frame_id = frame_id;
    v_map.info.map_load_time = ros::Time::now();
    v_map.info.resolution = res_meters;
    v_map.info.width = n_rows;
    v_map.info.height = n_cols;
    
    v_map.info.origin.position.x = 0.0;
    v_map.info.origin.position.y = 0.0;
    v_map.info.origin.position.z = 0.0;
    v_map.info.origin.orientation.w = 1.0;
    
    // Convert grid to occupancy data (0-100 scale)
    v_map.data.resize(n_rows * n_cols);
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            v_map.data[j * n_rows + i] = grid(i, j) * 100;
        }
    }
    
    return v_map;
}

} // namespace RvizUtil

#endif // MM_BT_RVIZ_UTIL_H
