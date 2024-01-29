/** @file 
    @brief Rao-Blackwellized Gmapping, with the counting mapping model
    @author Rico Ruotong Jia
    @date 2024-01-28
    SUBSCRIBES:
    - @subscribes: /scan: laser scan data  
    - @subscribes: /tf: listens for for odom -> baselink
    PUBLISHES:
    - @publishes: /map: map of the current environment
    - @publishes: /tf: map -> odom transform

    PARAMETERS:
    - @param: base_frame
    - @param: map_frame
    - @param: odom_frame
    - @param: map_update_interval
    laser params
    - @param: maxRange: maximum laser scanner range
    - @param: sigma: standard deviation for the scan matching process 
    - @param: search window for scan matching
    // TODO: scan matching kernel, and steps?
    // TODO: benefits of 
        scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
        scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
        scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));
*/


// Need message filter. Also, a 1s "expiration time"?
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include "dream_gmapping/dream_gmapping.hpp"
#include <iostream>
#include <memory>
#include <ros/node_handle.h>
#include <ros/ros.h>
namespace DreamGMapping
{
    DreamGMapper::DreamGMapper(ros::NodeHandle& nh_)
    {
        // Read parameters without default values here, so they must be specified in launch file
        nh_.getParam("base_frame", base_frame_);
        nh_.getParam("map_frame", map_frame_);
        nh_.getParam("odom_frame", odom_frame_);
        nh_.getParam("map_update_interval", map_update_interval_);
        ROS_INFO_STREAM("Successfully read parameters for dream_gmapping");

        scan_sub_ptr_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, "scan", 1);
        // wait on the odom frame
        scan_filter_ptr_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*scan_sub_ptr_, tf_buffer_, odom_frame_, 1, nh_);
        scan_filter_ptr_ -> registerCallback(boost::bind(&DreamGMapper::laser_scan, this, _1));
    }

    DreamGMapper::~DreamGMapper() = default;

    void DreamGMapper::laser_scan(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan_msg){
        // TODO: test the filter, with a laserscan msg first, then with odom
        if (!received_first_laser_scan_){
            // Store the laser->base transform
            received_first_laser_scan_ = true;
            ROS_DEBUG_STREAM("Received first laser scan");
        }
    }
}
int main(int argc, char**argv){
    ros::init(argc, argv, "dream_gmapping");
    ros::NodeHandle nh_("~");
    DreamGMapping::DreamGMapper dg(nh_);

    // init map
    // get laser scanner -> base link tf from the scan topic

    // For simplicity, we are using a single threaded model for subscribers
    ros::spin();
    return 0;
}