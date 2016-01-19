/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#ifndef INCLUDE_XIMEA_CAMERA_XIMEA_ROS_DRIVER_H_
#define INCLUDE_XIMEA_CAMERA_XIMEA_ROS_DRIVER_H_

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ximea_camera/ximea_driver.h>

#include <string>

#include "ximea_camera/xiAPIConfig.h"

class ximea_ros_driver : public ximea_driver {
 public:
    ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no,
                     std::string yaml_url);
    ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name);
    virtual void setImageDataFormat(std::string s);

    // since these 2 functions should have the same time stamp we leave it up to the user to
    // specify the time if it is needed to do one or the other
    void publishImage(const ros::Time & now);
    void publishCamInfo(const ros::Time &now);
    void publishImageAndCamInfo();

 private:
    void common_initialize(const ros::NodeHandle &nh);
    void dynamic_reconfigure_callback(ximea_camera::xiAPIConfig &config, uint32_t level);
    bool dynamic_reconfigure_float(const char *param, float value);

    ros::NodeHandle pnh_;
    camera_info_manager::CameraInfoManager *cam_info_manager_;
    image_transport::ImageTransport *it_;
    image_transport::Publisher ros_cam_pub_;
    ros::Publisher cam_info_pub_;

    sensor_msgs::Image ros_image_;
    sensor_msgs::CameraInfo cam_info_;
    char * cam_buffer_;
    int cam_buffer_size_;
    int bpp_;  // the next 2 paramaeters are used by the ros_image_transport publisher
    std::string encoding_;
    dynamic_reconfigure::Server<ximea_camera::xiAPIConfig> *server;
};

#endif  // INCLUDE_XIMEA_CAMERA_XIMEA_ROS_DRIVER_H_
