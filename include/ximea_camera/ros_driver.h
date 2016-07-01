/******************************************************************************

Copyright 2016  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]
                Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#ifndef INCLUDE_XIMEA_CAMERA_ROS_DRIVER_H_
#define INCLUDE_XIMEA_CAMERA_ROS_DRIVER_H_

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ximea_camera/driver.h>

#include <string>

#include "ximea_camera/xiAPIConfig.h"

namespace ximea_camera {

class RosDriver : public Driver {
 public:
    RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh, std::string cam_name, int serial_no,
                     std::string yaml_url);
    RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh, std::string file_name);
    explicit RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    virtual void setImageDataFormat(std::string s);

    // since these 2 functions should have the same time stamp we leave it up to the user to
    // specify the time if it is needed to do one or the other
    void publishImage(const ros::Time & now);
    void publishCamInfo(const ros::Time &now);
    void publishImageAndCamInfo();

 private:
    void applyParameters();
    void commonInitialize(const ros::NodeHandle &nh);
    void attachToDynamicReconfigureServer();
    void dynamicReconfigureCallback(const ximea_camera::xiAPIConfig &config, uint32_t level);
    bool dynamicReconfigureFloat(const char *param, float value);
    bool dynamicReconfigureInt(const char *param, int value);

    ros::Time getTimestamp();

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
    dynamic_reconfigure::Server<ximea_camera::xiAPIConfig> *reconf_server_;
};

}  // namespace ximea_camera

#endif  // INCLUDE_XIMEA_CAMERA_ROS_DRIVER_H_
