/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#include <string>
#include "ximea_camera/ros_nodelet.h"

using ximea_camera::RosNodelet;
using ximea_camera::RosDriver;

RosNodelet::RosNodelet() : running_(false) {
    // nothing to do
}

RosNodelet::~RosNodelet() {
    if (running_) {
        NODELET_INFO("ximea_camera: shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("ximea_camera: driver thread stopped");
    }
    if (drv_->hasValidHandle()) {
        drv_->stopAcquisition();
        drv_->closeDevice();
    }
}

void RosNodelet::onInit() {
    std::string camera_name;
    std::string config_filename;

    NODELET_DEBUG("ximea_camera: onInit()");

    ros::NodeHandle node(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    // we either get the config values as parameters from the launch file
    // or we read it from a given yaml
    // first check if a settings yaml is passed:
    if (priv_nh.getParam("settings_yaml", config_filename)) {
        // fine, use that data
        NODELET_INFO("ximea_camera: loading cam settings from given yaml");
        // create driver
        drv_.reset(new RosDriver(node, priv_nh, config_filename));
    } else {
        // ok, no config file given, we will use data passed as parameters
        // by the launch file
        // create driver
        drv_.reset(new RosDriver(node, priv_nh));
    }

    // open device
    drv_->openDevice();

    // start acquisition
    drv_->startAcquisition();

    // spawn device thread
    running_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
            (new boost::thread(boost::bind(&RosNodelet::devicePoll, this)));
}



void RosNodelet::devicePoll() {
    while (running_) {
        drv_->acquireImage();
        drv_->publishImageAndCamInfo();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
PLUGINLIB_EXPORT_CLASS(ximea_camera::RosNodelet, nodelet::Nodelet);
