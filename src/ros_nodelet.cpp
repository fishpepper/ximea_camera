/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#include <string>
#include "ximea_camera/ros_nodelet.h"

using ximea_camera::ros_nodelet;
using ximea_camera::ros_driver;

ros_nodelet::ros_nodelet() : running_(false) {
    // nothing to do
    NODELET_INFO("ros_nodelet initialized");
}

ros_nodelet::~ros_nodelet() {
    if (running_) {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
    }
    if (drv_->hasValidHandle()) {
        drv_->stopAcquisition();
        drv_->closeDevice();
    }
}

void ros_nodelet::onInit() {
    std::string camera_name;
    std::string config_filename;

    NODELET_INFO("ros_nodelet::onInit()\n");

    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    ros::NodeHandle node(getNodeHandle());

    if (!priv_nh.getParam("settings_yaml", config_filename)) {
        NODELET_ERROR("failed to load camera settings. please pass config yaml "
                      "as parameter 'settings_yaml' "
                      "(e.g. using _settings_yaml:=\"ximea_mq022.yaml\")\n");
        exit(EXIT_FAILURE);
    }


    // create driver
    drv_.reset(new ros_driver(node, config_filename));

    // open device
    drv_->openDevice();

    // start acquisition
    drv_->startAcquisition();

    // spawn device thread
    running_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
            (new boost::thread(boost::bind(&ros_nodelet::devicePoll, this)));
}



void ros_nodelet::devicePoll() {
    while (running_) {
        drv_->acquireImage();
        drv_->publishImageAndCamInfo();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
PLUGINLIB_EXPORT_CLASS(ximea_camera::ros_nodelet, nodelet::Nodelet);
