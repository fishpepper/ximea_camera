/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#include <string>
#include "ximea_camera/ximea_nodelet.h"

using ximea_camera::ximea_nodelet;

ximea_nodelet::ximea_nodelet() : running_(false) {
    // nothing to do
    NODELET_INFO("ximea_nodelet initialized");
}

ximea_nodelet::~ximea_nodelet() {
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

void ximea_nodelet::onInit() {
    std::string camera_name;
    std::string config_filename;

    NODELET_INFO("ximea_nodelet::onInit()\n");

    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    ros::NodeHandle node(getNodeHandle());

    if (!priv_nh.getParam("settings_yaml", config_filename)) {
        NODELET_ERROR("failed to load camera settings. please pass config yaml "
                      "as parameter 'settings_yaml' "
                      "(e.g. using _settings_yaml:=\"ximea_mq022.yaml\")\n");
        exit(EXIT_FAILURE);
    }


    // create driver
    drv_.reset(new ximea_ros_driver(node, config_filename));

    // open device
    drv_->openDevice();

    // start acquisition
    drv_->startAcquisition();

    /*dvr_->

            op  ]->openDevice();
    if (fixed_init_) {
        cams_[i]->setImageDataFormat("XI_MONO8");
        cams_[i]->setROI(200, 200, 900, 600);
        cams_[i]->setExposure(10000);
    }
    // cams_[i]->limitBandwidth((USB3_BANDWIDTH) - USB_BUS_SAFETY_MARGIN);
    cams_[i]->startAcquisition();


    dvr_->setup();
*/
    // spawn device thread
    running_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
            (new boost::thread(boost::bind(&ximea_nodelet::devicePoll, this)));
}



void ximea_nodelet::devicePoll() {
    while (running_) {
        drv_->acquireImage();
        drv_->publishImageAndCamInfo();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
PLUGINLIB_EXPORT_CLASS(ximea_camera::ximea_nodelet, nodelet::Nodelet);
