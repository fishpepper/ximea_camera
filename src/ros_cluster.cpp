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

#include <ximea_camera/ros_cluster.h>
#include <string>
#include <vector>

using ximea_camera::ros_cluster;
using ximea_camera::ros_driver;
using ximea_camera::driver;


std::string getCamNameFromYaml(std::string file_name) {
    std::ifstream fin(file_name.c_str());
    if (fin.fail()) {
        ROS_ERROR_STREAM("could not open file '" << file_name.c_str() << "'" << std::endl);
        exit(-1);  // this has to be changed
    }

    YAML::Node doc = YAML::LoadFile(file_name);
    std::string ret;
    ret = doc["cam_name"].as<std::string>();
    return ret;
}

ros_cluster::ros_cluster(std::vector<std::string> filenames) {
    devices_open_ = false;
    for (int i = 0 ; i < filenames.size(); i ++) {
        std::string cam_name = getCamNameFromYaml(filenames[i]);
        ros::NodeHandle nh(std::string("/") + cam_name);

        boost::shared_ptr<ros_driver> ros_driver_ptr(
                    new ros_driver(nh, filenames[i]));

        add_camera(ros_driver_ptr);
    }
}

void ros_cluster::add_camera(boost::shared_ptr<ros_driver> xd) {
    if (devices_open_) {
        clusterEnd();
    }

    cams_.push_back(xd);
    num_cams_++;
    threads_.resize(num_cams_);
    ROS_INFO_STREAM("done camera add");
}

void ros_cluster::remove_camera(int serial_no) {
    if (devices_open_) {
        clusterEnd();
    }

    for (int i = 0; i < cams_.size(); i++) {
        if (serial_no == cams_[i]->getSerialNo()) {
            cams_.erase(cams_.begin() + i);
            delete threads_[i];
            threads_.erase(threads_.begin() + i);
            break;
        }
    }
    num_cams_--;
}

void ros_cluster::clusterInit() {
    for (int i = 0; i < cams_.size(); i++) {
        ROS_INFO_STREAM("opening device " << cams_[i]->getSerialNo());
        cams_[i]->openDevice();
        cams_[i]->startAcquisition();
        // FIXME: remove this into constructor
    }
    devices_open_ = true;
}

void ros_cluster::clusterEnd() {
    for (int i = 0; i < cams_.size(); i  ++) {
        cams_[i]->stopAcquisition();
        cams_[i]->closeDevice();
    }
    devices_open_ = false;
}

// triggered_acquire
void ros_cluster::clusterAcquire() {
    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i] = new boost::thread(&driver::acquireImage, cams_[i]);
    }

    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i]->join();
        delete threads_[i];
    }
}

void ros_cluster::clusterPublishImages() {
    // FIXME: might want to think as to how to multithread this
    // FIXME: use cam timestamp if requested
    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i] = new boost::thread(&ros_driver::publishImage, cams_[i], ros::Time::now());
    }

    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i]->join();
        delete threads_[i];
    }
}


void ros_cluster::clusterPublishCamInfo() {
    for (int i = 0 ; i < cams_.size(); i ++) {
        cams_[i]->publishCamInfo(ros::Time::now());
    }
}

void ros_cluster::clusterPublishImageAndCamInfo() {
    ros::Time curr_time = ros::Time::now();

    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i] = new boost::thread(&ros_driver::publishImage, cams_[i], curr_time);
    }

    for (int i = 0; i < cams_.size(); i  ++) {
        threads_[i]->join();
        delete threads_[i];
    }

    for (int i = 0 ; i < cams_.size(); i ++) {
        cams_[i]->publishCamInfo(curr_time);
    }
}

int ros_cluster::getCameraIndex(int serial_no) {
    for (int i = 0; i < cams_.size(); i++) {
        if (serial_no == cams_[i]->getSerialNo()) {
            return i;
        }
    }
    return -1;
}

void ros_cluster::setImageDataFormat(int serial_no, std::string s) {
    int idx;
    if (idx = getCameraIndex(serial_no) != -1) {
        cams_[idx]->setImageDataFormat(s);
    }
}

