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
#ifndef INCLUDE_XIMEA_CAMERA_XIMEA_DRIVER_H_
#define INCLUDE_XIMEA_CAMERA_XIMEA_DRIVER_H_

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <m3api/xiApi.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "ximea_camera/ximea_param.h"

class ximea_driver{
 public:
    // if no serial no is specified select the first cam on the bus
    explicit ximea_driver(int serial_no = 0 , std::string cam_name = "");
    explicit ximea_driver(std::string file_name);

    int readParamsFromFile(std::string file_name);
    void applyParameters();

    // 0 none, 1 soft_trigger, 2 hard_trigger_rising edge (unsupported) FIXME
    void enableTrigger(unsigned char trigger_mode);

    void limitBandwidth(float factor);
    void openDevice();
    void closeDevice();
    void startAcquisition();
    void stopAcquisition();
    void acquireImage();
    void triggerDevice();
    int getSerialNo() const {
        return serial_no_;
    }

    // this is virtual because the ros class needs to do a bit more work to publish the right image
    virtual void setImageDataFormat(std::string s);
    void setROI(int rect_left, int rect_top, int rect_width, int rect_height);
    void setExposure(int time);
    bool hasValidHandle() {
        return xiH_ == NULL ? false : true;
    }

    const XI_IMG& getImage()const {
        return image_;
    }

 protected:
    bool errorHandling(XI_RETURN ret, std::string command, std::string param, float val = 0.0);

    bool setParamInt(const char *param, int var, bool global = false);
    int  getParamInt(const char *param, bool global = false);

    bool  setParamFloat(const char *param, float var, bool global = false);
    float getParamFloat(const char *param, bool global = false);

    std::string getParamString(const char *param, bool global = false);

    std::string cam_name_;
    std::string image_data_format_;  // One of XI_MONO8, XI_RGB24, XI_RGB32, XI_RAW
    std::string yaml_url_;

    HANDLE xiH_;
    XI_IMG image_;

    typedef std::map<std::string, float> float_param_map_t;
    float_param_map_t float_param_map;
    typedef std::map<std::string, int> int_param_map_t;
    int_param_map_t int_param_map;

 private:
    void assignDefaultValues();
    void fetchLimits();

    // variables for ximea api internals
    int serial_no_;
    int cams_on_bus_;
    int bandwidth_safety_margin_;
    int frame_rate_;
    int bandwidth_;
    int exposure_time_;
    bool auto_exposure_;
    bool binning_enabled_;
    float allocated_bandwidth_;
    int downsample_factor_;
    int rect_left_;
    int rect_top_;
    int rect_width_;
    int rect_height_;
    int cam_resolution_h;
    int cam_resolution_w;
    bool acquisition_active_;
    int image_capture_timeout_;  // max amount of time to wait for an image to come in
    unsigned char trigger_mode_;
};

#endif  // INCLUDE_XIMEA_CAMERA_XIMEA_DRIVER_H_
