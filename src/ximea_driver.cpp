/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#include <ximea_camera/ximea_driver.h>
#include <sstream>
#include <stdexcept>
#include <string>

#define XIMEA_DRIVER_DEBUG_LEVEL XI_DL_FATAL    // XI_DL_DETAIL

ximea_driver::ximea_driver(int serial_no, std::string cam_name) {
    serial_no_ = serial_no;
    cam_name_ = cam_name;
    cam_resolution_w = 0;
    cam_resolution_h = 0;
    assignDefaultValues();
}

void ximea_driver::assignDefaultValues() {
    allocated_bandwidth_ = 1.0;
    cams_on_bus_ = 4;
    bandwidth_safety_margin_ = 30;
    binning_enabled_ = false;
    downsample_factor_ = false;
    auto_exposure_ = false;
    exposure_time_ = 1000;
    image_data_format_ = "XI_MONO8";
    rect_left_ = 0;
    rect_top_ = 0;
    rect_width_ = cam_resolution_w;
    rect_height_ = cam_resolution_h;
    xiH_ = NULL;
    // pre init all XI_IMG fields to zero:
    memset(&image_, 0, sizeof(XI_IMG));
    image_.size = sizeof(XI_IMG);
    image_.bp = NULL;
    image_.bp_size = 0;
    acquisition_active_ = false;
    image_capture_timeout_ = 1000;
}

ximea_driver::ximea_driver(std::string file_name) {
    assignDefaultValues();
    readParamsFromFile(file_name);
    ROS_INFO_STREAM("ximea_driver: reading paramter values from file: " << file_name);
}

inline bool ximea_driver::errorHandling(XI_RETURN ret, std::string command,
                                        std::string param, float val) {
    if (ret != XI_OK) {
        std::cout << "ximea_driver: " << command << "(" << param << ", "<< val
                  << " ) failed (errno " << ret << "\n";
        closeDevice();
        return false;
    } else {
        return true;
    }
}

void ximea_driver::fetchLimits() {
    // fetch frame size from cam:
    cam_resolution_w = getParamInt(XI_PRM_WIDTH XI_PRM_INFO_MAX);
    cam_resolution_h = getParamInt(XI_PRM_HEIGHT XI_PRM_INFO_MAX);
}

void ximea_driver::applyParameters() {
    setImageDataFormat(image_data_format_);
    setExposure(exposure_time_);
    setROI(rect_left_, rect_top_, rect_width_, rect_height_);
}

void ximea_driver::openDevice() {
    XI_RETURN stat;

    // set xiapi debug level:
    setParamInt(XI_PRM_DEBUG_LEVEL, XIMEA_DRIVER_DEBUG_LEVEL, true);

    // disable auto bandwidth measurements:
    setParamInt(XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF, true);

    if (serial_no_ == 0) {
        stat = xiOpenDevice(0, &xiH_);
        errorHandling(stat, "xiOpenDevice", "");
    } else {
        std::stringstream conv;
        conv << serial_no_;
        stat = xiOpenDeviceBy(XI_OPEN_BY_SN, conv.str().c_str(), &xiH_);
        errorHandling(stat, "xiOpenDevice",  conv.str());
    }

    // fetch cam name
    std::string camera_name = getParamString(XI_PRM_DEVICE_NAME);

    // fetch cam serial (useful if no serial was given)
    std::string camera_serial = getParamString(XI_PRM_DEVICE_SN);

    // some info
    std::cout << "Opened Camera with Serial " << camera_serial << " and name '"
              << camera_name << "' \n";

    // limit bandwidth
    // in case multiple cameras share the same bus we
    // have to manually split the overall bandwidth between devices
    // see allocated_bandwidth parameter in config file
    limitBandwidth(allocated_bandwidth_);

    fetchLimits();

    applyParameters();
}

void ximea_driver::closeDevice() {
    if (xiH_) {
        xiCloseDevice(xiH_);
        xiH_ = NULL;
    }
}

void ximea_driver::stopAcquisition() {
    XI_RETURN stat;
    if (!hasValidHandle()) {
        return;
    }

    stat = xiStopAcquisition(xiH_);
    errorHandling(stat, "xiStopAcquisition", "");

    acquisition_active_ = false;
}

void ximea_driver::startAcquisition() {
    if (!hasValidHandle()) {
        return;
    }
    XI_RETURN stat = xiStartAcquisition(xiH_);
    errorHandling(stat, "xiStartAcquisition", "");

    acquisition_active_ = true;
}

void ximea_driver::acquireImage() {
    XI_RETURN stat;

    if (!hasValidHandle()) {
        return;
    }

    // try to fetch an image
    stat = xiGetImage(xiH_, image_capture_timeout_, &image_);

    // try to restart acq if it stopped for any reason
    while (stat == XI_ACQUISITION_STOPED) {
        // grabbing is stopped ?! (re)start grabbing
        std::cerr << "WARNING: ximera acquisition seems to be stopped, will restart now\n";

        stat = xiStartAcquisition(xiH_);
        errorHandling(stat, "xiStartAcquisition", "");

        stat = xiGetImage(xiH_, image_capture_timeout_, &image_);
    }

    if (stat != 0) {
        std::cerr << "Error on " << cam_name_ << ": xiGetImage resulted in error "
                  <<  stat << std::endl;
    }
}

void ximea_driver::setImageDataFormat(std::string image_format) {
    int image_data_format;

    if (!hasValidHandle()) {
        return;
    }

    if (image_format == std::string("XI_MONO16")) {
        image_data_format = XI_MONO16;
    } else if (image_format == std::string("XI_RGB24")) {
        image_data_format = XI_RGB24;
    } else if (image_format == std::string("XI_RGB32")) {
        image_data_format = XI_RGB32;
    } else if (image_format == std::string("XI_RGB_PLANAR")) {
        image_data_format = XI_MONO8;
        std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
    } else if (image_format == std::string("XI_RAW8")) {
        image_data_format = XI_RAW8;
    } else if (image_format == std::string("XI_RAW16")) {
        image_data_format = XI_RAW16;
    } else {
        image_data_format = XI_MONO8;
    }

    setParamInt(XI_PRM_IMAGE_DATA_FORMAT, image_data_format);

    image_data_format_ = image_data_format;
}

void ximea_driver::setROI(int l, int t, int w, int h) {
    int tmp;

    if (!hasValidHandle()) {
        return;
    }

    if (l < 0 || l > cam_resolution_w) {
        rect_left_ = 0;
    } else {
        rect_left_ = l;
    }
    if (t < 0 || t > cam_resolution_h) {
        rect_top_ = 0;
    } else {
        rect_top_ = t;
    }

    if (w < 0 || w > cam_resolution_w) {
        rect_width_ = cam_resolution_w;
    } else {
        rect_width_ = w;
    }

    if (h < 0 || h > cam_resolution_h) {
        rect_height_ = cam_resolution_h;
    } else {
        rect_height_ = h;
    }

    if (l + w > cam_resolution_w) {
        rect_left_ =  0;
        rect_width_ = cam_resolution_w;
    }
    if (h + t > cam_resolution_h) {
        rect_top_ =  0;
        rect_height_ = cam_resolution_h;
    }

    std::cout << "will set roi to: " << rect_width_ << "x" << rect_height_
              << " " << rect_left_ << " " << rect_top_ << std::endl;

    setParamInt(XI_PRM_WIDTH, rect_width_);
    setParamInt(XI_PRM_HEIGHT, rect_height_);
    setParamInt(XI_PRM_OFFSET_X, rect_left_);
    setParamInt(XI_PRM_OFFSET_Y, rect_top_);

    // show some info:
    tmp = getParamInt(XI_PRM_WIDTH XI_PRM_INFO_INCREMENT);
    std::cout << "width increment " << tmp << std::endl;

    tmp = getParamInt(XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT);
    std::cout << "height increment " << tmp << std::endl;

    tmp = getParamInt(XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT);
    std::cout << "left increment " << tmp << std::endl;

    tmp = getParamInt(XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT);
    std::cout << "top increment " << tmp << std::endl;
}

void ximea_driver::setExposure(int time) {
    bool result = setParamInt(XI_PRM_EXPOSURE, time);
    if (result) {
        exposure_time_ = time;
    }
}


int ximea_driver::readParamsFromFile(std::string file_name) {
    std::ifstream fin(file_name.c_str());
    if (fin.fail()) {
        ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
        exit(-1);
    }


    YAML::Node doc = YAML::LoadFile(file_name);
    std::string tmpS;
    int tmpI1, tmpI2, tmpI3, tmpI4;
    bool tmpB;

    // FIXME: add proper exception handling!
    try {
        serial_no_ = doc["serial_no"].as<int>();
    } catch (std::runtime_error) {}

    try {
        cam_name_ =  doc["cam_name"].as<std::string>();
    } catch (std::runtime_error) {}

    try {
        yaml_url_ = doc["yaml_url"].as<std::string>();
    } catch (std::runtime_error) {}

    try {
        cams_on_bus_ = doc["cams_on_bus"].as<int>();
    } catch (std::runtime_error) {}

    try {
        bandwidth_safety_margin_ = doc["bandwidth_safety_margin"].as<int>();
    } catch (std::runtime_error) {}

    try {
        frame_rate_ = doc["frame_rate"].as<int>();
    } catch (std::runtime_error) {}

    try {
        exposure_time_ = doc["exposure_time"].as<int>();
    } catch (std::runtime_error) {}

    try {
        auto_exposure_ = doc["auto_exposure"].as<bool>();
    } catch (std::runtime_error) {}

    try {
        binning_enabled_ = doc["binning_enabled"].as<bool>();
    } catch (std::runtime_error) {}

    try {
        downsample_factor_ = doc["downsample_factor_"].as<int>();
    } catch (std::runtime_error) {}

    try {
        rect_left_ = doc["rect_left"].as<int>();
    } catch (std::runtime_error) {}

    try {
        rect_top_ = doc["rect_top"].as<int>();
    } catch (std::runtime_error) {}
    try {
        rect_width_ = doc["rect_width"].as<int>();
    } catch (std::runtime_error) {}

    try {
        rect_height_ = doc["rect_height"].as<int>();
    } catch (std::runtime_error) {}

    try {
        allocated_bandwidth_ = doc["allocated_bandwidth"].as<float>();
    } catch (std::runtime_error) { std::cerr << "missing parameter allocated_bandwidth\n"; }

    setROI(rect_left_, rect_top_, rect_width_, rect_height_);
    try {
        image_data_format_ = doc["image_data_format"].as<std::string>();
    } catch (std::runtime_error) {}
    setImageDataFormat(image_data_format_);
}

void ximea_driver::enableTrigger(unsigned char trigger_mode) {
    if (!xiH_) {
        return;
    }

    trigger_mode_ = trigger_mode;
    if (trigger_mode_ > 3 || trigger_mode_ < 0) {
        trigger_mode_ = 0;
    }

    XI_RETURN stat;
    switch (trigger_mode_) {
        case (0):
            break;

        case (1):
            setParamInt(XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
            break;

        case (2):
            break;

        default:
            break;
    }
}

void ximea_driver::triggerDevice() {
    if (!xiH_) return;
    XI_RETURN stat;

    switch (trigger_mode_) {
        case (0):
            break;

        case (1):
            setParamInt(XI_PRM_TRG_SOFTWARE, 1);
            std::cout << "triggering " << cam_name_ <<  std::endl;
            break;

        case (2):
            break;

        default:
            break;
    }
}

// assign a part of available bandwidth to this cam:
void ximea_driver::limitBandwidth(float factor) {
    if (!xiH_) return;
    XI_RETURN stat;

    int bandwidth = getParamInt(XI_PRM_AVAILABLE_BANDWIDTH);
    std::cout << "measured total bandwith as " << bandwidth << "mbps\n";

    unsigned int cam_bandwidth = (bandwidth-bandwidth_safety_margin_) * allocated_bandwidth_;
    std::cout << "will assign a total of " << cam_bandwidth << "mbps ("
              << allocated_bandwidth_ << ") to this camera\n";

    setParamInt(XI_PRM_LIMIT_BANDWIDTH , cam_bandwidth);
}

inline bool ximea_driver::setParamInt(const char *param, int var, bool global) {
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiSetParamInt(handle, param, var);
    return errorHandling(stat, "xiSetParamInt", param, var);
}

inline int ximea_driver::getParamInt(const char *param, bool global) {
    int var;
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamInt(handle, param, &var);
    errorHandling(stat, "xiGetParamInt", param);
    return var;
}

inline bool ximea_driver::setParamFloat(const char *param, float var, bool global) {
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiSetParamFloat(handle, param, var);
    return errorHandling(stat, "setParamFloat", param, var);
}

inline float ximea_driver::getParamFloat(const char *param, bool global) {
    float var;
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamFloat(handle, param, &var);
    errorHandling(stat, "xiGetParamFloat", param);
    return var;
}

inline std::string ximea_driver::getParamString(const char *param, bool global) {
    char var[256];
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamString(handle, param, var, sizeof(var));
    errorHandling(stat, "xiGetParamString", param);
    return var;
}

