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

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ximea_camera/driver.h>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>

#define driver_DEBUG_LEVEL XI_DL_FATAL    // XI_DL_DETAIL

using ximea_camera::driver;

driver::driver(int serial_no, std::string cam_name) {
    serial_no_ = serial_no;
    cam_name_ = cam_name;
    assignDefaultValues();
}

void driver::assignDefaultValues() {
    cam_resolution_w_ = 0;
    cam_resolution_h_ = 0;
    bayer_filter_array_ = XI_CFA_NONE;
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
    rect_width_ = cam_resolution_w_;
    rect_height_ = cam_resolution_h_;
    xiH_ = NULL;
    // pre init all XI_IMG fields to zero:
    memset(&image_, 0, sizeof(XI_IMG));
    image_.size = sizeof(XI_IMG);
    image_.bp = NULL;
    image_.bp_size = 0;
    acquisition_active_ = false;
    image_capture_timeout_ = 1000;
    camera_to_localtime_offset_ = boost::posix_time::microsec_clock::local_time();
}

driver::driver(std::string file_name) {
    assignDefaultValues();
    readParamsFromFile(file_name);
    ROS_INFO_STREAM("driver: reading paramter values from file: " << file_name);
}

bool driver::errorHandling(XI_RETURN ret, std::string command,
                                 std::string param, float val) {
    if (ret != XI_OK) {
        std::cout << "driver: " << command << "(" << param << ", "<< val
                  << " ) failed (errno " << ret << ", handle=" << xiH_ << ")\n";
        // handle error cases:
        if (ret == XI_INVALID_ARG) {
             throw std::invalid_argument("xiAPI: invalid parameter (XI_INVALID_ARG)");
        } else if (ret == XI_WRONG_PARAM_VALUE) {
            throw std::invalid_argument("xiAPI: invalid parameter value passed "
                                        "(XI_WRONG_PARAM_VALUE)");
        } else {
            closeDevice();
            exit(EXIT_FAILURE);
        }
        return false;
    } else {
        return true;
    }
}

void driver::fetchValues() {
    // fetch frame size from cam:
    cam_resolution_w_ = getParamInt(XI_PRM_WIDTH XI_PRM_INFO_MAX);
    cam_resolution_h_ = getParamInt(XI_PRM_HEIGHT XI_PRM_INFO_MAX);

    // fetch bayer pattern from cam:
    bayer_filter_array_ = getParamInt(XI_PRM_COLOR_FILTER_ARRAY);
}


void driver::applyParameters() {
    setImageDataFormat(image_data_format_);
    setExposure(exposure_time_);
    setROI(rect_left_, rect_top_, rect_width_, rect_height_);
}

void driver::openDevice() {
    XI_RETURN stat;

    // set xiapi debug level:
    setParamInt(XI_PRM_DEBUG_LEVEL, driver_DEBUG_LEVEL, true);

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
              << camera_name << "' (handle " << xiH_ << ")\n";

    // limit bandwidth
    // in case multiple cameras share the same bus we
    // have to manually split the overall bandwidth between devices
    // see allocated_bandwidth parameter in config file
    limitBandwidth(allocated_bandwidth_);

    fetchValues();

    applyParameters();

    sync_camera_timestamp();
}

void driver::sync_camera_timestamp() {
    // this will store the current time as a timestamp
    // and trigger a reset of the cameras internal timestamp
    // assuming that the setParamInt call is fast enough
    // we should get a good estimate for the difference between
    // the system time and the camera timestamp
    //
    // FIXME: this should be done regularly -> find a goot time/place to call this
    //        every second or so...

    // store local time
    camera_to_localtime_offset_ = boost::posix_time::microsec_clock::local_time();

    // and trigger a timestamp reset, this will immediately reset the internal ts counter to zero
    setParamInt(XI_PRM_TS_RST_SOURCE, XI_TS_RST_SRC_SW);
}

void driver::closeDevice() {
    if (xiH_) {
        xiCloseDevice(xiH_);
        xiH_ = NULL;
    }
}

void driver::stopAcquisition() {
    XI_RETURN stat;
    if (!hasValidHandle()) {
        return;
    }

    stat = xiStopAcquisition(xiH_);
    errorHandling(stat, "xiStopAcquisition", "");

    acquisition_active_ = false;
}

void driver::startAcquisition() {
    if (!hasValidHandle()) {
        return;
    }
    XI_RETURN stat = xiStartAcquisition(xiH_);
    errorHandling(stat, "xiStartAcquisition", "");

    acquisition_active_ = true;
}

void driver::acquireImage() {
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

void driver::setImageDataFormat(std::string image_format) {
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

    std::cout << "xiApi: setting image format to " << image_data_format << std::endl;
    setParamInt(XI_PRM_IMAGE_DATA_FORMAT, image_data_format);

    image_data_format_ = image_data_format;
}

void driver::setROI(int l, int t, int w, int h) {
    int tmp;

    if (!hasValidHandle()) {
        return;
    }

    if (l < 0 || l > cam_resolution_w_) {
        rect_left_ = 0;
    } else {
        rect_left_ = l;
    }
    if (t < 0 || t > cam_resolution_h_) {
        rect_top_ = 0;
    } else {
        rect_top_ = t;
    }

    if (w < 0 || w > cam_resolution_w_) {
        rect_width_ = cam_resolution_w_;
    } else {
        rect_width_ = w;
    }

    if (h < 0 || h > cam_resolution_h_) {
        rect_height_ = cam_resolution_h_;
    } else {
        rect_height_ = h;
    }

    if (l + w > cam_resolution_w_) {
        rect_left_ =  0;
        rect_width_ = cam_resolution_w_;
    }
    if (h + t > cam_resolution_h_) {
        rect_top_ =  0;
        rect_height_ = cam_resolution_h_;
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

void driver::setExposure(int time) {
    bool result = setParamInt(XI_PRM_EXPOSURE, time);
    if (result) {
        exposure_time_ = time;
    }
}


int driver::readParamsFromFile(std::string file_name) {
    std::ifstream fin(file_name.c_str());
    if (fin.fail()) {
        ROS_ERROR_STREAM("could not open file '" << file_name.c_str() << "'" << std::endl);
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
    } catch (std::runtime_error) { std::cerr << "missing parameter allocated_bandwidth. "
                                                "this is mandatory!\n";
                                   exit(EXIT_FAILURE); }

    try {
        use_cam_timestamp_ = doc["use_cam_timestamp"].as<bool>();
        std::cerr << doc["use_cam_timestamp"];
    } catch (std::runtime_error) { std::cerr << "missing parameter use_cam_timestamp";}

    setROI(rect_left_, rect_top_, rect_width_, rect_height_);
    try {
        image_data_format_ = doc["image_data_format"].as<std::string>();
    } catch (std::runtime_error) {}
    setImageDataFormat(image_data_format_);
}

void driver::enableTrigger(unsigned char trigger_mode) {
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

void driver::triggerDevice() {
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
void driver::limitBandwidth(float factor) {
    if (!xiH_) return;
    XI_RETURN stat;

    int bandwidth = getParamInt(XI_PRM_AVAILABLE_BANDWIDTH);
    std::cout << "measured total bandwith as " << bandwidth << "mbps\n";

    unsigned int cam_bandwidth = (bandwidth-bandwidth_safety_margin_) * allocated_bandwidth_;
    std::cout << "will assign a total of " << cam_bandwidth << "mbps ("
              << allocated_bandwidth_ << ") to this camera\n";

    setParamInt(XI_PRM_LIMIT_BANDWIDTH , cam_bandwidth);
}

bool driver::setParamInt(const char *param, int var, bool global) {
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiSetParamInt(handle, param, var);
    bool result = errorHandling(stat, "xiSetParamInt", param, var);

    // read back non global values to make sure we really suceeded
    // and keep our data consistent:
    if ((result) && (!global)) {
        int_param_map[param] = getParamInt(param, global);
    }

    return result;
}

int driver::getParamInt(const char *param, bool global) {
    int var;
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamInt(handle, param, &var);
    errorHandling(stat, "xiGetParamInt", param);

    // store non global values in order to keep our data consistent:
    if (!global) {
        int_param_map[param] = var;
    }

    return var;
}

bool driver::setParamFloat(const char *param, float var, bool global) {
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiSetParamFloat(handle, param, var);
    bool result = errorHandling(stat, "setParamFloat", param, var);

    // read back non global values to make sure we really suceeded
    // and keep our data consistent:
    if ((result) && (!global)) {
        float_param_map[param] = getParamFloat(param, global);
    }

    return result;
}

float driver::getParamFloat(const char *param, bool global) {
    float var;
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamFloat(handle, param, &var);
    errorHandling(stat, "xiGetParamFloat", param);

    // store non global values in order to keep our data consistent:
    if (!global) {
        float_param_map[param] = var;
    }

    return var;
}

std::string driver::getParamString(const char *param, bool global) {
    char var[256];
    // global or device handle?
    HANDLE handle = (global)?0:xiH_;
    XI_RETURN stat = xiGetParamString(handle, param, var, sizeof(var));
    errorHandling(stat, "xiGetParamString", param);
    return var;
}
