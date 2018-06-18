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

#include<cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <ximea_camera/ros_driver.h>

#include <algorithm>
#include <string>
#include <vector>

using ximea_camera::RosDriver;

RosDriver::RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
                     std::string cam_name, int serial_no,
                     std::string yaml_url) : Driver(serial_no, cam_name) {
    commonInitialize(nh);
}

RosDriver::RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : Driver() {
    // fetch config from launch file parameters
    pnh.getParam("serial", serial_no_);
    pnh.getParam("name", cam_name_);
    pnh.getParam("calibration", yaml_url_);
    pnh.getParam("cams_on_bus", cams_on_bus_);
    pnh.getParam("use_cam_timestamp", use_cam_timestamp_);
    pnh.getParam("allocated_bandwidth", allocated_bandwidth_);
    pnh.getParam("image_data_format", image_data_format_);

    commonInitialize(nh);
}

RosDriver::RosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
                     std::string file_name) : Driver(file_name) {
    commonInitialize(nh);
}

void RosDriver::commonInitialize(const ros::NodeHandle &nh) {
    pnh_ = nh;
    cam_info_manager_ = new camera_info_manager::CameraInfoManager(pnh_, cam_name_);
    cam_info_manager_->loadCameraInfo(yaml_url_);

    it_ = new image_transport::ImageTransport(nh);
    ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
    cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);

    ROS_INFO_STREAM("config[serial]      = " << serial_no_);
    ROS_INFO_STREAM("config[name]        = " << cam_name_);
    ROS_INFO_STREAM("config[calibration] = " << yaml_url_);
}

void RosDriver::attachToDynamicReconfigureServer() {
    // attach to dynamic reconfigure server
    // IMPORTANT: do this after the cam is set up in order
    // to set the default values (or the values passed by the launch file params)
    // immediately
    ROS_DEBUG("ximea_camera: connecting to dynamic reconfiguration server");
    ros::NodeHandle reconf_node(pnh_, "settings");
    reconf_server_ = new dynamic_reconfigure::Server<ximea_camera::xiAPIConfig>(reconf_node);
    reconf_server_->setCallback(boost::bind(&RosDriver::dynamicReconfigureCallback, this, _1, _2));
}

void RosDriver::publishImage(const ros::Time & ts) {
    // cast xiapi buffer to cam buffer
    cam_buffer_ = reinterpret_cast<char *>(image_.bp);
    cam_buffer_size_ = image_.width * image_.height * bpp_;

    // store value in ros msg
    ros_image_.header.stamp = ts;
    ros_image_.header.frame_id = cam_name_ + "_optical_frame";

    // setup image parameters
    ros_image_.data.resize(cam_buffer_size_);
    ros_image_.encoding = encoding_;
    ros_image_.width  = image_.width;
    ros_image_.height = image_.height;
    ros_image_.step   = image_.width * bpp_;

    // copy data to ros message
    copy(reinterpret_cast<char *>(cam_buffer_),
         (reinterpret_cast<char *>(cam_buffer_)) + cam_buffer_size_,
         ros_image_.data.begin());

    // publish message
    ros_cam_pub_.publish(ros_image_);
}

void RosDriver::publishCamInfo(const ros::Time &ts) {
    cam_info_ = cam_info_manager_->getCameraInfo();
    cam_info_.header.stamp = ts;
    cam_info_.header.frame_id = cam_name_ + "_optical_frame";
    cam_info_pub_.publish(cam_info_);
}

ros::Time RosDriver::getTimestamp() {
    // set timestamp:
    ros::Time timestamp;
    if (use_cam_timestamp_) {
        // use camera timestamp
        // camera timestamp was reset during ximea_driver::sync_camera_timestamp() call
        // and the offset to the local time was stored. in order to calc
        // the actual timestamp we have to add those two values:
        timestamp = ros::Time::fromBoost(camera_to_localtime_offset_)
                + ros::Duration(image_.tsSec, image_.tsUSec*1000L);
    } else {
        // use Time::now()
        timestamp = ros::Time::now();
    }
    return timestamp;
}

void RosDriver::publishImageAndCamInfo() {
    ros::Time timestamp = getTimestamp();

    publishImage(timestamp);
    publishCamInfo(timestamp);
}


void RosDriver::setImageDataFormat(std::string image_format) {
    XI_RETURN stat;
    int image_data_format;

    if (!hasValidHandle()) {
        return;
    }

    if (image_format == std::string("XI_MONO16")) {
        image_data_format = XI_MONO16;
        encoding_ = sensor_msgs::image_encodings::MONO16;
        bpp_ = 2;
    } else if (image_format == std::string("XI_RGB24")) {
        image_data_format = XI_RGB24;
        encoding_ = sensor_msgs::image_encodings::BGR8;
        bpp_ = 3;
    } else if (image_format == std::string("XI_RGB32")) {
        image_data_format = XI_RGB32;
        encoding_ = sensor_msgs::image_encodings::BGR16;
        bpp_ = 3;
    } else if (image_format == std::string("XI_RGB_PLANAR")) {
        image_data_format = XI_MONO8;
        ROS_ERROR("ximea_camera: this image format (%s) is unsupported in ROS defaulting to MONO8",
                 image_format.c_str());
        encoding_ = sensor_msgs::image_encodings::MONO8;
        bpp_ = 1;
    } else if (image_format == std::string("XI_RAW8")) {
        image_data_format = XI_RAW8;
        // in order to get the right bayer pattern we need to know the
        // bayer pattern of the sensor
        switch (bayer_filter_array_) {
        default:
        case(XI_CFA_NONE):
        case(XI_CFA_CMYG):
        case(XI_CFA_RGR):
            // fallback for invalid/unsupported values
            ROS_ERROR("ximea_camera: unknown sensor bayer pattern, defaulting to mono8");
            encoding_ = sensor_msgs::image_encodings::MONO8;
            break;

        case(XI_CFA_BAYER_RGGB):
            encoding_ = sensor_msgs::image_encodings::BAYER_RGGB8;
            break;

        case(XI_CFA_BAYER_BGGR):
            encoding_ = sensor_msgs::image_encodings::BAYER_BGGR8;
            break;

        case(XI_CFA_BAYER_GRBG):
            encoding_ = sensor_msgs::image_encodings::BAYER_GRBG8;
            break;

        case(XI_CFA_BAYER_GBRG):
            encoding_ = sensor_msgs::image_encodings::BAYER_GBRG8;
            break;
        }
        bpp_ = 1;
    } else if (image_format == std::string("XI_RAW16")) {
        image_data_format = XI_RAW16;
        // in order to get the right bayer pattern we need to know the
        // bayer pattern of the sensor
        switch (bayer_filter_array_) {
        default:
        case(XI_CFA_NONE):
        case(XI_CFA_CMYG):
        case(XI_CFA_RGR):
            // fallback for invalid/unsupported values
            encoding_ = sensor_msgs::image_encodings::MONO16;
            break;

        case(XI_CFA_BAYER_RGGB):
            encoding_ = sensor_msgs::image_encodings::BAYER_RGGB16;
            break;

        case(XI_CFA_BAYER_BGGR):
            encoding_ = sensor_msgs::image_encodings::BAYER_BGGR16;
            break;

        case(XI_CFA_BAYER_GRBG):
            encoding_ = sensor_msgs::image_encodings::BAYER_GRBG16;
            break;

        case(XI_CFA_BAYER_GBRG):
            encoding_ = sensor_msgs::image_encodings::BAYER_GBRG16;
            break;
        }
        bpp_ = 2;
    } else {
        image_data_format = XI_MONO8;
        encoding_ = sensor_msgs::image_encodings::MONO8;
        bpp_ = 1;
    }

    setParamInt(XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
    // FIXME: if we cannot set the format then there is something wrong
    // we should probably quit then..
    image_data_format_ = image_data_format;
}

void RosDriver::applyParameters() {
    setImageDataFormat(image_data_format_);
    attachToDynamicReconfigureServer();
}

bool RosDriver::dynamicReconfigureInt(const char *param, int value) {
    if (int_param_map.find(param) != int_param_map.end()) {
        if (int_param_map[param] == value) {
            // entry found & value matches -> no update necessary
            ROS_DEBUG("dynamicReconfigureInt(%s, %d) not executed, "
                      "cached value matches new value\n", param, value);
            return true;
        }
    }

    // else: we have to update the camera data:
    return setParamInt(param, value);
}

bool RosDriver::dynamicReconfigureFloat(const char *param, float value) {
    if (float_param_map.find(param) != float_param_map.end()) {
        if (float_param_map[param] == value) {
            // entry found & value matches -> no update necessary
            ROS_DEBUG("dynamicReconfigureFloat(%s, %f) not executed, "
                      "cached value matches new value\n", param, value);
            return true;
        }
    }

    // else: we have to update the camera data:
    return setParamFloat(param, value);
}

void RosDriver::dynamicReconfigureCallback(const ximea_camera::xiAPIConfig &config,
                                                    uint32_t level) {
    // ignore incoming requests as long cam is not set up properly
    if (!hasValidHandle()) {
        return;
    }

    // use some tricks to iterate through all config entries:
    std::vector<ximea_camera::xiAPIConfig::AbstractParamDescriptionConstPtr>::const_iterator _i;
    for (_i = config.__getParamDescriptions__().begin();
         _i != config.__getParamDescriptions__().end(); ++_i) {
        try {
            boost::any val;
            boost::shared_ptr<const ximea_camera::xiAPIConfig::AbstractParamDescription>
                   description = *_i;

            // fetch actual value:
            description->getValue(config, val);

            ROS_DEBUG("dynamicReconfigure request: name=%s type=%s ",
                      description->name.c_str(), description->type.c_str());

            // copy data to ximea api:
            if (description->type == "double") {
                ROS_DEBUG("%f", static_cast<float>(boost::any_cast<double>(val)));
                dynamicReconfigureFloat(description->name.c_str(),
                                          static_cast<float>(boost::any_cast<double>(val)));
            } else if (description->type == "bool") {
                ROS_DEBUG("%d", (boost::any_cast<bool>(val))?1:0);
                dynamicReconfigureInt(description->name.c_str(), (boost::any_cast<bool>(val))?1:0);
            } else if (description->type == "int") {
                ROS_DEBUG("%d", boost::any_cast<int>(val));
                dynamicReconfigureInt(description->name.c_str(), boost::any_cast<int>(val));
            } else {
                std::cerr << "ERROR: unsupported config type " << description->type  << "\n";
            }
        } catch (std::invalid_argument exception) {
            ROS_ERROR("failed to set parameter, xiapi returned an error code");
        }
    }
}
