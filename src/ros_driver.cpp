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

#include <sensor_msgs/image_encodings.h>
#include <ximea_camera/ros_driver.h>

#include <algorithm>
#include <string>
#include <vector>

using ximea_camera::ros_driver;

ros_driver::ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no,
                                   std::string yaml_url) : driver(serial_no, cam_name) {
    common_initialize(nh);
}

ros_driver::ros_driver(const ros::NodeHandle &nh, std::string file_name) : driver(file_name) {
    common_initialize(nh);
}

void ros_driver::common_initialize(const ros::NodeHandle &nh) {
    pnh_ = nh;
    cam_info_manager_ = new camera_info_manager::CameraInfoManager(pnh_, cam_name_);
    cam_info_manager_->loadCameraInfo(yaml_url_);

    it_ = new image_transport::ImageTransport(nh);
    ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
    cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);

    // connect to dynamic reconf server
    std::cout << "connecting to dynamic reconfiguration server\n";

    // dynamic reconfig
    ros::NodeHandle reconf_node(pnh_, "settings");
    server = new dynamic_reconfigure::Server<ximea_camera::xiAPIConfig>(reconf_node);
    server->setCallback(boost::bind(&ros_driver::dynamic_reconfigure_callback,
                                    this, _1, _2));
}

void ros_driver::publishImage(const ros::Time & now) {
    // cast xiapi buffer to cam buffer
    cam_buffer_ = reinterpret_cast<char *>(image_.bp);
    cam_buffer_size_ = image_.width * image_.height * bpp_;

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
        // use incoming time from ros::Time::now() call
        timestamp = now;
    }

    // store value in ros msg
    ros_image_.header.stamp = timestamp;

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

void ros_driver::publishCamInfo(const ros::Time &now) {
    cam_info_.header.stamp = now;
    cam_info_ = cam_info_manager_->getCameraInfo();
    cam_info_pub_.publish(cam_info_);
}

void ros_driver::publishImageAndCamInfo() {
    ros::Time now = ros::Time::now();
    publishImage(now);
    publishCamInfo(now);
}


void ros_driver::setImageDataFormat(std::string image_format) {
    XI_RETURN stat;
    int image_data_format;

    if (!hasValidHandle()) {
        return;
    }

    if (image_format == std::string("XI_MONO16")) {
        image_data_format = XI_MONO16;
        encoding_ = std::string("mono16");
        bpp_ = 2;
    } else if (image_format == std::string("XI_RGB24")) {
        image_data_format = XI_RGB24;
        encoding_ = std::string("bgr8");
        bpp_ = 3;
    } else if (image_format == std::string("XI_RGB32")) {
        image_data_format = XI_RGB32;
        encoding_ = std::string("bgr16");
        bpp_ = 3;
    } else if (image_format == std::string("XI_RGB_PLANAR")) {
        image_data_format = XI_MONO8;
        std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
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
            encoding_ = std::string("mono8");
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
            encoding_ = std::string("mono16");
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
        encoding_ = std::string("mono8");
        bpp_ = 1;
    }

    setParamInt(XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
    // FIXME: if we cannot set the format then there is something wrong
    // we should probably quit then..

    image_data_format_ = image_data_format;
}

bool ros_driver::dynamic_reconfigure_int(const char *param, int value) {
    if (int_param_map.find(param) != int_param_map.end()) {
        if (int_param_map[param] == value) {
            // entry found & value matches -> no update necessary
            return true;
        }
    }

    // else: we have to update the camera data:
    return setParamInt(param, value);
}

bool ros_driver::dynamic_reconfigure_float(const char *param, float value) {
    if (float_param_map.find(param) != float_param_map.end()) {
        if (float_param_map[param] == value) {
            // entry found & value matches -> no update necessary
            return true;
        }
    }

    // else: we have to update the camera data:
    return setParamFloat(param, value);
}

void ros_driver::dynamic_reconfigure_callback(const ximea_camera::xiAPIConfig &config,
                                                    uint32_t level) {
    // ignore incoming requests as long cam is not set up properly
    if (!hasValidHandle()) {
        return;
    }

    // use some tricks to iterate through all config entries:
    std::vector<ximea_camera::xiAPIConfig::AbstractParamDescriptionConstPtr>::const_iterator _i;
    for (_i = config.__getParamDescriptions__().begin();
         _i != config.__getParamDescriptions__().end(); ++_i) {
        boost::any val;
        boost::shared_ptr<const ximea_camera::xiAPIConfig::AbstractParamDescription>
                description = *_i;

        // fetch actual value:
        description->getValue(config, val);

        //  std::cout << description->name << " " << description->type << "\n";

        // copy data to ximea api:
        if (description->type == "double") {
            dynamic_reconfigure_float(description->name.c_str(),
                                      static_cast<float>(boost::any_cast<double>(val)));
        } else if (description->type == "bool") {
            dynamic_reconfigure_int(description->name.c_str(), (boost::any_cast<bool>(val))?1:0);
        } else if (description->type == "int") {
            dynamic_reconfigure_int(description->name.c_str(), boost::any_cast<int>(val));
        } else {
            std::cerr << "ERROR: unsupported config type " << description->type  << "\n";
        }
    }
}
