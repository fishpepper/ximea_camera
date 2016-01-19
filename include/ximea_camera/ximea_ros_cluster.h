/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#ifndef INCLUDE_XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H_
#define INCLUDE_XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Config.h>
#include <ximea_camera/ximea_ros_driver.h>

#include <string>
#include <vector>


class ximea_ros_cluster{
 public:
    explicit ximea_ros_cluster(int num_cams);
    explicit ximea_ros_cluster(std::vector < std::string > filenames);
    void add_camera(boost::shared_ptr<ximea_ros_driver> xd);
    void remove_camera(int serial_no);

    // cluster functions
    void clusterInit();
    void clusterAcquire();
    void clusterPublishImages();
    void clusterPublishCamInfo();
    void clusterPublishImageAndCamInfo();
    void clusterEnd();
    bool isDeviceOpen() {
        return devices_open_;
    }

    // individual camera functions (encapsulated for thread security)
    void setExposure(int serial_no, int time);
    void setImageDataFormat(int serial_no, std::string s);
    void setROI(int serial_no, int l, int t, int w, int h);

 private:
    std::vector<boost::shared_ptr<ximea_ros_driver> > cams_;
    std::vector<boost::thread*> threads_;
    bool devices_open_;
    int num_cams_;
    int getCameraIndex(int serial_no);
    const int USB_BUS_SAFETY_MARGIN;
    const int USB3_BANDWIDTH;
    bool fixed_init_;
};

#endif  // INCLUDE_XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H_
