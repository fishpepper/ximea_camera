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
#ifndef INCLUDE_XIMEA_CAMERA_ROS_CLUSTER_H_
#define INCLUDE_XIMEA_CAMERA_ROS_CLUSTER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Config.h>
#include <ximea_camera/ros_driver.h>

#include <string>
#include <vector>

namespace ximea_camera {

class ros_cluster {
 public:
    explicit ros_cluster(int num_cams);
    explicit ros_cluster(std::vector < std::string > filenames);
    void add_camera(boost::shared_ptr<ros_driver> xd);
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
    std::vector<boost::shared_ptr<ros_driver> > cams_;
    std::vector<boost::thread*> threads_;
    bool devices_open_;
    int num_cams_;
    int getCameraIndex(int serial_no);
    bool fixed_init_;
};

}  // namespace ximea_camera

#endif  // INCLUDE_XIMEA_CAMERA_ROS_CLUSTER_H_
