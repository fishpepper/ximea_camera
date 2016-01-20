/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#ifndef INCLUDE_XIMEA_CAMERA_XIMEA_NODELET_H_
#define INCLUDE_XIMEA_CAMERA_XIMEA_NODELET_H_
#include <signal.h>

#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "ximea_camera/ximea_ros_driver.h"

namespace ximea_camera {

/** ximea xiAPI camera driver nodelet implementation. */
class ximea_nodelet : public nodelet::Nodelet{
 public:
  ximea_nodelet();
  ~ximea_nodelet();

 private:
  virtual void onInit();
  virtual void devicePoll();

  volatile bool running_;
  boost::shared_ptr<ximea_ros_driver> drv_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

}  // namespace ximea_camera

#endif  // INCLUDE_XIMEA_CAMERA_XIMEA_NODELET_H_
