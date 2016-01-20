#include <ros/ros.h>
#include <nodelet/loader.h>
#include <string>


int main(int arc, char **argv) {
    ros::init(arc, argv, "xi_provider_node");
    nodelet::Loader manager(true);
    // nodelet::M_string remap(ros::names::getRemappings());
    nodelet::M_string remap;
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    manager.load(nodelet_name, "ximea_camera/ximea_nodelet", remap, nargv);
    ros::spin();
}
