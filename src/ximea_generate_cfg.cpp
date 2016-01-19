/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#include <m3api/xiApi.h>

#include <fstream>
#include <iostream>
#include <string>

#define check_xiapi_result(res, place, val) if (res != XI_OK) { cerr << "ERROR. " << place \
    << "(" << val << ") failed -> " << res << "\n"; exit(EXIT_FAILURE); }

using std::cout;
using std::cerr;
using std::ofstream;
using std::string;

enum property_type_t {PT_INT, PT_DOUBLE, PT_STRING, PT_BOOL};

typedef struct {
    string property;
    string description;
    string ptype;
} cfg_entry_t;

cfg_entry_t property[] = {
    {"gain", "Gain in dB", "double_t"},
    {"exposure", "Exposure time in us", "double_t"},
    {"aeag", "Automatic exposure/gain", "bool_t" },
    {"aeag_roi_offset_x", "Automatic exposure/gain ROI offset X", "int_t"},
    {"aeag_roi_offset_y", "Automatic exposure/gain ROI offset Y", "int_t"},
    {"aeag_roi_width", "Automatic exposure/gain ROI Width", "int_t"},
    {"aeag_roi_height", "Automatic exposure/gain ROI Height", "int_t"},
    {"bpc", "Correction of bad pixels", "bool_t"},
    {"auto_wb", "Automatic white balance", "bool_t"},
    {"width", "Width of the Image provided by the device (in pixels)", "int_t"},
    {"height", "Height of the Image provided by the device (in pixels)", "int_t"},
    {"offsetX", "Horizontal offset from the origin to the area of interest (in pixels)", "int_t"},
    {"offsetY", "Vertical offset from the origin to the area of interest (in pixels)", "int_t"}
};

/*
#gen.add("offsetX", double_t, 0, "Horizontal offset from the origin to the area of interest (in pixels). ", 0.5, 0, 1)
#gen.add("offsetY", double_t, 0, "Vertical offset from the origin to the area of interest (in pixels). ", 0.5, 0, 1)
#gen.add("region_selector", double_t, 0, "Selects Region in Multiple ROI which parameters are set by width, height, ... ,region mode ", 0.5, 0, 1)
#gen.add("region_mode", double_t, 0, "Activates/deactivates Region selected by Region Selector ", 0.5, 0, 1)
#gen.add("exp_priority", double_t, 0, "Exposure priority (0.8 - exposure 80%, gain 20%). ", 0.5, 0, 1)
#gen.add("ag_max_limit", double_t, 0, "Maximum limit of gain in AEAG procedure ", 0.5, 0, 1)
#gen.add("ae_max_limit", double_t, 0, "Maximum time (us) used for exposure in AEAG procedure ", 0.5, 0, 1)
#gen.add("aeag_level", double_t, 0, "Average intensity of output signal AEAG should achieve(in %) ", 0.5, 0, 1)
#gen.add("limit_bandwidth", double_t, 0, "Set/get bandwidth(datarate)(in Megabits) ", 0.5, 0, 1)
#gen.add("sensor_bit_depth", double_t, 0, "Sensor output data bit depth. XI_BIT_DEPTH", 0.5, 0, 1)
#gen.add("output_bit_depth", double_t, 0, "Device output data bit depth. XI_BIT_DEPTH", 0.5, 0, 1)
#gen.add("image_data_bit_depth", double_t, 0, "bitdepth of data returned by function xiGetImage XI_BIT_DEPTH", 0.5, 0, 1)
#gen.add("output_bit_packing", double_t, 0, "Device output data packing (or grouping) enabled. Packing could be enabled if output_data_bit_depth > 8 and packing capability is available. ", 0.5, 0, 1)
#gen.add("output_bit_packing_type", double_t, 0, "Data packing type. Some cameras supports only specific packing type. XI_OUTPUT_DATA_PACKING_TYPE", 0.5, 0, 1)
#gen.add("iscooled", double_t, 0, "Returns 1 for cameras that support cooling. ", 0.5, 0, 1)
#gen.add("cooling", double_t, 0, "Start camera cooling. ", 0.5, 0, 1)
#gen.add("target_temp", double_t, 0, "Set sensor target temperature for cooling. ", 0.5, 0, 1)
#gen.add("chip_temp", double_t, 0, "Camera sensor temperature ", 0.5, 0, 1)
#gen.add("hous_temp", double_t, 0, "Camera housing tepmerature ", 0.5, 0, 1)
#gen.add("hous_back_side_temp", double_t, 0, "Camera housing back side tepmerature ", 0.5, 0, 1)
#gen.add("cms", double_t, 0, "Mode of color management system. XI_CMS_MODE", 0.5, 0, 1)
#gen.add("apply_cms", double_t, 0, "Enable applying of CMS profiles to xiGetImage (see XI_PRM_INPUT_CMS_PROFILE, XI_PRM_OUTPUT_CMS_PROFILE). ", 0.5, 0, 1)
#gen.add("input_cms_profile", double_t, 0, "Filename for input cms profile (e.g. input.icc) ", 0.5, 0, 1)
#gen.add("output_cms_profile", double_t, 0, "Filename for output cms profile (e.g. input.icc) ", 0.5, 0, 1)
#gen.add("iscolor", double_t, 0, "Returns 1 for color cameras. ", 0.5, 0, 1)
#gen.add("cfa", double_t, 0, "Returns color filter array type of RAW data. XI_COLOR_FILTER_ARRAY", 0.5, 0, 1)
#gen.add("gammaY", double_t, 0, "Luminosity gamma ", 0.5, 0, 1)
#gen.add("gammaC", double_t, 0, "Chromaticity gamma ", 0.5, 0, 1)
#gen.add("sharpness", double_t, 0, "Sharpness Strenght ", 0.5, 0, 1)
#gen.add("ccMTX00", double_t, 0, "Color Correction Matrix element [0][0] ", 0.5, 0, 1)
#gen.add("ccMTX01", double_t, 0, "Color Correction Matrix element [0][1] ", 0.5, 0, 1)
#gen.add("ccMTX02", double_t, 0, "Color Correction Matrix element [0][2] ", 0.5, 0, 1)
#gen.add("ccMTX03", double_t, 0, "Color Correction Matrix element [0][3] ", 0.5, 0, 1)
#gen.add("ccMTX10", double_t, 0, "Color Correction Matrix element [1][0] ", 0.5, 0, 1)
#gen.add("ccMTX11", double_t, 0, "Color Correction Matrix element [1][1] ", 0.5, 0, 1)
#gen.add("ccMTX12", double_t, 0, "Color Correction Matrix element [1][2] ", 0.5, 0, 1)
#gen.add("ccMTX13", double_t, 0, "Color Correction Matrix element [1][3] ", 0.5, 0, 1)
#gen.add("ccMTX20", double_t, 0, "Color Correction Matrix element [2][0] ", 0.5, 0, 1)
#gen.add("ccMTX21", double_t, 0, "Color Correction Matrix element [2][1] ", 0.5, 0, 1)
#gen.add("ccMTX22", double_t, 0, "Color Correction Matrix element [2][2] ", 0.5, 0, 1)
#gen.add("ccMTX23", double_t, 0, "Color Correction Matrix element [2][3] ", 0.5, 0, 1)
#gen.add("ccMTX30", double_t, 0, "Color Correction Matrix element [3][0] ", 0.5, 0, 1)
#gen.add("ccMTX31", double_t, 0, "Color Correction Matrix element [3][1] ", 0.5, 0, 1)
#gen.add("ccMTX32", double_t, 0, "Color Correction Matrix element [3][2] ", 0.5, 0, 1)
#gen.add("ccMTX33", double_t, 0, "Color Correction Matrix element [3][3] ", 0.5, 0, 1)
#gen.add("defccMTX", double_t, 0, "Set default Color Correction Matrix ", 0.5, 0, 1)
#gen.add("trigger_source", double_t, 0, "Defines source of trigger. XI_TRG_SOURCE", 0.5, 0, 1)
#gen.add("trigger_software", double_t, 0, "Generates an internal trigger. XI_PRM_TRG_SOURCE must be set to TRG_SOFTWARE. ", 0.5, 0, 1)
#gen.add("trigger_selector", double_t, 0, "Selects the type of trigger. XI_TRG_SELECTOR", 0.5, 0, 1)
#gen.add("acq_frame_burst_count", double_t, 0, "Sets number of frames acquired by burst. This burst is used only if trigger is set to FrameBurstStart ", 0.5, 0, 1)
#gen.add("gpi_selector", double_t, 0, "Selects GPI XI_GPI_SELECTOR", 0.5, 0, 1)
#gen.add("gpi_mode", double_t, 0, "Defines GPI functionality XI_GPI_MODE", 0.5, 0, 1)
#gen.add("gpi_level", double_t, 0, "GPI level ", 0.5, 0, 1)
#gen.add("gpo_selector", double_t, 0, "Selects GPO XI_GPO_SELECTOR", 0.5, 0, 1)
#gen.add("gpo_mode", double_t, 0, "Defines GPO functionality XI_GPO_MODE", 0.5, 0, 1)
#gen.add("led_selector", double_t, 0, "Selects LED XI_LED_SELECTOR", 0.5, 0, 1)
#gen.add("led_mode", double_t, 0, "Defines LED functionality XI_LED_MODE", 0.5, 0, 1)
#gen.add("dbnc_en", double_t, 0, "Enable/Disable debounce to selected GPI ", 0.5, 0, 1)
#gen.add("dbnc_t0", double_t, 0, "Debounce time (x * 10us) ", 0.5, 0, 1)
#gen.add("dbnc_t1", double_t, 0, "Debounce time (x * 10us) ", 0.5, 0, 1)
#gen.add("dbnc_pol", double_t, 0, "Debounce polarity (pol = 1 t0 - falling edge, t1 - rising edge) ", 0.5, 0, 1)
#gen.add("lens_mode", double_t, 0, "Status of lens control interface. This shall be set to XI_ON before any Lens operations. ", 0.5, 0, 1)
#gen.add("lens_aperture_value", double_t, 0, "Current lens aperture value in stops. Examples: 2.8, 4, 5.6, 8, 11 ", 0.5, 0, 1)
#gen.add("lens_focus_move", double_t, 0, "Moves lens focus motor by steps set in XI_PRM_LENS_FOCUS_MOVEMENT_VALUE. ", 0.5, 0, 1)
#gen.add("lens_focus_distance", double_t, 0, "Lens focus distance in cm. ", 0.5, 0, 1)
#gen.add("lens_focal_length", double_t, 0, "Lens focal distance in mm. ", 0.5, 0, 1)
#gen.add("lens_feature_selector", double_t, 0, "Selects the current feature which is accessible by XI_PRM_LENS_FEATURE. XI_LENS_FEATURE", 0.5, 0, 1)
#gen.add("lens_feature", double_t, 0, "Allows access to lens feature value currently selected by XI_PRM_LENS_FEATURE_SELECTOR. ", 0.5, 0, 1)
#gen.add("device_name", double_t, 0, "Return device name ", 0.5, 0, 1)
#gen.add("device_type", double_t, 0, "Return device type ", 0.5, 0, 1)
#gen.add("device_model_id", double_t, 0, "Return device model id ", 0.5, 0, 1)
#gen.add("device_sn", double_t, 0, "Return device serial number ", 0.5, 0, 1)
#gen.add("device_sens_sn", double_t, 0, "Return sensor serial number ", 0.5, 0, 1)
#gen.add("device_id", double_t, 0, "Return unique device ID ", 0.5, 0, 1)
#gen.add("device_inst_path", double_t, 0, "Return device system instance path. ", 0.5, 0, 1)
#gen.add("device_loc_path", double_t, 0, "Represents the location of the device in the device tree. ", 0.5, 0, 1)
#gen.add("device_user_id", double_t, 0, "Return custom ID of camera. ", 0.5, 0, 1)
#gen.add("device_manifest", double_t, 0, "Return device capability description XML. ", 0.5, 0, 1)
#gen.add("imgdataformatrgb32alpha", double_t, 0, "The alpha channel of RGB32 output image format. ", 0.5, 0, 1)
#gen.add("imgpayloadsize", double_t, 0, "Buffer size in bytes sufficient for output image returned by xiGetImage ", 0.5, 0, 1)
#gen.add("transport_pixel_format", double_t, 0, "Current format of pixels on transport layer. XI_GenTL_Image_Format_e", 0.5, 0, 1)
#gen.add("sensor_clock_freq_hz", double_t, 0, "Sensor clock frequency in Hz. ", 0.5, 0, 1)
#gen.add("sensor_clock_freq_index", double_t, 0, "Sensor clock frequency index. Sensor with selected frequencies have possibility to set the frequency only by this index. ", 0.5, 0, 1)
#gen.add("framerate", double_t, 0, "Define framerate in Hz ", 0.5, 0, 1)
#gen.add("counter_selector", double_t, 0, "Select counter XI_COUNTER_SELECTOR", 0.5, 0, 1)
#gen.add("counter_value", double_t, 0, "Counter status ", 0.5, 0, 1)
#gen.add("acq_timing_mode", double_t, 0, "Type of sensor frames timing. XI_ACQ_TIMING_MODE", 0.5, 0, 1)
#gen.add("available_bandwidth", double_t, 0, "Calculate and return available interface bandwidth(int Megabits) ", 0.5, 0, 1)
#gen.add("buffer_policy", double_t, 0, "Data move policy XI_BP", 0.5, 0, 1)
#gen.add("LUTEnable", double_t, 0, "Activates LUT. ", 0.5, 0, 1)
#gen.add("LUTIndex", double_t, 0, "Control the index (offset) of the coefficient to access in the LUT. ", 0.5, 0, 1)
#gen.add("LUTValue", double_t, 0, "Value at entry LUTIndex of the LUT ", 0.5, 0, 1)
#gen.add("trigger_delay", double_t, 0, "Specifies the delay in microseconds (us) to apply after the trigger reception before activating it. XI_GPI_SELECTOR", 0.5, 0, 1)
#gen.add("ts_rst_mode", double_t, 0, "Defines how time stamp reset engine will be armed XI_TS_RST_MODE", 0.5, 0, 1)
#gen.add("ts_rst_source", double_t, 0, "Defines which source will be used for timestamp reset. Writing this parameter will trigger settings of engine (arming) XI_TS_RST_SOURCE", 0.5, 0, 1)
#gen.add("isexist", double_t, 0, "Returns 1 if camera connected and works properly. ", 0.5, 0, 1)
#gen.add("acq_buffer_size", double_t, 0, "Acquisition buffer size in buffer_size_unit. Default bytes. ", 0.5, 0, 1)
#gen.add("acq_buffer_size_unit", double_t, 0, "Acquisition buffer size unit in bytes. Default 1. E.g. Value 1024 means that buffer_size is in KiBytes ", 0.5, 0, 1)
#gen.add("buffers_queue_size", double_t, 0, "Queue of field/frame buffers ", 0.5, 0, 1)
#gen.add("recent_frame", double_t, 0, "GetImage returns most recent frame ", 0.5, 0, 1)
#gen.add("device_reset", double_t, 0, "Resets the camera to default state. ", 0.5, 0, 1)
#gen.add("column_fpn_correction", double_t, 0, "Correction of column FPN XI_SWITCH", 0.5, 0, 1)
#gen.add("sensor_mode", double_t, 0, "Current sensor mode. Allows to select sensor mode by one integer. Setting of this parameter affects: image dimensions and downsampling. XI_SENSOR_MODE", 0.5, 0, 1)
#gen.add("hdr", double_t, 0, "Enable High Dynamic Range feature. ", 0.5, 0, 1)
#gen.add("hdr_kneepoint_count", double_t, 0, "The number of kneepoints in the PWLR. ", 0.5, 0, 1)
#gen.add("hdr_t1", double_t, 0, "position of first kneepoint(in % of XI_PRM_EXPOSURE) ", 0.5, 0, 1)
#gen.add("hdr_t2", double_t, 0, "position of second kneepoint (in % of XI_PRM_EXPOSURE) ", 0.5, 0, 1)
#gen.add("hdr_kneepoint1", double_t, 0, "value of first kneepoint (% of sensor saturation) ", 0.5, 0, 1)
#gen.add("hdr_kneepoint2", double_t, 0, "value of second kneepoint (% of sensor saturation) ", 0.5, 0, 1)
#gen.add("image_black_level", double_t, 0, "Last image black level counts. Can be used for Offline processing to recall it. ", 0.5, 0, 1)
#gen.add("api_version", double_t, 0, "Returns version of API. ", 0.5, 0, 1)
#gen.add("drv_version", double_t, 0, "Returns version of current device driver. ", 0.5, 0, 1)
#gen.add("version_mcu1", double_t, 0, "Returns version of MCU1 firmware. ", 0.5, 0, 1)
#gen.add("version_mcu2", double_t, 0, "Returns version of MCU2 firmware. ", 0.5, 0, 1)
#gen.add("version_fpga1", double_t, 0, "Returns version of FPGA1 firmware. ", 0.5, 0, 1)
#gen.add("hw_revision", double_t, 0, "Returns hardware revision number. ", 0.5, 0, 1)
#gen.add("debug_level", double_t, 0, "Set debug level XI_DEBUG_LEVEL", 0.5, 0, 1)
#gen.add("read_file_ffs", double_t, 0, "Read file from camera flash filesystem. ", 0.5, 0, 1)
#gen.add("write_file_ffs", double_t, 0, "Write file to camera flash filesystem. ", 0.5, 0, 1)
#gen.add("ffs_file_name", double_t, 0, "Set name of file to be written/read from camera FFS. ", 0.5, 0, 1)
#gen.add("ffs_file_id", double_t, 0, "File number. ", 0.5, 0, 1)
#gen.add("ffs_file_size", double_t, 0, "Size of file. ", 0.5, 0, 1)
#gen.add("free_ffs_size", double_t, 0, "Size of free camera FFS. ", 0.5, 0, 1)
#gen.add("used_ffs_size", double_t, 0, "Size of used camera FFS. ", 0.5, 0, 1)
#gen.add("ffs_access_key", double_t, 0, "Setting of key enables file operations on some cameras. ", 0.5, 0, 1)
#gen.add("xiapi_context_list", double_t, 0, "List of current parameters settings context - parameters with values. Used for offline processing. ", 0.5, 0, 1)
#gen.add("sensor_feature_selector", double_t, 0, "Selects the current feature which is accessible by XI_PRM_SENSOR_FEATURE_VALUE. XI_SENSOR_FEATURE_SELECTOR", 0.5, 0, 1)
#gen.add("sensor_feature_value", double_t, 0, "Allows access to sensor feature value currently selected by XI_PRM_SENSOR_FEATURE_SELECTOR. ", 0.5, 0, 1)
#gen.add(":min", double_t, 0, "Parameter minimum", 0.5, 0, 1)
#gen.add(":max", double_t, 0, "Parameter maximum", 0.5, 0, 1)
#gen.add(":inc", double_t, 0, "Parameter increment", 0.5, 0, 1)
#gen.add(":info", double_t, 0, "Parameter value", 0.5, 0, 1)
#gen.add(":direct_update", double_t, 0, "Parameter modifier for direct update without stopping the streaming. E.g. XI_PRM_EXPOSURE XI_PRMM_DIRECT_UPDATE can be used with this modifier", 0.5, 0, 1)
#*/


int main(int argc, char ** argv) {
    XI_RETURN result;
    HANDLE handle;
    unsigned int camera_count;

    if (argc != 2) {
        cerr << "ERROR: invalid parameter count\n\n";
        cerr << "usage: " << argv[0] << " " << "<output file>\n\n";
        exit(EXIT_FAILURE);
    }

    // open output file:
    ofstream file_output;
    file_output.open(argv[1]);

    // simple helper program to dump allowed min/max settings to a ros cfg
    // fetch num of cams
    result = xiGetNumberDevices(&camera_count);
    check_xiapi_result(result, "xiGetNumberDevices", "");

    if (camera_count == 0) {
        cerr << "ERROR: no camera found\n";
        exit(EXIT_FAILURE);
    }

    cout << "xiApi: found " << camera_count << " ximea cameras on bus.\n";

    // fetch camera name
    char camera_name[256];
    result = xiGetDeviceInfoString(0, XI_PRM_DEVICE_NAME, camera_name, sizeof(camera_name));
    check_xiapi_result(result, "xiGetDeviceInfoString", XI_PRM_DEVICE_NAME);
    cout << "xiApi: will use Device 0 (" << camera_name << ")\n";

    // fetch camera handle
    result = xiOpenDevice(0, &handle);
    check_xiapi_result(result, "xiOpenDevice", 0);

    file_output << "#!/usr/bin/env python\n"
                << "PACKAGE = \"ximea_camera\"\n"
                << "from dynamic_reconfigure.parameter_generator_catkin import *\n"
                << "\n"
                << "gen = ParameterGenerator()\n\n";

    for (unsigned int i=0; i < sizeof(property) / sizeof(property[0]); i++) {
        float val_min;
        float val_max;
        float val_default;

        // fine tune some parameters:
        if (property[i].description == "exposure"){
            val_max = 100000;
        }

        result = xiGetParamFloat(handle, property[i].property.c_str(), &val_default);
        if (result != XI_OK) {
            cerr << "ERROR, could not fetch property " << property[i].property << "\n";
            continue;
        }

        result = xiGetParamFloat(handle, (property[i].property + XI_PRM_INFO_MIN).c_str(),
                                 &val_min);
        if (result != XI_OK) {
            cerr << "ERROR, could not fetch min property " << property[i].property << "\n";
            continue;
        }

        result = xiGetParamFloat(handle, (property[i].property + XI_PRM_INFO_MAX).c_str() ,
                                 &val_max);
        if (result != XI_OK) {
            cerr << "ERROR, could not fetch max property " << property[i].property << "\n";
            continue;
        }


        file_output << "gen.add(\""
                    << property[i].property << "\", "
                    << property[i].ptype << ", 0, \""
                    << property[i].description << "\", ";

        if ((property[i].ptype == "double_t") || (property[i].ptype == "int_t")) {
            file_output << val_default << ", "
                        << val_min << ", "
                        << val_max << ")\n";
        } else if (property[i].ptype == "bool_t") {
            file_output << (val_default?"True":"False") << ")\n";
        } else {
            cerr << "ERROR: type " << property[i].ptype << " not supported yet\n";
            exit(EXIT_FAILURE);
        }
    }

    file_output << "\nexit(gen.generate(PACKAGE, \"ximea_camera\", \"xiAPI\"))\n\n";

    cout << "success, wrote output to " << argv[1] << "\n";

    xiCloseDevice(handle);
    return(EXIT_SUCCESS);
}
