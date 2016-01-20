/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/
#ifndef INCLUDE_XIMEA_CAMERA_XIMEA_PARAM_H_
#define INCLUDE_XIMEA_CAMERA_XIMEA_PARAM_H_
#include <m3api/xiApi.h>
#include <map>
#include <string>
/*
typedef struct {
    std::string name;
    std::string description;
    float value;
    XI_PRM_TYPE ptype;
} ximea_param_t;

typedef std::map<std::string, ximea_param_t> ximea_param_map_t;

#define XIMEA_PARAM_MAP_POPULATE(_map, _name, _description, _ptype, _value) { \
    ximea_param_t _p; \
    _p.name = _name; \
    _p.description = _description; \
    _p.value = _value; \
    _p.ptype = _ptype; \
    _map[_name] = _p; \
}

ximea_param_map_t populate_ximea_param_map() {
    ximea_param_map_t m;

    XIMEA_PARAM_MAP_POPULATE(m,
                             "exposure",
                             "Exposure time in microseconds ",
                             xiTypeInteger,
                             0.0);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "exposure_burst_count",
                             "Sets the number of times of exposure in one frame. ",
                             xiTypeInteger,
                             1.0);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "gain_selector",
                             "Gain selector for parameter Gain allows to select different type "
                             "of gains. XI_GAIN_SELECTOR_TYPE",
                             xiTypeInteger,
                             XI_GAIN_SELECTOR_ANALOG_ALL);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "gain",
                             "Gain in dB ",
                             xiTypeFloat,
                             0.0);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "downsampling",
                             "Change image resolution by binning or skipping.XI_DOWNSAMPLING_VALUE",
                             xiTypeInteger,
                             1.0);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "downsampling_type",
                             "Change image downsampling type. XI_DOWNSAMPLING_TYPE",
                             xiTypeInteger,
                             XI_BINNING);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "binning_selector",
                             "Binning engine selector. XI_BIN_SELECTOR",
                             xiTypeInteger,
                             XI_BIN_SELECT_SENSOR);

    XIMEA_PARAM_MAP_POPULATE(m,
                             "binning_vertical",
                             "Vertical Binning - number of vertical photo-sensitive cells to "
                             "combine together. ",
                             xiTypeInteger,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "binning_horizontal",
                             "Horizontal Binning - number of horizontal photo-sensitive cells to "
                             "combine together. ",
                             xiTypeInteger,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "binning_pattern",
                             "Binning pattern type. XI_BIN_PATTERN",
                             xiTypeInteger,
                             XI_BIN_BAYER);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "decimation_selector",
                             "Decimation engine selector. XI_DEC_SELECTOR",
                             xiTypeInteger,
                             XI_DEC_SELECT_SENSOR);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "decimation_vertical",
                             "Vertical Decimation - vertical sub-sampling of the image - reduces "
                             "the vertical resolution of the image by the specified vertical "
                             "decimation factor. ",
                             xiTypeInteger,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "decimation_horizontal",
                             "Horizontal Decimation - horizontal sub-sampling of the image - "
                             "reduces the horizontal resolution of the image by the specified "
                             "vertical decimation factor. ",
                             xiTypeInteger,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "decimation_pattern",
                             "Decimation pattern type. XI_DEC_PATTERN",
                             xiTypeInteger,
                             XI_DEC_BAYER);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "test_pattern",
                             "Selects which test pattern type is generated by the selected "
                             "generator. XI_TEST_PATTERN",
                             xiTypeInteger,
                             0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "imgdataformat",
                             "Output data format. XI_IMG_FORMAT",
                             xiTypeInteger,
                             0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "shutter_type",
                             "Change sensor shutter type(CMOS sensor). XI_SHUTTER_TYPE",
                             xiTypeInteger,
                             XI_SHUTTER_GLOBAL);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sensor_taps",
                             "Number of taps XI_SENSOR_TAP_CNT",
                             xiTypeInteger,
                             1);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag",
                             "Automatic exposure/gain ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag_roi_offset_x",
                             "Automatic exposure/gain ROI offset X ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag_roi_offset_y",
                             "Automatic exposure/gain ROI offset Y ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag_roi_width",
                             "Automatic exposure/gain ROI Width ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag_roi_height",
                             "Automatic exposure/gain ROI Height ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "bpc",
                             "Correction of bad pixels ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "auto_wb",
                             "Automatic white balance ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "manual_wb",
                             "Calculates White Balance(xiGetImage function must be called) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "wb_kr",
                             "White balance red coefficient ",
                             xiTypeFloat,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "wb_kg",
                             "White balance green coefficient ",
                             xiTypeFloat,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "wb_kb",
                             "White balance blue coefficient ",
                             xiTypeFloat,
                             1.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "width",
                             "Width of the Image provided by the device (in pixels). ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "height",
                             "Height of the Image provided by the device (in pixels). ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,

                             "offsetX",
                             "Horizontal offset from the origin to the area of interest "
                             "(in pixels)",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "offsetY",
                             "Vertical offset from the origin to the area of interest (in pixels).",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "region_selector",
                             "Selects Region in Multiple ROI which parameters are set by width, "
                             "height, region mode ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "region_mode",
                             "Activates/deactivates Region selected by Region Selector ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "exp_priority",
                             "Exposure priority (0.8 - exposure 80%, gain 20%). ",
                             xiTypeFloat,
                             0.8);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ag_max_limit",
                             "Maximum limit of gain in AEAG procedure ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ae_max_limit",
                             "Maximum time (us) used for exposure in AEAG procedure ",
                             xiTypeInteger,
                             100000);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "aeag_level",
                             "Average intensity of output signal AEAG should achieve(in %) ",
                             xiTypeFloat,
                             0.4);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "limit_bandwidth",
                             "Set/get bandwidth(datarate)(in Megabits) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sensor_bit_depth",
                             "Sensor output data bit depth. XI_BIT_DEPTH",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "output_bit_depth",
                             "Device output data bit depth. XI_BIT_DEPTH",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "image_data_bit_depth",
                             "bitdepth of data returned by function xiGetImage XI_BIT_DEPTH",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "output_bit_packing",
                             "Device output data packing (or grouping) enabled. Packing could be "
                             "enabled if output_data_bit_depth > 8 and packing capability is "
                             "available. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "output_bit_packing_type",
                             "Data packing type. Some cameras supports only specific packing "
                             "type. XI_OUTPUT_DATA_PACKING_TYPE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "iscooled",
                             "Returns 1 for cameras that support cooling. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "cooling",
                             "Start camera cooling. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "target_temp",
                             "Set sensor target temperature for cooling. ",
                             xiTypeFloat,
                             20.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "chip_temp",
                             "Camera sensor temperature ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hous_temp",
                             "Camera housing tepmerature ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hous_back_side_temp",
                             "Camera housing back side tepmerature ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "cms",
                             "Mode of color management system. XI_CMS_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "apply_cms",
                             "Enable applying of CMS profiles to xiGetImage (see "
                             "XI_PRM_INPUT_CMS_PROFILE, XI_PRM_OUTPUT_CMS_PROFILE). ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "input_cms_profile",
                             "Filename for input cms profile (e.g. input.icc) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "output_cms_profile",
                             "Filename for output cms profile (e.g. input.icc) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "iscolor",
                             "Returns 1 for color cameras. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "cfa",
                             "Returns color filter array type of RAW data. XI_COLOR_FILTER_ARRAY",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gammaY",
                             "Luminosity gamma ",
                             xiTypeFloat,
                             0.47);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gammaC",
                             "Chromaticity gamma ",
                             xiTypeFloat,
                             0.8);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sharpness",
                             "Sharpness Strenght ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX00",
                             "Color Correction Matrix element [0][0] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX01",
                             "Color Correction Matrix element [0][1] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX02",
                             "Color Correction Matrix element [0][2] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX03",
                             "Color Correction Matrix element [0][3] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX10",
                             "Color Correction Matrix element [1][0] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX11",
                             "Color Correction Matrix element [1][1] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX12",
                             "Color Correction Matrix element [1][2] \n",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX13",
                             "Color Correction Matrix element [1][3] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX20",
                             "Color Correction Matrix element [2][0] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX21",
                             "Color Correction Matrix element [2][1] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX22",
                             "Color Correction Matrix element [2][2] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX23",
                             "Color Correction Matrix element [2][3] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX30",
                             "Color Correction Matrix element [3][0] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX31",
                             "Color Correction Matrix element [3][1] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX32",
                             "Color Correction Matrix element [3][2] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ccMTX33",
                             "Color Correction Matrix element [3][3] ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "defccMTX",
                             "Set default Color Correction Matrix ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "trigger_source",
                             "Defines source of trigger. XI_TRG_SOURCE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "trigger_software",
                             "Generates an internal trigger. XI_PRM_TRG_SOURCE must be set to "
                             "TRG_SOFTWARE. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "trigger_selector",
                             "Selects the type of trigger. XI_TRG_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "acq_frame_burst_count",
                             "Sets number of frames acquired by burst. This burst is used only if "
                             "trigger is set to FrameBurstStart ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gpi_selector",
                             "Selects GPI XI_GPI_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gpi_mode",
                             "Defines GPI functionality XI_GPI_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gpi_level",
                             "GPI level ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gpo_selector",
                             "Selects GPO XI_GPO_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "gpo_mode",
                             "Defines GPO functionality XI_GPO_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "led_selector",
                             "Selects LED XI_LED_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "led_mode",
                             "Defines LED functionality XI_LED_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "dbnc_en",
                             "Enable/Disable debounce to selected GPI ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "dbnc_t0",
                             "Debounce time (x * 10us) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "dbnc_t1",
                             "Debounce time (x * 10us) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "dbnc_pol",
                             "Debounce polarity (pol = 1 t0 - falling edge, t1 - rising edge) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_mode",
                             "Status of lens control interface. This shall be set to XI_ON before "
                             "any Lens operations. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_aperture_value",
                             "Current lens aperture value in stops. Examples: 2.8, 4, 5.6, 8, 11",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_focus_move",
                             "Moves lens focus motor by steps set in "
                             "XI_PRM_LENS_FOCUS_MOVEMENT_VALUE. ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_focus_distance",
                             "Lens focus distance in cm. ",
                             xiTypeFloat,
                             1000.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_focal_length",
                             "Lens focal distance in mm. ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_feature_selector",
                             "Selects the current feature which is accessible by "
                             "XI_PRM_LENS_FEATURE. XI_LENS_FEATURE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "lens_feature",
                             "Allows access to lens feature value currently selected by "
                             "XI_PRM_LENS_FEATURE_SELECTOR. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,

                             "imgdataformatrgb32alpha",
                             "The alpha channel of RGB32 output image format. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "imgpayloadsize",
                             "Buffer size in bytes sufficient for output image returned "
                             "by xiGetImage ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "transport_pixel_format",
                             "Current format of pixels on transport layer. XI_GenTL_Image_Format_e",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sensor_clock_freq_hz",
                             "Sensor clock frequency in Hz. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sensor_clock_freq_index",
                             "Sensor clock frequency index. Sensor with selected frequencies "
                             "have possibility to set the frequency only by this index. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "framerate",
                             "Define framerate in Hz ",
                             xiTypeFloat,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "counter_selector",
                             "Select counter XI_COUNTER_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "counter_value",
                             "Counter status ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "acq_timing_mode",
                             "Type of sensor frames timing. XI_ACQ_TIMING_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "available_bandwidth",
                             "Calculate and return available interface bandwidth(int Megabits) ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "buffer_policy",
                             "Data move policy XI_BP",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "LUTEnable",
                             "Activates LUT. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "LUTIndex",
                             "Control the index (offset) of the coefficient to access in the LUT. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "LUTValue",
                             "Value at entry LUTIndex of the LUT ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "trigger_delay",
                             "Specifies the delay in microseconds (us) to apply after the trigger "
                             "reception before activating it. XI_GPI_SELECTOR",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ts_rst_mode",
                             "Defines how time stamp reset engine will be armed XI_TS_RST_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "ts_rst_source",
                             "Defines which source will be used for timestamp reset. Writing this "
                             "parameter will trigger settings of engine (arming) XI_TS_RST_SOURCE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "isexist",
                             "Returns 1 if camera connected and works properly. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "acq_buffer_size",
                             "Acquisition buffer size in buffer_size_unit. Default bytes. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "acq_buffer_size_unit",
                             "Acquisition buffer size unit in bytes. Default 1. E.g. Value 1024 "
                             "means that buffer_size is in KiBytes ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "buffers_queue_size",
                             "Queue of field/frame buffers ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "recent_frame",
                             "GetImage returns most recent frame ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "device_reset",
                             "Resets the camera to default state. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "column_fpn_correction",
                             "Correction of column FPN XI_SWITCH",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "sensor_mode",
                             "Current sensor mode. Allows to select sensor mode by one integer. "
                             "Setting of this parameter affects: image dimensions and downsampling."
                             "XI_SENSOR_MODE",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,

                             "hdr",
                             "Enable High Dynamic Range feature. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,

                             "hdr_kneepoint_count",
                             "The number of kneepoints in the PWLR. ",
                             xiTypeInteger,
                             0.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hdr_t1",
                             "position of first kneepoint(in % of XI_PRM_EXPOSURE) ",
                             xiTypeInteger,
                             60.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hdr_t2",
                             "position of second kneepoint (in % of XI_PRM_EXPOSURE) ",
                             xiTypeInteger,
                             30.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hdr_kneepoint1",
                             "value of first kneepoint (% of sensor saturation) ",
                             xiTypeInteger,
                             40.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "hdr_kneepoint2",
                             "value of second kneepoint (% of sensor saturation) ",
                             xiTypeInteger,
                             60.0);
    XIMEA_PARAM_MAP_POPULATE(m,
                             "image_black_level",
                             "Last image black level counts. Can be used for Offline processing "
                             "to recall it. ",
                             xiTypeInteger,
                             0.0);

    return m;
}
*/
#endif  // INCLUDE_XIMEA_CAMERA_XIMEA_PARAM_H_
