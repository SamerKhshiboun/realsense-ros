// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../../include/base_realsense_node.h"

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::getDeviceInfo(const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr,
                                            realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
{
    res->device_name = _dev.supports(RS2_CAMERA_INFO_NAME) ? create_graph_resource_name(_dev.get_info(RS2_CAMERA_INFO_NAME)) : "";
    res->serial_number = _dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
    res->firmware_version = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) : "";
    res->usb_type_descriptor = _dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) ? _dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "";
    res->firmware_update_id = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) : "";

    std::stringstream sensors_names;

    for(auto&& sensor : _available_ros_sensors)
    {
        sensors_names << create_graph_resource_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME))) << ",";
    }

    res->sensors = sensors_names.str().substr(0, sensors_names.str().size()-1);
    res->physical_port = _dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT) ? _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) : "";
}
