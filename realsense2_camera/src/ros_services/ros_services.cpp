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


void BaseRealSenseNode::publishServices()
{
    // adding "~/" to the service name will add node namespace and node name to the service
    // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
    _device_info_srv = _node.create_service<realsense2_camera_msgs::srv::DeviceInfo>(
            "~/device_info",
            [&](const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
                        {getDeviceInfo(req, res);});

    _calib_config_read_srv = _node.create_service<realsense2_camera_msgs::srv::CalibConfigRead>(
            "~/calib_config_read",
            [&](const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res)
                        {CalibConfigReadService(req, res);});

    _calib_config_write_srv = _node.create_service<realsense2_camera_msgs::srv::CalibConfigWrite>(
            "~/calib_config_write",
            [&](const realsense2_camera_msgs::srv::CalibConfigWrite::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::CalibConfigWrite::Response::SharedPtr res)
                        {CalibConfigWriteService(req, res);});

}
