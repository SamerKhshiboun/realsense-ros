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

void BaseRealSenseNode::CalibConfigReadService(const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res){
    try
    {
        (void)req; // silence unused parameter warning
        rs2_calibration_config calib_config = _dev.as<rs2::auto_calibrated_device>().get_calibration_config();
        res->calib_config = _dev.as<rs2::auto_calibrated_device>().calibration_config_to_json_string(calib_config);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

