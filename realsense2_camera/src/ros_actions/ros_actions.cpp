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


void BaseRealSenseNode::publishActions()
{

    using namespace std::placeholders;
    _triggered_calibration_action_server = rclcpp_action::create_server<TriggeredCalibration>(
      _node.get_node_base_interface(),
      _node.get_node_clock_interface(),
      _node.get_node_logging_interface(),
      _node.get_node_waitables_interface(),
      "~/triggered_calibration",
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleGoal, this, _1, _2),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleCancel, this, _1),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleAccepted, this, _1));
}

