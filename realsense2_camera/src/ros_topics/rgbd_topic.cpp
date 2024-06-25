// Copyright 2024 Intel Corporation. All Rights Reserved.
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

#include "../include/base_realsense_node.h"
#include <rclcpp/qos.hpp>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::startRGBDPublisherIfNeeded()
{
    _rgbd_publisher.reset();
    if(_enable_rgbd && !_rgbd_publisher)
    {
        if (_sync_frames && _is_color_enabled && _is_depth_enabled && _align_depth_filter->is_enabled())
        {
            rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(IMAGE_QOS);

            // adding "~/" to the topic name will add node namespace and node name to the topic
            // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
            _rgbd_publisher = _node.create_publisher<realsense2_camera_msgs::msg::RGBD>("~/rgbd",
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
        }
        else {
            ROS_ERROR("In order to get rgbd topic enabled, "\
             "you should enable: color stream, depth stream, sync_mode and align_depth");
        }
    }
}

void BaseRealSenseNode::publishRGBD(
    const cv::Mat& rgb_cv_matrix,
    const rs2_format& color_format,
    const cv::Mat& depth_cv_matrix,
    const rs2_format& depth_format,
    const rclcpp::Time& t)
{
    if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
    {
        ROS_DEBUG_STREAM("Publishing RGBD message");
        unsigned int rgb_width = rgb_cv_matrix.size().width;
        unsigned int rgb_height = rgb_cv_matrix.size().height;
        unsigned int depth_width = depth_cv_matrix.size().width;
        unsigned int depth_height = depth_cv_matrix.size().height;

        realsense2_camera_msgs::msg::RGBD::UniquePtr msg(new realsense2_camera_msgs::msg::RGBD());

        bool rgb_message_filled = fillROSImageMsgAndReturnStatus(rgb_cv_matrix, COLOR, rgb_width, rgb_height, color_format, t, &msg->rgb);
        if(!rgb_message_filled)
        {
            ROS_ERROR_STREAM("Failed to fill rgb message inside RGBD message");
            return;
        }

        bool depth_messages_filled = fillROSImageMsgAndReturnStatus(depth_cv_matrix, DEPTH, depth_width, depth_height, depth_format, t, &msg->depth);
        if(!depth_messages_filled)
        {
            ROS_ERROR_STREAM("Failed to fill depth message inside RGBD message");
            return;
        }

        msg->header.frame_id = "camera_rgbd_optical_frame";
        msg->header.stamp = t;

        auto rgb_camera_info = _camera_info.at(COLOR);
        msg->rgb_camera_info = rgb_camera_info;

        auto depth_camera_info = _camera_info.at(DEPTH);
        msg->depth_camera_info = depth_camera_info;

        realsense2_camera_msgs::msg::RGBD *msg_address = msg.get();
        _rgbd_publisher->publish(std::move(msg));
        ROS_DEBUG_STREAM("rgbd stream published, message address: " << std::hex << msg_address);
    }
}
