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
#include <image_publisher.h>
#include <rclcpp/qos.hpp>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::stopPublishers(const std::vector<stream_profile>& profiles)
{
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (profile.is<rs2::video_stream_profile>())
        {
            _image_publishers.erase(sip);
            _info_publishers.erase(sip);
            _depth_aligned_image_publishers.erase(sip);
            _depth_aligned_info_publisher.erase(sip);
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            _is_accel_enabled = false;
            _is_gyro_enabled = false;
            _synced_imu_publisher.reset();
            _imu_publishers.erase(sip);
            _imu_info_publishers.erase(sip);
        }
        _metadata_publishers.erase(sip);
        _extrinsics_publishers.erase(sip);

        if (_publish_tf)
        {
            eraseTransformMsgs(sip, profile);
        }
    }
}

void BaseRealSenseNode::startPublishers(const std::vector<stream_profile>& profiles, const RosSensor& sensor)
{
    const std::string module_name(create_graph_resource_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME))));
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        std::string stream_name(STREAM_NAME(sip));

        rmw_qos_profile_t qos = sensor.getQOS(sip);
        rmw_qos_profile_t info_qos = sensor.getInfoQOS(sip);

        if (profile.is<rs2::video_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_COLOR)
                _is_color_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_DEPTH)
                _is_depth_enabled = true;
            std::stringstream image_raw, camera_info;
            bool rectified_image = false;
            if (sensor.rs2::sensor::is<rs2::depth_sensor>())
                rectified_image = true;

            // adding "~/" to the topic name will add node namespace and node name to the topic
            // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
            image_raw << "~/" << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << "~/" << stream_name << "/camera_info";

            // We can use 2 types of publishers:
            // Native RCL publisher that support intra-process zero-copy comunication
            // image-transport package publisher that adds a commpressed image topic if package is found installed
            if (_use_intra_process)
            {
                _image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, image_raw.str(), qos);
            }
            else
            {
                _image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, image_raw.str(), qos);
                ROS_DEBUG_STREAM("image transport publisher was created for topic" << image_raw.str());
            }

            _info_publishers[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info.str(),
                                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));

            if (_align_depth_filter->is_enabled() && (sip != DEPTH) && sip.second < 2)
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "~/" << "aligned_depth_to_" << stream_name << "/image_raw";
                aligned_camera_info << "~/" << "aligned_depth_to_" << stream_name << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + stream_name;

                // We can use 2 types of publishers:
                // Native RCL publisher that support intra-process zero-copy comunication
                // image-transport package publisher that add's a commpressed image topic if the package is installed
                if (_use_intra_process)
                {
                    _depth_aligned_image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, aligned_image_raw.str(), qos);
                }
                else
                {
                    _depth_aligned_image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, aligned_image_raw.str(), qos);
                    ROS_DEBUG_STREAM("image transport publisher was created for topic" << image_raw.str());
                }
                _depth_aligned_info_publisher[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(aligned_camera_info.str(),
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));
            }
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_ACCEL)
                _is_accel_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_GYRO)
                _is_gyro_enabled = true;

            std::stringstream data_topic_name, info_topic_name;
            data_topic_name << "~/" << stream_name << "/sample";
            _imu_publishers[sip] = _node.create_publisher<sensor_msgs::msg::Imu>(data_topic_name.str(),
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
            // Publish Intrinsics:
            info_topic_name << "~/" << stream_name << "/imu_info";
            _imu_info_publishers[sip] = _node.create_publisher<IMUInfo>(info_topic_name.str(),
                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));
            IMUInfo info_msg = getImuInfo(profile);
            _imu_info_publishers[sip]->publish(info_msg);
        }
        std::string topic_metadata("~/" + stream_name + "/metadata");
        _metadata_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Metadata>(topic_metadata, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));

        if (!((rs2::stream_profile)profile==(rs2::stream_profile)_base_profile))
        {

            // intra-process do not support latched QoS, so we need to disable intra-process for this topic
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            rmw_qos_profile_t extrinsics_qos = rmw_qos_profile_latched;

            std::string topic_extrinsics("~/extrinsics/" + create_graph_resource_name(ros_stream_to_string(_base_profile.stream_type()) + "_to_" + stream_name));
            _extrinsics_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Extrinsics>(topic_extrinsics,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(extrinsics_qos), extrinsics_qos), std::move(options));
        }
    }
    if (_is_accel_enabled && _is_gyro_enabled && (_imu_sync_method > imu_sync_method::NONE))
    {
        rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(HID_QOS);
        
        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("~/imu", 
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)));
    }

}


