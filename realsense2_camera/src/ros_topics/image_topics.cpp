
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

#include "../include/base_realsense_node.h"
#include "assert.h"
#include <algorithm>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <fstream>
#include <image_publisher.h>

// Header files for disabling intra-process comms for static broadcaster.
#include <rclcpp/publisher_options.hpp>
#include <tf2_ros/qos.hpp>

using namespace realsense2_camera;

bool BaseRealSenseNode::fillROSImageMsgAndReturnStatus(
    const cv::Mat& cv_matrix_image,
    const stream_index_pair& stream,
    unsigned int width,
    unsigned int height,
    const rs2_format& stream_format,
    const rclcpp::Time& t,
    sensor_msgs::msg::Image* img_msg_ptr)
{
    if (cv_matrix_image.empty())
    {
        ROS_ERROR_STREAM("cv::Mat is empty. Ignoring this frame.");
        return false;
    }
    else if (_rs_format_to_ros_format.find(stream_format) == _rs_format_to_ros_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in ROS2 image messages"
                                   << "Please try different format of this stream.");
        return false;
    }
    // Convert the CV::Mat into a ROS image message (1 copy is done here)
    cv_bridge::CvImage(std_msgs::msg::Header(), _rs_format_to_ros_format[stream_format], cv_matrix_image).toImageMsg(*img_msg_ptr);

    // Convert OpenCV Mat to ROS Image
    img_msg_ptr->header.frame_id = OPTICAL_FRAME_ID(stream);
    img_msg_ptr->header.stamp = t;
    img_msg_ptr->height = height;
    img_msg_ptr->width = width;
    img_msg_ptr->is_bigendian = false;
    img_msg_ptr->step = width * cv_matrix_image.elemSize();
    return true;
}

bool BaseRealSenseNode::fillCVMatImageAndReturnStatus(
    rs2::frame& frame,
    std::map<stream_index_pair, cv::Mat>& images,
    unsigned int width,
    unsigned int height,
    const stream_index_pair& stream)
{
    auto& image = images[stream];
    auto stream_format = frame.get_profile().format();

    if (_rs_format_to_cv_format.find(stream_format) == _rs_format_to_cv_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in realsense2_camera node."
                                   << "\nPlease try different format of this stream.");
        return false;
    }
    // we try to reduce image creation as much we can, so we check if the same image structure
    // was already created before, and we fill this image next with the frame data
    // image.create() should be called once per <stream>_<profile>_<format>
    if (image.size() != cv::Size(width, height) || CV_MAKETYPE(image.depth(), image.channels()) != _rs_format_to_cv_format[stream_format])
    {
        image.create(height, width, _rs_format_to_cv_format[stream_format]);
    }

    image.data = (uint8_t*)frame.get_data();

    if (frame.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, _depth_scaled_image[stream]);
    }

    return true;
}

void BaseRealSenseNode::publishFrame(
    rs2::frame f,
    const rclcpp::Time& t,
    const stream_index_pair& stream,
    std::map<stream_index_pair, cv::Mat>& images,
    const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
    const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
    const bool is_publishMetadata)
{
    ROS_DEBUG("publishFrame(...)");
    unsigned int width = 0;
    unsigned int height = 0;
    auto stream_format = RS2_FORMAT_ANY;
    if (f.is<rs2::video_frame>())
    {
        auto timage = f.as<rs2::video_frame>();
        width = timage.get_width();
        height = timage.get_height();
        stream_format = timage.get_profile().format();
    }
    else
    {
        ROS_ERROR("f.is<rs2::video_frame>() check failed. Frame was dropped.");
        return;
    }

    // Publish stream image
    if (image_publishers.find(stream) != image_publishers.end())
    {
        auto &image_publisher = image_publishers.at(stream);
        cv::Mat image_cv_matrix;

        // if rgbd has subscribers we fetch the CV image here
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            if (fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }
        }

        // if depth/color has subscribers, ask first if rgbd already fetched
        // the images from the frame. if not, fetch the relevant color/depth image.
        if (0 != image_publisher->get_subscription_count())
        {
            if (image_cv_matrix.empty() && fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }

            // Prepare image topic to be published
            // We use UniquePtr for allow intra-process publish when subscribers of that type are available
            sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
            if (!img_msg_ptr)
            {
                ROS_ERROR("Sensor image message allocation failed. Frame was dropped.");
                return;
            }

            if (fillROSImageMsgAndReturnStatus(image_cv_matrix, stream, width, height, stream_format, t, img_msg_ptr.get()))
            {

                // Transfer the unique pointer ownership to the RMW
                sensor_msgs::msg::Image *msg_address = img_msg_ptr.get();
                image_publisher->publish(std::move(img_msg_ptr));

                ROS_DEBUG_STREAM(rs2_stream_to_string(f.get_profile().stream_type()) << " stream published, message address: " << std::hex << msg_address);
            }
            else
            {
                ROS_ERROR("Could not fill ROS message. Frame was dropped.");
            }
        }
    }

    // Publish stream camera info
    if(info_publishers.find(stream) != info_publishers.end())
    {
        auto& info_publisher = info_publishers.at(stream);

        // If rgbd has subscribers, get the camera info of color/detph sensors from _camera_info map.
        // We need this camera info to fill the rgbd msg, regardless if there subscribers to depth/color camera info.
        // We are not publishing this cam_info here, but will be published by rgbd publisher.
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
        }

        // If depth/color camera info has subscribers get camera info from _camera_info map,
        // and publish this msg.
        if(0 != info_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
            info_publisher->publish(cam_info);
        }
    }

    // Publish stream metadata
    if (is_publishMetadata)
    {
        publishMetadata(f, t, OPTICAL_FRAME_ID(stream));
    }
}
