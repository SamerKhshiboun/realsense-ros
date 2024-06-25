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
#include <image_publisher.h>
#include <fstream>
#include <rclcpp/qos.hpp>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setup()
{
#if defined (ACCELERATE_GPU_WITH_GLSL)
    initOpenGLProcessing(_accelerate_gpu_with_glsl);
    _is_accelerate_gpu_with_glsl_changed = false;
#endif
    setDynamicParams();
    startDiagnosticsUpdater();
    setAvailableSensors();
    SetBaseStream();
    setupFilters();
    setCallbackFunctions();
    monitoringProfileChanges();
    updateSensors();
    publishServices();
    publishActions();
}

void BaseRealSenseNode::monitoringProfileChanges()
{
    int time_interval(10000);
    std::function<void()> func = [this, time_interval](){
        std::unique_lock<std::mutex> lock(_profile_changes_mutex);
        while(_is_running) {
            _cv_mpc.wait_for(lock, std::chrono::milliseconds(time_interval),
                                               [&]{return (!_is_running || _is_profile_changed
                                                                        || _is_align_depth_changed
                                                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                            || _is_accelerate_gpu_with_glsl_changed
                                                                        #endif
                                                           );});

            if (_is_running && (_is_profile_changed
                                        || _is_align_depth_changed
                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                            || _is_accelerate_gpu_with_glsl_changed
                                        #endif
                                ))
            {
                ROS_DEBUG("Profile has changed");
                try
                {
                    updateSensors();
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Error updating the sensors: " << e.what());
                }
                _is_profile_changed = false;
                _is_align_depth_changed = false;

                #if defined (ACCELERATE_GPU_WITH_GLSL)
                    _is_accelerate_gpu_with_glsl_changed = false;
                #endif
            }
        }
    };
    _monitoring_pc = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::setAvailableSensors()
{
    if (!_json_file_path.empty())
    {
        if (_dev.is<rs400::advanced_mode>())
        {
            std::stringstream ss;
            std::ifstream in(_json_file_path);
            if (in.is_open())
            {
                ss << in.rdbuf();
                std::string json_file_content = ss.str();

                auto adv = _dev.as<rs400::advanced_mode>();
                adv.load_json(json_file_content);
                ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
            }
            else
                ROS_WARN_STREAM("JSON file provided doesn't exist! (" << _json_file_path << ")");
        }
        else
            ROS_WARN("Device does not support advanced settings!");
    }
    else
        ROS_INFO("JSON file is not provided");

    auto device_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
    ROS_INFO_STREAM("Device Name: " << device_name);

    auto serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    ROS_INFO_STREAM("Device Serial No: " << serial_no);

    auto device_port_id = _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    ROS_INFO_STREAM("Device physical port: " << device_port_id);

    auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    ROS_INFO_STREAM("Device FW version: " << fw_ver);

    auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    ROS_INFO_STREAM("Device Product ID: 0x" << pid);

    ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

    std::function<void(rs2::frame)> frame_callback_function = [this](rs2::frame frame){
        bool is_filter(_filters.end() != find_if(_filters.begin(), _filters.end(), [](std::shared_ptr<NamedFilter> f){return (f->is_enabled()); }));
        if (_sync_frames || is_filter)
            this->_asyncer.invoke(frame);
        else
            frame_callback(frame);
    };

    std::function<void(rs2::frame)> imu_callback_function = [this](rs2::frame frame){
        imu_callback(frame);
        if (_imu_sync_method != imu_sync_method::NONE)
            imu_callback_sync(frame);
    };

    std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};

    std::function<void()> update_sensor_func = [this](){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_profile_changed = true;
        }
        _cv_mpc.notify_one();
    };

    std::function<void()> hardware_reset_func = [this](){hardwareResetRequest();};

    _dev_sensors = _dev.query_sensors();

    for(auto&& sensor : _dev_sensors)
    {
        const std::string module_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME)));
        std::unique_ptr<RosSensor> rosSensor;
        if (sensor.is<rs2::depth_sensor>() ||
            sensor.is<rs2::color_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as VideoSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, frame_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, _use_intra_process, _dev.is<playback>());
        }
        else if (sensor.is<rs2::motion_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as ImuSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, imu_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, false, _dev.is<playback>());
        }
        else
        {
            ROS_WARN_STREAM("Module Name \"" << module_name << "\" does not define a callback.");
            continue;
        }
        _available_ros_sensors.push_back(std::move(rosSensor));
    }

}

void BaseRealSenseNode::setCallbackFunctions()
{
    _asyncer.start([this](rs2::frame f)
    {
        frame_callback(f);
    });
}

void BaseRealSenseNode::updateSensors()
{
    std::lock_guard<std::mutex> lock_guard(_update_sensor_mutex);
    try{
        stopRequiredSensors();

        #if defined (ACCELERATE_GPU_WITH_GLSL)
            if (_is_accelerate_gpu_with_glsl_changed)
            {
                shutdownOpenGLProcessing();

                initOpenGLProcessing(_accelerate_gpu_with_glsl);
            }
        #endif

        startUpdatedSensors();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::stopRequiredSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            // do all updates if profile has been changed, or if the align depth filter or gpu acceleration status has been changed
            // and we are on a video sensor. TODO: explore better options to monitor and update changes
            // without resetting the whole sensors and topics.
            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                std::vector<stream_profile> active_profiles = sensor->get_active_streams();
                if (is_profile_changed
                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                            || _is_accelerate_gpu_with_glsl_changed
                        #endif
                    )
                {
                    // Start/stop sensors only if profile or gpu acceleration status was changed
                    // No need to start/stop sensors if align_depth was changed
                    ROS_INFO_STREAM("Stopping Sensor: " << module_name);
                    sensor->stop();
                }
                stopPublishers(active_profiles);
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::startUpdatedSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                if (!wanted_profiles.empty())
                {
                    startPublishers(wanted_profiles, *sensor);
                    updateProfilesStreamCalibData(wanted_profiles);
                    if (_publish_tf)
                    {
                        std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
                        for (auto &profile : wanted_profiles)
                        {
                            calcAndAppendTransformMsgs(profile, _base_profile);
                        }
                    }

                    if (is_profile_changed
                            #if defined (ACCELERATE_GPU_WITH_GLSL)
                                || _is_accelerate_gpu_with_glsl_changed
                            #endif
                        )
                    {
                        // Start/stop sensors only if profile or gpu acceleration was changed
                        // No need to start/stop sensors if align_depth was changed
                        ROS_INFO_STREAM("Starting Sensor: " << module_name);
                        sensor->start(wanted_profiles);
                    }

                    if (sensor->rs2::sensor::is<rs2::depth_sensor>())
                    {
                        _depth_scale_meters = sensor->as<rs2::depth_sensor>().get_depth_scale();
                    }
                }
            }
        }
        if (_publish_tf)
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            publishStaticTransforms();
        }
        startRGBDPublisherIfNeeded();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

