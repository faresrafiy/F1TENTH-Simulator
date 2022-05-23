-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Modified by the F1/10 Autonomous Racing Project Group

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30


local ros_version_file = assert(io.popen("echo -n $ROS_DISTRO", "r"))
local ros_version = ros_version_file:read("*all")
ros_version_file:close()
print("[Lua] ROS version: " .. ros_version)

local imu_active_file = assert(io.popen("rostopic info /imu | grep -q -e 'Publishers: None' -e 'ERROR'; echo -n $?", "r"))
local imu_active = (imu_active_file:read("*all") ~= "0")
imu_active_file:close()
print("[Lua] IMU active: " .. tostring(imu_active))

if (ros_version == "kinetic") then
    -- kinetic only parameters
    options.use_laser_scan = true
    options.use_multi_echo_laser_scan = false
elseif (ros_version == "melodic") then
    -- melodic only parameters
    options.publish_frame_projected_to_2d = false
    options.use_nav_sat = false
    options.use_landmarks = false
    options.num_laser_scans = 1
    options.num_multi_echo_laser_scans = 0
    options.num_subdivisions_per_laser_scan = 1
    options.rangefinder_sampling_ratio = 1.
    options.odometry_sampling_ratio = 1.
    options.fixed_frame_pose_sampling_ratio = 1.
    options.imu_sampling_ratio = 1.
    options.landmarks_sampling_ratio = 1.
else
    print("[Lua] ROS version is unsupported (must be kinetic or melodic). Program may crash.")
end

if (imu_active) then
    -- IMU only parameters
    TRAJECTORY_BUILDER_2D.use_imu_data = true
else
    -- no IMU only parameters
    TRAJECTORY_BUILDER_2D.use_imu_data = false
end

return options
