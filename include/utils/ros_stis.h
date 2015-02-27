/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*

ros_stis is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ros_stis is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ros_stis. If not, see <http://www.gnu.org/licenses/>.
*/  

#pragma once

/// set of explicit conversion functions between ros and st_is

#include "geometry_msgs/Pose.h"
#include "utils/geometry_pose.h"

namespace st_is
{

geometry_msgs::Pose stisToROS( st_is::GeometryPose _pose );
st_is::GeometryPose ROSToStis( geometry_msgs::Pose _pose );
std::vector<geometry_msgs::Pose> stisToROS( std::vector<GeometryPose> _to_convert );
std::vector<GeometryPose> ROSToStis( std::vector<geometry_msgs::Pose> _to_convert );

}