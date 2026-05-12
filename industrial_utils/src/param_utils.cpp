/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>

#include "industrial_utils/param_utils.h"
#include "industrial_utils/utils.h"
#include <rclcpp/rclcpp.hpp>
#include <urdf/urdfdom_compatibility.h>

namespace industrial_utils
{
namespace param
{
bool getListParam(rclcpp::Node::SharedPtr node, const std::string param_name, std::vector<std::string> & list_param)
{
  bool rtn = false;

  list_param.clear(); //clear out return value

  try
  {
    node->declare_parameter(param_name, rclcpp::PARAMETER_STRING_ARRAY);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
  {
  }

  rclcpp::Parameter param;
  if (node->get_parameter(param_name, param))
  {
    try
    {
      list_param = param.as_string_array();
      for (const auto & s : list_param)
      {
        RCLCPP_INFO(rclcpp::get_logger("industrial_utils"), "Adding %s to list parameter", s.c_str());
      }
      rtn = true;
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("industrial_utils"), "Parameter: %s not of string array type", param_name.c_str());
      rtn = false;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("industrial_utils"), "Failed to get parameter: %s", param_name.c_str());
    rtn = false;
  }

  return rtn;
}

std::string vec2str(const std::vector<std::string> &vec)
{
  std::string s, delim = ", ";
  std::stringstream ss;
  std::copy(vec.begin(), vec.end(), std::ostream_iterator<std::string>(ss, delim.c_str()));
  s = ss.str();
  return "[" + s.erase(s.length()-2) + "]";
}

bool getJointNames(rclcpp::Node::SharedPtr node, const std::string joint_list_param, const std::string urdf_param,
		           std::vector<std::string> & joint_names)
{
  joint_names.clear();

  // 1) Try to read explicit list of joint names
  if (node->has_parameter(joint_list_param) && getListParam(node, joint_list_param, joint_names))
  {
    RCLCPP_INFO(rclcpp::get_logger("industrial_utils"), "Found user-specified joint names in '%s': %s",
      joint_list_param.c_str(), vec2str(joint_names).c_str());
    return true;
  }
  else
    RCLCPP_WARN(rclcpp::get_logger("industrial_utils"), "Unable to find user-specified joint names in '%s'",
      joint_list_param.c_str());

  // 2) Try to find joint names from URDF model
  urdf::Model model;
  if ( node->has_parameter(urdf_param)
       && model.initString(node->get_parameter(urdf_param).as_string())
       && findChainJointNames(model.getRoot(), true, joint_names) )
  {
    RCLCPP_INFO(rclcpp::get_logger("industrial_utils"), "Using joint names from URDF: '%s': %s",
      urdf_param.c_str(), vec2str(joint_names).c_str());
    return true;
  }
  else
    RCLCPP_WARN(rclcpp::get_logger("industrial_utils"), "Unable to find URDF joint names in '%s'",
      urdf_param.c_str());

  // 3) Raise an error
  RCLCPP_ERROR(rclcpp::get_logger("industrial_utils"),
      "Cannot find user-specified joint names. Tried ROS parameter '%s' and the URDF in '%s'.",
      joint_list_param.c_str(), urdf_param.c_str());
  return false;
}

bool getJointVelocityLimits(rclcpp::Node::SharedPtr node, const std::string urdf_param_name, std::map<std::string, double> &velocity_limits)
{
  urdf::Model model;
  std::map<std::string, urdf::JointSharedPtr >::iterator iter;

  if (!node->has_parameter(urdf_param_name) || !model.initString(node->get_parameter(urdf_param_name).as_string()))
    return false;

  velocity_limits.clear();
  for (iter=model.joints_.begin(); iter!=model.joints_.end(); ++iter)
  {
    std::string joint_name(iter->first);
    urdf::JointLimitsSharedPtr limits = iter->second->limits;
    if ( limits && (limits->velocity > 0) )
      velocity_limits.insert(std::pair<std::string,double>(joint_name,limits->velocity));
  }

  return true;
}

} //industrial_utils::param
} //industrial_utils
