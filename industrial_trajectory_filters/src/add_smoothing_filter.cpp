/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Southwest Research Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Southwest Research Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Chris Lewis */

#include <moveit/planning_interface/planning_request_adapter.h>
#include <moveit/version.h>
#include <class_loader/class_loader.hpp>

#include <industrial_trajectory_filters/smoothing_trajectory_filter.h>
#include <rclcpp/rclcpp.hpp>

namespace industrial_trajectory_filters
{

class AddSmoothingFilter : public planning_request_adapter::PlanningRequestAdapter
{
public:

  static const std::string FILTER_PARAMETER_NAME_; // base name for filter parameters

  /*!  \brief Constructor AddSmoothingFilter is a planning request adapter plugin which post-processes
  *             The robot's trajectory to round out corners
  *             The filter name and then its coefficients are obtained from the ROS parameter server
  */
  AddSmoothingFilter() : planning_request_adapter::PlanningRequestAdapter()
  {
    nh_ = rclcpp::Node::make_shared("add_smoothing_filter");
    //in noetic and newer initialize will be called by external code
#if MOVEIT_VERSION_MAJOR < 1 || (MOVEIT_VERSION_MAJOR == 1 && MOVEIT_VERSION_MINOR < 1)
    initialize(nh_);
#endif
  }

  /**
  * \brief load parameters from a certain namespace
  */
#if MOVEIT_VERSION_MAJOR >= 1 && MOVEIT_VERSION_MINOR >= 1
  virtual void initialize(const rclcpp::Node::SharedPtr& node_handle) override
#else
  virtual void initialize(const rclcpp::Node::SharedPtr& node_handle)
#endif
  {
      nh_ = node_handle;

      // set of default filter coefficients in case paramter, or its associated file, or its syntax is wrong
      filter_coef_.push_back(0.25);
      filter_coef_.push_back(0.5);
      filter_coef_.push_back(1.0);
      filter_coef_.push_back(0.5);
      filter_coef_.push_back(0.25);

    // see if a new set of filter parameters is on the parameter server, if so, install them
    try {
      nh_->declare_parameter(FILTER_PARAMETER_NAME_, std::string(""));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

    std::string param_val;
    if (!nh_->get_parameter(FILTER_PARAMETER_NAME_, param_val) || param_val.empty()) {
      RCLCPP_INFO(nh_->get_logger(), "Param '%s' was not set. Using default filter values", FILTER_PARAMETER_NAME_.c_str());
    }
    else {
      filter_name_ = param_val;
      std::vector<double> temp_coef;
      try {
        nh_->declare_parameter(filter_name_, std::vector<double>{});
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
      nh_->get_parameter(filter_name_, temp_coef);
      if (temp_coef.size() % 2 == 1 && temp_coef.size() > 2) {
        filter_coef_.clear();
        for (int i = 0; i < (int)temp_coef.size(); i++) {
          filter_coef_.push_back(temp_coef[i]);
        }
      }
      else {
        RCLCPP_INFO(nh_->get_logger(), "Could not read filter, using default filter coefficients");
      }
    }
    if (!smoothing_filter_.init(filter_coef_))
      RCLCPP_ERROR(nh_->get_logger(), "Initialization error on smoothing filter. Requires an odd number of coeficients");
  };

  /*!  \brief Destructor */
  ~AddSmoothingFilter(){ };

  /*!  \brief Returns a short description of this plugin */
  virtual std::string getDescription() const { return "Add Smoothing Trajectory Filter"; }

  /*!  \brief The work hourse of planning request adapters
   *   \param planner A function called somewhere within this subroutine
   *   \param planning_scene  an object describing the objects and kinematics
   *   \param req   the request, includes the starting config, the goal, constraints, etc
   *   \param res    the response, includes the robot trajectory and other info
   *   \param added_path_index, a index of the points added by this adapter, which in this case will be empty
   */
  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    bool result = planner(planning_scene, req, res);

    if (result && res.trajectory_)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Running '%s'", getDescription().c_str());
      if (!smoothing_filter_.applyFilter(*res.trajectory_)) {
        RCLCPP_ERROR(nh_->get_logger(), "Smoothing filter of the solution path failed. Filter Not Initialized");
      }
    }
    return result;
  };


private:
  rclcpp::Node::SharedPtr nh_;
  industrial_trajectory_filters::SmoothingTrajectoryFilter smoothing_filter_;
  std::string filter_name_;
  std::vector<double> filter_coef_;

};

const std::string  AddSmoothingFilter::FILTER_PARAMETER_NAME_ = "smoothing_filter_name";

}

CLASS_LOADER_REGISTER_CLASS(industrial_trajectory_filters::AddSmoothingFilter,
                            planning_request_adapter::PlanningRequestAdapter);
