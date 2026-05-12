/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include <kdl/velocityprofile_spline.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace industrial_trajectory_filters;

const double DEFAULT_SAMPLE_DURATION = 0.050; //seconds

template<typename T>
  UniformSampleFilter<T>::UniformSampleFilter() :
      industrial_trajectory_filters::FilterBase<T>()
  {
    RCLCPP_INFO(rclcpp::get_logger("uniform_sample_filter"), "Constructing N point filter");
    sample_duration_ = DEFAULT_SAMPLE_DURATION;
    this->filter_name_ = "UniformSampleFilter";
    this->filter_type_ = "UniformSampleFilter";
  }

template<typename T>
  UniformSampleFilter<T>::~UniformSampleFilter()
  {
  }

template<typename T>
  bool UniformSampleFilter<T>::configure()
  {
    if (!this->nh_ || !this->nh_->get_parameter("sample_duration", sample_duration_))
    {
      RCLCPP_WARN(rclcpp::get_logger("uniform_sample_filter"), "UniformSampleFilter, params has no attribute sample_duration.");
    }
    RCLCPP_INFO(rclcpp::get_logger("uniform_sample_filter"), "Using a sample_duration value of %f", sample_duration_);

    return true;
  }

template<typename T>
  bool UniformSampleFilter<T>::update(const T& trajectory_in, T& trajectory_out)
  {
    bool success = false;
    size_t size_in = trajectory_in.request.trajectory.points.size();
    double duration_in = rclcpp::Duration(trajectory_in.request.trajectory.points.back().time_from_start).seconds();
    double interpolated_time = 0.0;
    size_t index_in = 0;

    trajectory_msgs::msg::JointTrajectoryPoint p1, p2, interp_pt;

    trajectory_out = trajectory_in;

    // Clear out the trajectory points
    trajectory_out.request.trajectory.points.clear();

    while (interpolated_time < duration_in)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "Interpolated time: %f", interpolated_time);
      // Increment index until the interpolated time is past the start time.
      while (interpolated_time > rclcpp::Duration(trajectory_in.request.trajectory.points[index_in + 1].time_from_start).seconds())
      {
        RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"),
            "Interpolated time: %f, next point time: %f", interpolated_time,
            rclcpp::Duration(trajectory_in.request.trajectory.points[index_in + 1].time_from_start).seconds());
        RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "Incrementing index");
        index_in++;
        if (index_in >= size_in)
        {
          RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"),
              "Programming error, index: %zu, greater(or equal) to size: %zu input duration: %f interpolated time: %f",
              index_in, size_in, duration_in, interpolated_time);
          return false;
        }
      }
      p1 = trajectory_in.request.trajectory.points[index_in];
      p2 = trajectory_in.request.trajectory.points[index_in + 1];
      if (!interpolatePt(p1, p2, interpolated_time, interp_pt))
      {
        RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"), "Failed to interpolate point");
        return false;
      }
      trajectory_out.request.trajectory.points.push_back(interp_pt);
      interpolated_time += sample_duration_;

    }

    RCLCPP_INFO(rclcpp::get_logger("uniform_sample_filter"),
        "Interpolated time exceeds original trajectory (quitting), original: %f final interpolated time: %f",
        duration_in, interpolated_time);
    p2 = trajectory_in.request.trajectory.points.back();
    p2.time_from_start = rclcpp::Duration::from_seconds(interpolated_time);
    // TODO: Really should check that appending the last point doesn't result in
    // really slow motion at the end.  This could happen if the sample duration is a
    // large percentage of the trajectory duration (not likely).
    trajectory_out.request.trajectory.points.push_back(p2);

    RCLCPP_INFO(rclcpp::get_logger("uniform_sample_filter"),
        "Uniform sampling, resample duraction: %f input traj. size: %zu output traj. size: %zu",
        sample_duration_,
        trajectory_in.request.trajectory.points.size(),
        trajectory_out.request.trajectory.points.size());

    success = true;
    return success;
  }

template<typename T>
  bool UniformSampleFilter<T>::interpolatePt(trajectory_msgs::msg::JointTrajectoryPoint & p1,
                                             trajectory_msgs::msg::JointTrajectoryPoint & p2, double time_from_start,
                                             trajectory_msgs::msg::JointTrajectoryPoint & interp_pt)
  {
    bool rtn = false;
    double p1_time_from_start = rclcpp::Duration(p1.time_from_start).seconds();
    double p2_time_from_start = rclcpp::Duration(p2.time_from_start).seconds();

    RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "time from start: %f", time_from_start);

    if (time_from_start >= p1_time_from_start && time_from_start <= p2_time_from_start)
    {
      if (p1.positions.size() == p1.velocities.size() && p1.positions.size() == p1.accelerations.size())
      {
        if (p1.positions.size() == p2.positions.size() && p1.velocities.size() == p2.velocities.size()
            && p1.accelerations.size() == p2.accelerations.size())
        {
          // Copy p1 to ensure the interp_pt has the correct size vectors
          interp_pt = p1;
          // TODO: Creating a new spline calculator in this function means that
          // it may be created multiple times for the same points (assuming the
          // resample duration is less that the actual duration, which it might
          // be sometimes)
          KDL::VelocityProfile_Spline spline_calc;
          RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "---------------Begin interpolating joint point---------------");

          for (size_t i = 0; i < p1.positions.size(); ++i)
          {
            // Calculated relative times for spline calculation
            double time_from_p1 = time_from_start - rclcpp::Duration(p1.time_from_start).seconds();
            double time_from_p1_to_p2 = p2_time_from_start - p1_time_from_start;

            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "time from p1: %f", time_from_p1);
            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "time_from_p1_to_p2: %f", time_from_p1_to_p2);

            spline_calc.SetProfileDuration(p1.positions[i], p1.velocities[i], p1.accelerations[i], p2.positions[i],
                                           p2.velocities[i], p2.accelerations[i], time_from_p1_to_p2);

            interp_pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "time from start: %f", time_from_start);

            interp_pt.positions[i] = spline_calc.Pos(time_from_p1);
            interp_pt.velocities[i] = spline_calc.Vel(time_from_p1);
            interp_pt.accelerations[i] = spline_calc.Acc(time_from_p1);

            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"),
                "p1.pos: %f, vel: %f, acc: %f", p1.positions[i], p1.velocities[i], p1.accelerations[i]);

            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"),
                "p2.pos: %f, vel: %f, acc: %f", p2.positions[i], p2.velocities[i], p2.accelerations[i]);

            RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"),
                "interp_pt.pos: %f, vel: %f, acc: %f", interp_pt.positions[i], interp_pt.velocities[i], interp_pt.accelerations[i]);
          }
          RCLCPP_DEBUG(rclcpp::get_logger("uniform_sample_filter"), "---------------End interpolating joint point---------------");
          rtn = true;
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"), "Trajectory point size mismatch");
          RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"),
              "Trajectory point 1, pos: %zu vel: %zu acc: %zu",
              p1.positions.size(), p1.velocities.size(), p1.accelerations.size());
          RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"),
              "Trajectory point 2, pos: %zu vel: %zu acc: %zu",
              p2.positions.size(), p2.velocities.size(), p2.accelerations.size());
          rtn = false;
        }

      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"),
            "Trajectory point not fully defined, pos: %zu vel: %zu acc: %zu",
            p1.positions.size(), p1.velocities.size(), p1.accelerations.size());
        rtn = false;
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("uniform_sample_filter"),
          "Time: %f not between interpolation point times[%f,%f]",
          time_from_start, p1_time_from_start, p2_time_from_start);
      rtn = false;
    }

    return rtn;
  }

// registering planner adapter
CLASS_LOADER_REGISTER_CLASS( industrial_trajectory_filters::UniformSampleFilterAdapter,
                            planning_request_adapter::PlanningRequestAdapter);

/*
 * Old plugin declaration for arm navigation trajectory filters
 PLUGINLIB_DECLARE_CLASS(industrial_trajectory_filters,
 IndustrialNPointFilterJointTrajectoryWithConstraints,
 industrial_trajectory_filters::NPointFilter<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
 filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>);

 */

