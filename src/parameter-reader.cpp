/*
* Copyright (c) 2018, Vision for Robotics Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/*
 * parameter-reader.cpp
 * @brief Implementation file for the ParameterReader Class
 * @author: Marco Karrer
 * Created on: Aug 14, 2018
 */

#include "parameter-reader.hpp"

namespace pgbe {

ParameterReader::ParameterReader(
    ros::NodeHandle& nh, const size_t num_agents) :
  nh_(&nh), num_agents_(num_agents), read_parameters_(false) {

}

ParameterReader::~ParameterReader() {

}

bool ParameterReader::getParameters(SystemParameters &params) {
  if (read_parameters_) {
    params = parameters_;
    return true;
  } else {
    return readParameters(params);
  }
}

bool ParameterReader::readParameters(SystemParameters &params) {
  CameraParametersVector cam_vector;
  cam_vector.reserve(num_agents_);
  GpsParametersVector gps_parameters;
  gps_parameters.reserve(num_agents_);
  std::vector<bool> gps_active;
  gps_active.reserve(num_agents_);
  bool successful = true;

  bool simulation_ = false;
  if (!nh_->getParam("simulation", simulation_)) {
    successful = false;
  }

  for (size_t i = 0; i < num_agents_; ++i) {
    // Get the config file for the camera
    const std::string param_name_i = "cam_config" + std::to_string(i);
    std::string file_name_i;
    if (!nh_->getParam(param_name_i, file_name_i)) {
      successful = false;
      break;
    }

    // Read the camera configuration file
    CameraParameters cam_params_i(i, file_name_i);
    if (cam_params_i.camera == NULL) {
       successful = false;
       break;
    }
    cam_vector.push_back(cam_params_i);

    // Read the GPS configuration
    const std::string gps_offset_name_i = "gps_offset" + std::to_string(i);
    std::vector<double> gps_offset_i;
    if (!nh_->getParam(gps_offset_name_i, gps_offset_i)) {
      successful = false;
      break;
    }
    const std::string gps_reference_name_i = "gps_reference" +
        std::to_string(i);
    std::vector<double> gps_reference_i;
    if (!nh_->getParam(gps_reference_name_i, gps_reference_i)) {
      successful = false;
      break;
    }
    gps_parameters.push_back(GpsParameters(
        i, Eigen::Vector3d(
                gps_reference_i[0], gps_reference_i[1], gps_reference_i[2]),
        Eigen::Vector3d(gps_offset_i[0], gps_offset_i[1], gps_offset_i[2])));

    const std::string gps_active_name_i = "gps_active_" + std::to_string(i);
    bool gps_active_i = false;
    if (!nh_->getParam(gps_active_name_i, gps_active_i)) {
      successful = false;
    }
    gps_active.push_back(gps_active_i);
  }

  // Read loop detection performance parameters
  double loop_candidate_min_score;
  if (!nh_->getParam("loop_candidate_min_score", loop_candidate_min_score)) {
    successful = false;
  }

  int loop_image_min_matches;
  if (!nh_->getParam("loop_image_min_matches", loop_image_min_matches)) {
    successful = false;
  }

  int loop_detect_sac_thresh;
  if (!nh_->getParam("loop_detect_sac_thresh", loop_detect_sac_thresh)) {
    successful = false;
  }

  int loop_detect_sac_max_iter;
  if (!nh_->getParam("loop_detect_sac_max_iter", loop_detect_sac_max_iter)) {
    successful = false;
  }

  int loop_detect_min_sac_inliers;
  if (!nh_->getParam("loop_detect_min_sac_inliers", loop_detect_min_sac_inliers)) {
    successful = false;
  }

  int loop_detect_min_sac_inv_inliers;
  if (!nh_->getParam("loop_detect_min_sac_inv_inliers", loop_detect_min_sac_inv_inliers)) {
    successful = false;
  }

  int loop_detect_min_pose_inliers;
  if (!nh_->getParam("loop_detect_min_pose_inliers", loop_detect_min_pose_inliers)) {
    successful = false;
  }

  double rel_pose_outlier_norm_min;
  if (!nh_->getParam("rel_pose_outlier_norm_min", rel_pose_outlier_norm_min)) {
    successful = false;
  }

  double loop_detect_reset_time;
  if (!nh_->getParam("loop_detect_reset_time", loop_detect_reset_time)) {
    successful = false;
  }

  int max_loop_candidates;
  if (!nh_->getParam("max_loop_candidates", max_loop_candidates)) {
    successful = false;
  }

  int gps_align_num_corr;
  if (!nh_->getParam("gps_align_num_corr", gps_align_num_corr)) {
    successful = false;
  }

  double gps_align_cov_max;
  if (!nh_->getParam("gps_align_cov_max", gps_align_cov_max)) {
    successful = false;
  }

  int loop_detect_skip_kf;
  if (!nh_->getParam("loop_detect_skip_kf", loop_detect_skip_kf)) {
    successful = false;
  }

  double information_odom_drift_yaw;
  if (!nh_->getParam("information_odom_drift_yaw", information_odom_drift_yaw)) {
    successful = false;
  }

  double information_odom_drift_p;
  if (!nh_->getParam("information_odom_drift_p", information_odom_drift_p)) {
    successful = false;
  }

  double information_odom_map_yaw;
  if (!nh_->getParam("information_odom_map_yaw", information_odom_map_yaw)) {
    successful = false;
  }

  double information_odom_map_p;
  if (!nh_->getParam("information_odom_map_p", information_odom_map_p)) {
    successful = false;
  }

  double information_odom_edges_yaw;
  if (!nh_->getParam("information_odom_edges_yaw", information_odom_edges_yaw)) {
    successful = false;
  }

  double information_odom_edges_p;
  if (!nh_->getParam("information_odom_edges_p", information_odom_edges_p)) {
    successful = false;
  }

  double information_loop_edges_yaw;
  if (!nh_->getParam("information_loop_edges_yaw", information_loop_edges_yaw)) {
    successful = false;
  }

  double information_loop_edges_p;
  if (!nh_->getParam("information_loop_edges_p", information_loop_edges_p)) {
    successful = false;
  }

  bool ignore_gps_altitude;
  if (!nh_->getParam("ignore_gps_altitude", ignore_gps_altitude)) {
    successful = false;
  }


  int local_opt_window_size;
  if (!nh_->getParam("local_opt_window_size", local_opt_window_size)) {
    successful = false;
  }

  int rel_pose_corr_min;
  if (!nh_->getParam("rel_pose_corr_min", rel_pose_corr_min)) {
    successful = false;
  }


  if (successful) {
    params = SystemParameters(
      num_agents_,
      simulation_,
      cam_vector,
      gps_parameters, 
      loop_candidate_min_score,
      loop_image_min_matches,
      loop_detect_sac_thresh,
      loop_detect_sac_max_iter,
      loop_detect_min_sac_inliers,
      loop_detect_min_sac_inv_inliers,
      loop_detect_min_pose_inliers,
      rel_pose_outlier_norm_min,
      loop_detect_reset_time,
      max_loop_candidates,
      gps_align_num_corr,
      gps_align_cov_max,
      loop_detect_skip_kf,
      information_odom_drift_yaw,
      information_odom_drift_p,
      information_odom_map_yaw,
      information_odom_map_p, 
      information_odom_edges_yaw,
      information_odom_edges_p, 
      information_loop_edges_yaw,
      information_loop_edges_p,
      gps_active,
      ignore_gps_altitude,
      local_opt_window_size,
      rel_pose_corr_min);
  } else {
    return successful;
  }

  // Read the vocabulary
  std::string bow_voc_file;
  successful &= nh_->getParam("bow_voc", bow_voc_file);
  params.voc_ptr = std::make_shared<BRISKVocabulary>(bow_voc_file);

  return successful;
}

} // namespace pgbe