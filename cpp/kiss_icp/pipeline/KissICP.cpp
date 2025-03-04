// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "KissICP.hpp"

#include <Eigen/Core>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "easy/profiler.h"
#include <iostream>

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const bool is_vehicle_moving,
                                                    const Sophus::SE3d delta_pose,
                                                    const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    // EASY_THREAD("KISS-ICP Thread");
    EASY_FUNCTION(profiler::colors::Purple);

    EASY_BLOCK("Preprocessing", profiler::colors::Red);
    // Preprocess the input cloud
    const auto &preprocessed_frame = preprocessor_.Preprocess(frame, timestamps, last_delta_);
    EASY_END_BLOCK;


    // Voxelize
    EASY_BLOCK("Voxelize", profiler::colors::Orange);
    const auto &[source, frame_downsample] = Voxelize(preprocessed_frame);
    EASY_END_BLOCK;

    // Get adaptive_threshold
    EASY_BLOCK("ComputeThreshold", profiler::colors::Orange);
    const double sigma = adaptive_threshold_.ComputeThreshold();
    EASY_END_BLOCK;

    Sophus::SE3d initial_guess = Sophus::SE3d();
    // =========== Compute initial_guess for ICP ===========
    switch (config_.initial_guess_mode)
    {
    case KISSConfig::InitialGuess::CV:
        // ----- constant-velocity approach --------------------
        initial_guess = last_pose_ * last_delta_;
        break;
    case KISSConfig::InitialGuess::VE:
        // ----- Velocity Estimation 3DoF prior -------------------------------
        initial_guess = last_pose_ * delta_pose;
        break;
    default:
        break;
    } 
    

    // ----- IMU prior + constant-velocity approach --------
    // const double alpha = 0.8;  // Weight factor on IMU vs. ICP
    // const auto interpolated_delta = Sophus::SE3d::exp(alpha * delta_pose.log() + (1 - alpha) * last_delta_.log());
    // const auto initial_guess = last_pose_ * interpolated_delta;

    auto new_pose = last_pose_; // TODO(Andrea): does it make sense to change this to initial_guess?
    if(is_vehicle_moving){
        // Run ICP
        EASY_BLOCK("ICP", profiler::colors::Yellow);
        new_pose = registration_.AlignPointsToMap(source,         // frame
                                                  local_map_,     // voxel_map
                                                  initial_guess,  // initial_guess
                                                  3.0 * sigma,    // max_correspondence_dist
                                                  sigma);         // kernel
        EASY_END_BLOCK;
    }
              
    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;
   
    // Update step: threshold, local map, delta, and the last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    EASY_BLOCK("Update Local Map", profiler::colors::Green);
    // auto new_pose_hesai_frame = new_pose * base_link_to_hesai_transform_;
    local_map_.Update(frame_downsample, new_pose);
    EASY_END_BLOCK;

    last_delta_ = last_pose_.inverse() * new_pose;
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (frame) and the points used for registration (source)
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    // EASY_FUNCTION(profiler::colors::Orange100);
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

}  // namespace kiss_icp::pipeline
