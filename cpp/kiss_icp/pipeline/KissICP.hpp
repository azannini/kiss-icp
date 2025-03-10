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
#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

#include <easy/profiler.h>
namespace kiss_icp::pipeline {

struct KISSConfig {
    enum InitialGuess { CV, VE };
    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 0.0;
    int max_points_per_voxel = 20;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // registration params
    int max_num_iterations = 500;
    double convergence_criterion = 0.0001;
    int max_num_threads = 0;

    // Motion compensation
    bool deskew = true;

    // Initial Guess Mode
    InitialGuess initial_guess_mode;

    bool profiling_enabled_ = true;
};

class KissICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

public:
    explicit KissICP(const KISSConfig &config)
        : config_(config),
          preprocessor_(config.max_range, config.min_range, config.deskew, config.max_num_threads),
          registration_(
              config.max_num_iterations, config.convergence_criterion, config.max_num_threads),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range) {
            if(config.profiling_enabled_){
                  EASY_PROFILER_ENABLE;
                  EASY_THREAD("KISS-ICP Thread");
            }
            //TODO(Andrea): Find a smarter way to do this
          }

public:
    Vector3dVectorTuple RegisterFrame(const bool is_vehicle_moving,
                                      const Sophus::SE3d delta_pose,
                                      const std::vector<Eigen::Vector3d> &frame,
                                      const std::vector<double> &timestamps);

    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d> &frame) const;

    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };

    const VoxelHashMap &VoxelMap() const { return local_map_; };
    VoxelHashMap &VoxelMap() { return local_map_; };

    const Sophus::SE3d &pose() const { return last_pose_; }
    Sophus::SE3d &pose() { return last_pose_; }

    const Sophus::SE3d &delta() const { return last_delta_; }
    Sophus::SE3d &delta() { return last_delta_; }

    double &mean_ICP_iterations() { return registration_.mean_iterations_ICP(); }
    std::vector<int> &num_iterations_ICP() { return registration_.num_iterations_ICP(); }
    std::vector<double> &max_distance_ICP() { return registration_.max_distance_ICP(); }

private:
    Sophus::SE3d last_pose_;
    Sophus::SE3d last_delta_;
    Sophus::SE3d base_link_to_hesai_transform_ = Sophus::SE3d(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(-0.0338, -0.0162, 0.698));

    // KISS-ICP pipeline modules
    KISSConfig config_;
    Preprocessor preprocessor_;
    Registration registration_;
    VoxelHashMap local_map_;
    AdaptiveThreshold adaptive_threshold_;
};

}  // namespace kiss_icp::pipeline
