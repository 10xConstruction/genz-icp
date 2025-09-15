// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
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

#include "GenZICP.hpp"

#include <unistd.h>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace genz_icp::pipeline {

std::ofstream debug_file("/tmp/genz_debug.log", std::ios::out | std::ios::app);

#define DEBUG_LOG(msg)                                         \
    do {                                                       \
        if (config_.print_debug) {                             \
            std::cerr << "[GenZ-DEBUG] " << msg << std::endl;  \
            std::cerr.flush();                                 \
        }                                                      \
        if (config_.save_debug && debug_file.is_open()) {      \
            debug_file << "[GenZ-DEBUG] " << msg << std::endl; \
            debug_file.flush();                                \
        }                                                      \
    } while (0)

#define WARN_LOG(msg)                                         \
    do {                                                      \
        if (config_.print_warn) {                             \
            std::cerr << "[GenZ-WARN] " << msg << std::endl;  \
            std::cerr.flush();                                \
        }                                                     \
        if (config_.save_debug && debug_file.is_open()) {     \
            debug_file << "[GenZ-WARN] " << msg << std::endl; \
            debug_file.flush();                               \
        }                                                     \
    } while (0)

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        // TODO(Nacho) Add some asserts here to sanitize the timestamps

        //  If not enough poses for the estimation, do not de-skew
        const size_t N = poses().size();
        if (N <= 2) return frame;

        // Estimate linear and angular velocities
        const auto &start_pose = poses_[N - 2];
        const auto &finish_pose = poses_[N - 1];

        return DeSkewScan(frame, timestamps, start_pose, finish_pose);
    }();
    return RegisterFrame(deskew_frame);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    // Preprocess the input cloud
    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);

    // ====== DATA QUALITY GUARD ======
    static size_t consecutive_bad_frames = 0;
    static size_t last_good_frame_size = 0;

    bool data_quality_ok = true;
    std::string failure_reason;

    DEBUG_LOG("min_points_absolute: " << config_.min_points_absolute);
    DEBUG_LOG("min_input_points: " << config_.min_input_points);

    // Check 1: Minimum absolute points
    if (cropped_frame.size() < config_.min_points_absolute) {
        data_quality_ok = false;
        failure_reason = "Too few points after filtering: " + std::to_string(cropped_frame.size());
    }

    // Check 2: Minimum input points
    if (frame.size() < config_.min_input_points) {
        data_quality_ok = false;
        failure_reason = "Insufficient input points: " + std::to_string(frame.size());
    }

    // Check 4: Sudden drop from previous good frame
    if (last_good_frame_size > 0 && static_cast<double>(cropped_frame.size()) <
                                        static_cast<double>(last_good_frame_size) * 0.3) {
        data_quality_ok = false;
        failure_reason = "Sudden data drop: " + std::to_string(cropped_frame.size()) +
                         " vs previous " + std::to_string(last_good_frame_size);
    }

    if (!data_quality_ok) {
        consecutive_bad_frames++;
        WARN_LOG("POOR DATA QUALITY [" << consecutive_bad_frames << "/"
                                       << config_.max_consecutive_bad_frames
                                       << "]: " << failure_reason);

        if (consecutive_bad_frames >= config_.max_consecutive_bad_frames) {
            WARN_LOG("TOO MANY BAD FRAMES - RESETTING SYSTEM STATE");

            // Reset adaptive threshold to prevent corruption propagation
            adaptive_threshold_ = AdaptiveThreshold(config_.initial_threshold,
                                                    config_.min_motion_th, config_.max_range);

            // Clear recent pose history to prevent bad motion prediction
            if (poses_.size() > 1) {
                poses_.erase(poses_.end() - std::min(poses_.size(), size_t(3)), poses_.end());
            }

            consecutive_bad_frames = 0;
        }

        // Return empty correspondences and reuse last pose
        std::vector<Eigen::Vector3d> empty_planar, empty_non_planar;
        if (!poses_.empty()) {
            // Don't advance pose - stay at last known good position
            WARN_LOG("Maintaining last known pose due to poor data quality");
            return {empty_planar, empty_non_planar};
        }
    } else {
        // Good data - reset bad frame counter and update last good size
        consecutive_bad_frames = 0;
        last_good_frame_size = cropped_frame.size();
        DEBUG_LOG("Data quality OK: " << cropped_frame.size() << " points");
    }

    // Continue with normal processing only if data quality is acceptable
    if (!data_quality_ok && consecutive_bad_frames < config_.max_consecutive_bad_frames) {
        std::vector<Eigen::Vector3d> empty_planar, empty_non_planar;
        return {empty_planar, empty_non_planar};
    }

    // LOG: Range filtering parameters and their effect
    size_t filtered_out = frame.size() - cropped_frame.size();
    DEBUG_LOG("==================================================================");
    DEBUG_LOG("=== RANGE FILTERING ===");
    DEBUG_LOG("Parameters: min_range=" << config_.min_range << "m, max_range=" << config_.max_range
                                       << "m");
    DEBUG_LOG("Effect: " << frame.size() << " -> " << cropped_frame.size() << " points (filtered "
                         << filtered_out << " points)");
    if (cropped_frame.size() < 100) {
        WARN_LOG("Very few points after preprocessing!");
    }

    // Adapt voxel size based on LOCUS 2.0's adaptive voxel grid filter
    static double voxel_size = config_.voxel_size;  // Initial voxel size
    const auto source_tmp = genz_icp::VoxelDownsample(cropped_frame, voxel_size);

    // LOG: Adaptive voxelization calculation breakdown
    DEBUG_LOG("=== ADAPTIVE VOXELIZATION ===");
    DEBUG_LOG("Parameters: initial_voxel_size=" << config_.voxel_size << "m, desired_points="
                                                << config_.desired_num_voxelized_points);
    DEBUG_LOG("Current voxel_size=" << voxel_size
                                    << "m -> points_after_voxel=" << source_tmp.size());

    double scale_factor = static_cast<double>(source_tmp.size()) /
                          static_cast<double>(config_.desired_num_voxelized_points);
    double raw_adaptive_size = voxel_size * scale_factor;
    double adaptive_voxel_size = genz_icp::Clamp(raw_adaptive_size, 0.02, 2.0);

    DEBUG_LOG("Calculation: scale_factor=" << scale_factor << " (actual/desired points)");
    DEBUG_LOG("Raw adaptive size=" << raw_adaptive_size << "m, clamped to=" << adaptive_voxel_size
                                   << "m");

    // Re-voxelize using the adaptive voxel size
    const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
    voxel_size = adaptive_voxel_size;  // Save for the next frame

    DEBUG_LOG("Final voxelization: " << cropped_frame.size() << " -> " << source.size()
                                     << " points for registration");
    DEBUG_LOG("Final voxelization: " << cropped_frame.size() << " -> " << frame_downsample.size()
                                     << " points for map update");

    // Get motion prediction and adaptive_threshold
    double sigma = GetAdaptiveThreshold();
    const double MAX_SIGMA_PLANAR = 0.15;      // 15cm max for 4cm/frame motion
    const double RESET_SIGMA_THRESHOLD = 0.5;  // Reset if above 50cm

    if (sigma > RESET_SIGMA_THRESHOLD) {
        DEBUG_LOG("RESETTING corrupted adaptive threshold: " << sigma);
        adaptive_threshold_ =
            AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
        sigma = GetAdaptiveThreshold();
    }

    // Always clamp for planar robot
    if (sigma > MAX_SIGMA_PLANAR) {
        sigma = MAX_SIGMA_PLANAR;
        DEBUG_LOG("Clamping sigma for planar robot: " << sigma);
    }

    // LOG: Adaptive threshold parameters and calculation
    // How far apart two points can be to still be considered a valid correspondence
    DEBUG_LOG("=== ADAPTIVE THRESHOLD ===");
    DEBUG_LOG("Parameters: initial_threshold=" << config_.initial_threshold << "m, min_motion_th="
                                               << config_.min_motion_th << "m");
    DEBUG_LOG("Computed sigma=" << sigma << "m (used for correspondence distance)");

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    // LOG: Motion prediction breakdown
    DEBUG_LOG("=== MOTION PREDICTION ===");
    DEBUG_LOG("Pose history size=" << poses_.size());
    if (poses_.size() >= 2) {
        auto pred_trans = prediction.translation();
        auto pred_rot = prediction.so3().log();
        DEBUG_LOG("Predicted motion: trans=[" << pred_trans.x() << "," << pred_trans.y() << ","
                                              << pred_trans.z() << "]m");
        DEBUG_LOG("Predicted rotation: [" << pred_rot.x() << "," << pred_rot.y() << ","
                                          << pred_rot.z() << "]rad");
        DEBUG_LOG("Prediction magnitude: " << pred_trans.norm() << "m translation, "
                                           << pred_rot.norm() << "rad rotation");
    } else {
        DEBUG_LOG("Using identity prediction (insufficient pose history)");
    }

    // LOG: Registration input parameters
    DEBUG_LOG("=== REGISTRATION INPUTS ===");
    DEBUG_LOG("Parameters: max_correspondence_distance=" << (3.0 * sigma) << "m, kernel_size="
                                                         << (sigma / 3.0) << "m");
    DEBUG_LOG("Source points for registration: " << source.size());
    DEBUG_LOG("Local map size: " << local_map_.Pointcloud().size() << " points");

    // Run GenZ-ICP
    const auto &[new_pose, planar_points, non_planar_points] =
        registration_.RegisterFrame(source,         //
                                    local_map_,     //
                                    initial_guess,  //
                                    3.0 * sigma,    // max_correspondence_distance
                                    sigma / 3.0);   // kernel

    // LOG: Registration results and their impact
    DEBUG_LOG("=== REGISTRATION RESULTS ===");
    DEBUG_LOG("Output correspondences: planar=" << planar_points.size()
                                                << ", non_planar=" << non_planar_points.size());

    if (planar_points.size() + non_planar_points.size() == 0) {
        WARN_LOG("NO CORRESPONDENCES FOUND! Check sigma/distance parameters");
    }

    double alpha = static_cast<double>(planar_points.size()) /
                   static_cast<double>(planar_points.size() + non_planar_points.size());
    DEBUG_LOG("Alpha (planarity ratio)=" << alpha
                                         << " (affects: structured vs unstructured optimization)");

    // LOG: Pose change analysis
    auto final_trans = new_pose.translation();
    auto final_rot = new_pose.so3().log();
    DEBUG_LOG("Final pose: trans=[" << final_trans.x() << "," << final_trans.y() << ","
                                    << final_trans.z() << "]m");
    DEBUG_LOG("Final pose: rot=[" << final_rot.x() << "," << final_rot.y() << "," << final_rot.z()
                                  << "]rad");

    // Compare initial guess vs final result
    // auto initial_trans = initial_guess.translation();
    auto correction = (initial_guess.inverse() * new_pose).translation();
    DEBUG_LOG("Correction from prediction: [" << correction.x() << "," << correction.y() << ","
                                              << correction.z() << "]m");
    DEBUG_LOG("Correction magnitude: " << correction.norm() << "m");

    if (correction.norm() > 1.0) {
        WARN_LOG("Large correction from prediction! Magnitude: " << correction.norm() << "m");
    }

    // Frame-to-frame motion analysis
    if (!poses_.empty()) {
        auto frame_motion = (poses_.back().inverse() * new_pose).translation();
        DEBUG_LOG("Frame-to-frame motion: " << frame_motion.norm() << "m");

        if (frame_motion.norm() > 5.0) {
            WARN_LOG("POTENTIAL ODOMETRY JUMP! Motion: " << frame_motion.norm() << "m");
        }
    }

    // Update system state
    const auto model_deviation = initial_guess.inverse() * new_pose;
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    // LOG: Map update impact
    size_t map_size_before = local_map_.Pointcloud().size();
    local_map_.Update(frame_downsample, new_pose);
    size_t map_size_after = local_map_.Pointcloud().size();

    DEBUG_LOG("=== MAP UPDATE ===");
    DEBUG_LOG("Parameters: map_cleanup_radius=" << config_.map_cleanup_radius << "m");
    DEBUG_LOG("Map size: " << map_size_before << " -> " << map_size_after << " points");
    DEBUG_LOG("Added " << frame_downsample.size() << " new points");

    poses_.push_back(new_pose);
    DEBUG_LOG("Total trajectory length: " << poses_.size() << " poses");
    DEBUG_LOG("==================================================================");

    return {planar_points, non_planar_points};
}

GenZICP::Vector3dVectorTuple GenZICP::Voxelize(const std::vector<Eigen::Vector3d> &frame,
                                               double adaptive_voxel_size) const {
    const auto frame_downsample = genz_icp::VoxelDownsample(
        frame, std::max(adaptive_voxel_size * 0.5, 0.02));  // localmap update
    const auto source =
        genz_icp::VoxelDownsample(frame_downsample, adaptive_voxel_size * 1.0);  // registration
    return {source, frame_downsample};
}

double GenZICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d GenZICP::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool GenZICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

}  // namespace genz_icp::pipeline
