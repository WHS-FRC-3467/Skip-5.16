/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.posestimator.visionprocessors;

import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.devices.AprilTagCamera.CameraProperties;

/**
 * A processing interface for converting raw vision observations into global robot pose estimates.
 *
 * <p>
 * Implementations of {@code VisionProcessor} interpret vision data and return an estimated
 * {@link Pose3d} of the robot along with confidence metrics. These estimates are used by
 * higher-level pose estimators such as {@link PoseEstimator} to fuse vision data with odometry.
 */
public interface VisionProcessor {
    /** Stores a vision pose estimate along with computed uncertainty metrics. */
    public static final record VisionPoseRecord(
        Pose3d pose,
        List<Integer> tagsUsed,
        double averageDistanceMeters) {
    }

    Optional<VisionPoseRecord> processVisionObservation(
        PhotonPipelineResult observation,
        CameraProperties camera,
        Rotation2d heading);
}
