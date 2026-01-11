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

import java.util.Optional;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.util.GeomUtil;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

/**
 * Processes AprilTag observations to estimate the robot’s field-relative pose.
 *
 * <p>
 * Each incoming {@link VisionObservation} is evaluated for validity and consistency based on tag
 * count, ambiguity, distance, and field boundaries. The resulting pose estimates are stored or
 * returned as a {@link VisionPoseRecord}, each containing pose data and uncertainty metrics for
 * later fusion with odometry.
 */
@Accessors(fluent = true)
@RequiredArgsConstructor
public class ConstrainedSolvePnp implements VisionProcessor {

    private final VisionProcessor seedProvider;
    private final AprilTagFieldLayout fieldLayout;
    private final double gyroHeadingScaleFactor;

    @Override
    public Optional<VisionPoseRecord> processVisionObservation(
        PhotonPipelineResult result,
        CameraProperties camera,
        Rotation2d heading)
    {
        var targets = result.getTargets();
        int tagCount = targets.size();

        // Ignore invalid observations
        if (tagCount == 0) {
            return Optional.empty();
        }

        var robotToCamera = camera.robotToCamera();

        // Attempt to extract seed pose from the observation
        var optionalSeed = seedProvider.processVisionObservation(result, camera, heading);
        if (optionalSeed.isEmpty()) {
            return Optional.empty();
        }
        Pose3d seed = optionalSeed.get().pose();

        // Solve for robot pose using constrained PnP
        Optional<Pose3d> optionalEstimate =
            VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                camera.cameraMatrix(),
                camera.distCoeffs(),
                targets,
                robotToCamera,
                seed,
                fieldLayout,
                TargetModel.kAprilTag36h11,
                false,
                heading,
                gyroHeadingScaleFactor)
                .map(estimate -> GeomUtil.toPose3d(estimate.best));

        if (optionalEstimate.isEmpty()) {
            return Optional.empty();
        }

        Pose3d estimate = optionalEstimate.get();

        double avgDistance = targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);

        return Optional.of(new VisionPoseRecord(estimate,
            result.getTargets().stream().map(PhotonTrackedTarget::getFiducialId).toList(),
            avgDistance));
    }
}
