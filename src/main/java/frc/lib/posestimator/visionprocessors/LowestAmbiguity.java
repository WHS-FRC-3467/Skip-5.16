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
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose using the lowest
 * ambiguity tag observation
 */
@RequiredArgsConstructor
@Accessors(fluent = true)
public class LowestAmbiguity implements VisionProcessor {

    private final AprilTagFieldLayout fieldLayout;

    @Override
    public Optional<VisionPoseRecord> processVisionObservation(
        PhotonPipelineResult observation,
        CameraProperties camera,
        Rotation2d heading)
    {

        // Ignore invalid observations
        if (observation.getTargets().isEmpty()) {
            return Optional.empty();
        }

        PhotonTrackedTarget lowestAmbiguity = observation.getBestTarget();

        var optionalTargetPose = fieldLayout.getTagPose(lowestAmbiguity.getFiducialId());
        if (optionalTargetPose.isEmpty()) {
            return Optional.empty();
        }
        Pose3d targetPose = optionalTargetPose.get();

        Transform3d targetToCamera = lowestAmbiguity.getBestCameraToTarget().inverse();
        Transform3d cameraToRobot = camera.robotToCamera().inverse();
        Pose3d robotPose = targetPose.plus(targetToCamera).plus(cameraToRobot);

        double distanceFromCamera = targetToCamera.getTranslation().getNorm();

        return Optional.of(
            new VisionPoseRecord(
                robotPose,
                List.of(lowestAmbiguity.fiducialId),
                distanceFromCamera));
    }
}
