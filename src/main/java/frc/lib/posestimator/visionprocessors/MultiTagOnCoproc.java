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
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

/**
 * A {@link VisionProcessor} implementation that estimates the robot's pose using multi-tag camera
 * pose data produced directly by a coprocessor.
 *
 * <p>
 * This processor assumes that the vision coprocessor provides a precomputed camera pose based on
 * multiple AprilTag detections, and converts that into a field-relative robot pose using the known
 * camera-to-robot transform.
 */
@RequiredArgsConstructor
@Accessors(fluent = true)
public class MultiTagOnCoproc implements VisionProcessor {

    private final Optional<VisionProcessor> fallbackProcessor;
    private final AprilTagFieldLayout fieldLayout;

    @Override
    public Optional<VisionPoseRecord> processVisionObservation(
        PhotonPipelineResult result,
        CameraProperties camera,
        Rotation2d heading)
    {
        if (result.getMultiTagResult().isEmpty()) {
            if (fallbackProcessor.isEmpty()) {
                return Optional.empty();
            }

            return fallbackProcessor.get().processVisionObservation(result, camera, heading);
        }

        var bestTF = result.getMultiTagResult().get().estimatedPose.best;
        var best =
            Pose3d.kZero
                .plus(bestTF) // field-to-camera
                .relativeTo(fieldLayout.getOrigin())
                .plus(camera.robotToCamera().inverse()); // field-to-robot

        // Compute distance-based uncertainty scaling
        double avgDistance = result.getTargets().stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);

        return Optional.of(
            new VisionPoseRecord(
                best,
                result.getMultiTagResult().get().fiducialIDsUsed.stream().map(Integer::valueOf)
                    .toList(),
                avgDistance));
    }
}
