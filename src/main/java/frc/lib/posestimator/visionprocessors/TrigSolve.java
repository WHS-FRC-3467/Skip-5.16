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
import java.util.function.Function;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

@RequiredArgsConstructor
@Accessors(fluent = true)
public class TrigSolve implements VisionProcessor {

    public static Optional<Pose2d> solveTrigPosition(
        AprilTagFieldLayout fieldLayout,
        CameraProperties camera,
        PhotonTrackedTarget target,
        Rotation2d heading)
    {

        Translation2d camToTagTranslation =
            new Translation3d(
                target.getBestCameraToTarget().getTranslation().getNorm(),
                new Rotation3d(
                    0,
                    -Math.toRadians(target.getPitch()),
                    -Math.toRadians(target.getYaw())))
                        .rotateBy(camera.robotToCamera().getRotation())
                        .toTranslation2d()
                        .rotateBy(heading);

        var tagPoseOpt = fieldLayout.getTagPose(target.getFiducialId());
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        var tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation =
            tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
            camera.robotToCamera().getTranslation().toTranslation2d().unaryMinus()
                .rotateBy(heading);

        Pose2d robotPose =
            new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), heading);

        return Optional.of(robotPose);
    }

    private final AprilTagFieldLayout fieldLayout;

    /**
     * A function that selects an optional {@link PhotonTrackedTarget} from a list of
     * {@link PhotonTrackedTarget}s. This can be used to determine which AprilTag target to
     * prioritize or process based on a custom selection strategy. Return {@link Optional#empty()}
     * to reject the observation entirely.
     */
    private final Function<List<PhotonTrackedTarget>, Optional<PhotonTrackedTarget>> aprilTagChooser;

    @Override
    public Optional<VisionPoseRecord> processVisionObservation(
        PhotonPipelineResult observation,
        CameraProperties camera,
        Rotation2d heading)
    {
        var tagObservations = observation.getTargets();

        // Nothing to go off of
        if (tagObservations.isEmpty()) {
            return Optional.empty();
        }

        // The observation that matches the tag we're looking for
        var optionalWantedTarget = aprilTagChooser.apply(tagObservations);

        // It wasn't found
        if (optionalWantedTarget.isEmpty()) {
            return Optional.empty();
        }

        PhotonTrackedTarget wantedTarget = optionalWantedTarget.get();

        return solveTrigPosition(fieldLayout, camera, wantedTarget, heading)
            .map(p -> new VisionPoseRecord(new Pose3d(p), List.of(wantedTarget.getFiducialId()),
                wantedTarget.getBestCameraToTarget().getTranslation().getNorm()));
    }
}
