/*
 * Copyright (C) 2026 Windham Windup
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

package frc.lib.devices;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.List;
import java.util.Optional;

/**
 * Utility methods for interpreting CamCam object-detection frames.
 *
 * <p>This keeps the useful object-localization behavior from the previous PhotonVision-backed stack
 * while remaining independent from any specific IO layer.
 */
public class ObjectDetection {

    /**
     * One detection reported by the coprocessor for a single frame.
     *
     * @param id target class or object ID reported by the coprocessor
     * @param pitchDegrees target pitch relative to the camera centerline in degrees
     * @param yawDegrees target yaw relative to the camera centerline in degrees
     * @param area normalized image area reported for the target
     */
    public record DetectionTarget(int id, double pitchDegrees, double yawDegrees, double area) {}

    /**
     * One object-detection frame emitted by the coprocessor.
     *
     * @param timestampSeconds frame timestamp in seconds
     * @param targets detections reported for the frame
     */
    public record DetectionFrame(double timestampSeconds, List<DetectionTarget> targets) {
        public DetectionFrame {
            targets = targets == null ? List.of() : List.copyOf(targets);
        }
    }

    /**
     * Represents an object detection observation.
     *
     * @param objID Object ID, or a negative sentinel value when the pipeline does not provide one
     * @param confidence Confidence for {@code objID}, or a negative sentinel value when unavailable
     * @param pitch Pitch of the object relative to the camera centerline
     * @param yaw Yaw of the object relative to the camera centerline
     * @param area Area of the object in the image
     * @param distance Approximate 2d robot-relative distance to the object
     * @param objectPose Estimated field-relative pose of the detected object
     */
    public record ObjectDetectionObservation(
            int objID,
            double confidence,
            Angle pitch,
            Angle yaw,
            double area,
            Optional<Distance> distance,
            Optional<Pose2d> objectPose) {}

    /**
     * Returns the latest object observation.
     *
     * <p>CamCam currently reports object ID, pitch, yaw, and area. Confidence is not available and
     * is returned as {@code -1}.
     *
     * @param target raw target reported by CamCam
     * @param robotToCamera transform from the robot frame to the camera frame
     * @param objectPhysicalHeightMeters physical height of the detected object in meters
     * @param rangeCalFactor scale factor applied to the range estimate
     * @param rangeCalOffset additive offset applied to the range estimate
     * @param headingCalFactor scale factor applied to the heading estimate
     * @param headingCalOffset additive offset applied to the heading estimate
     * @param robotPose current field-relative robot pose
     * @return latest observation, or {@link Optional#empty()} when {@code target} is null
     */
    public Optional<ObjectDetectionObservation> getObjectObservation(
            DetectionTarget target,
            Transform3d robotToCamera,
            double objectPhysicalHeightMeters,
            double rangeCalFactor,
            double rangeCalOffset,
            double headingCalFactor,
            double headingCalOffset,
            Pose2d robotPose) {
        if (target == null) {
            return Optional.empty();
        }

        double range =
                rangeToTargetPitch(
                        target,
                        robotToCamera,
                        objectPhysicalHeightMeters / 2.0,
                        rangeCalFactor,
                        rangeCalOffset);
        if (range == -1.0) {
            return Optional.of(
                    new ObjectDetectionObservation(
                            target.id(),
                            -1,
                            Degrees.of(target.pitchDegrees()),
                            Degrees.of(target.yawDegrees()),
                            target.area(),
                            Optional.empty(),
                            Optional.empty()));
        }

        double heading =
                headingToTargetYaw(
                        target, robotToCamera, range, headingCalFactor, headingCalOffset);
        double distance = distanceToTarget2d(range, heading);
        Translation2d targetLocation = estimateTargetToField(range, heading, robotPose);

        return Optional.of(
                new ObjectDetectionObservation(
                        target.id(),
                        -1,
                        Degrees.of(target.pitchDegrees()),
                        Degrees.of(target.yawDegrees()),
                        target.area(),
                        Optional.of(Meters.of(distance)),
                        Optional.of(new Pose2d(targetLocation, new Rotation2d()))));
    }

    /**
     * Returns an object observation without attempting field localization.
     *
     * <p>This is used when the robot pose buffer cannot provide a pose sample for the frame
     * timestamp. Pitch, yaw, and area remain valid, but distance and object pose are omitted.
     *
     * @param target raw target reported by CamCam
     * @return partial observation, or {@link Optional#empty()} when {@code target} is null
     */
    public Optional<ObjectDetectionObservation> getUnlocalizedObservation(DetectionTarget target) {
        if (target == null) {
            return Optional.empty();
        }

        return Optional.of(
                new ObjectDetectionObservation(
                        target.id(),
                        -1,
                        Degrees.of(target.pitchDegrees()),
                        Degrees.of(target.yawDegrees()),
                        target.area(),
                        Optional.empty(),
                        Optional.empty()));
    }

    private double rangeToTargetPitch(
            DetectionTarget target,
            Transform3d cameraTransform,
            double targetHeightMeters,
            double cameraCalFactor,
            double cameraOffset) {
        double tolerance = 0.175;
        double cameraRangeDelta = cameraTransform.getX();
        double cameraHeightMeters = cameraTransform.getZ();
        double cameraPitchRadians = -cameraTransform.getRotation().getY();
        double cameraYawRadians = cameraTransform.getRotation().getZ();

        if (Math.abs(targetHeightMeters - cameraHeightMeters) <= tolerance) {
            return -1.0;
        }

        double yawCorrection = 1.0;
        if (cameraYawRadians != 0.0) {
            yawCorrection =
                    Math.cos(Math.abs(cameraYawRadians - Math.toRadians(target.yawDegrees())));
        }

        return (((targetHeightMeters - cameraHeightMeters)
                                / Math.tan(
                                        cameraPitchRadians + Math.toRadians(target.pitchDegrees())))
                        * yawCorrection
                        * cameraCalFactor)
                + cameraOffset
                + cameraRangeDelta;
    }

    private double headingToTargetYaw(
            DetectionTarget target,
            Transform3d cameraTransform,
            double targetRangeMeters,
            double cameraCalFactor,
            double cameraOffset) {
        double cameraRangeMeters = targetRangeMeters - cameraTransform.getX();
        double cameraHeadingDelta = cameraTransform.getY();
        double cameraYawRadians = cameraTransform.getRotation().getZ();
        return (Math.tan(cameraYawRadians - Math.toRadians(target.yawDegrees()))
                        * cameraRangeMeters
                        * cameraCalFactor)
                + cameraOffset
                + cameraHeadingDelta;
    }

    private double distanceToTarget2d(double targetRangeMeters, double targetHeadingMeters) {
        return Math.hypot(targetRangeMeters, targetHeadingMeters);
    }

    private Translation2d estimateTargetToField(
            double targetRangeMeters, double targetHeadingMeters, Pose2d robotPose) {
        return robotPose
                .transformBy(
                        new Transform2d(targetRangeMeters, targetHeadingMeters, new Rotation2d()))
                .getTranslation();
    }
}
