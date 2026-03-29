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

package frc.robot.subsystems.objectdetector;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.DetectionFrame;
import frc.lib.devices.ObjectDetection.DetectionTarget;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.robot.RobotState;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

/**
 * Tracks the latest object-detection frame received from CamCam and exposes derived observations
 * for alignment commands.
 */
public class ObjectDetector extends SubsystemBase {
    private static final double STALE_OBSERVATION_TIMEOUT_SECONDS = 0.25;
    private static final Translation2d INVALID_TRANSLATION = new Translation2d(-1, -1);

    private final ObjectDetection objectDetection = new ObjectDetection();
    private final String cameraName;
    private final DoubleSupplier currentTimestampSupplier;
    private final DoubleFunction<Optional<Pose2d>> poseAtTimeSupplier;

    private int maxDetectionsSize = 0;
    private double lastFrameTimestampSeconds = Double.NEGATIVE_INFINITY;

    @Getter private List<Optional<ObjectDetectionObservation>> latestObjectObservation = List.of();
    @Getter private Optional<ObjectDetectionObservation> latestTargetObservation = Optional.empty();

    /**
     * Creates the subsystem wrapper for one logical object-detection camera stream.
     *
     * @param cameraName name used for logging this detector's outputs
     */
    public ObjectDetector(String cameraName) {
        this(cameraName, Timer::getTimestamp, RobotState.getInstance()::getPoseAtTime);
    }

    ObjectDetector(
            String cameraName,
            DoubleSupplier currentTimestampSupplier,
            DoubleFunction<Optional<Pose2d>> poseAtTimeSupplier) {
        this.cameraName = cameraName;
        this.currentTimestampSupplier = currentTimestampSupplier;
        this.poseAtTimeSupplier = poseAtTimeSupplier;
    }

    /**
     * Accepts the newest detection frame from the vision subsystem and refreshes derived
     * observations.
     *
     * @param frame latest CamCam object-detection frame
     */
    public void acceptLatestFrame(DetectionFrame frame) {
        double frameTimestampSeconds = frame.timestampSeconds();
        if (frameTimestampSeconds < lastFrameTimestampSeconds) {
            return;
        }

        lastFrameTimestampSeconds = frameTimestampSeconds;
        List<DetectionTarget> targets = frame.targets();
        if (targets.isEmpty()) {
            clearObservations();
            return;
        }

        Optional<Pose2d> robotPoseAtFrame = poseAtTimeSupplier.apply(frameTimestampSeconds);
        latestObjectObservation =
                targets.stream()
                        .map(target -> generateObjectObservation(target, robotPoseAtFrame))
                        .toList();
        latestTargetObservation = selectPreferredObservation(latestObjectObservation);
    }

    private Optional<ObjectDetectionObservation> generateObjectObservation(
            DetectionTarget target, Optional<Pose2d> robotPoseAtFrame) {
        if (robotPoseAtFrame.isEmpty()) {
            return objectDetection.getUnlocalizedObservation(target);
        }

        return objectDetection.getObjectObservation(
                target,
                ObjectDetectorConstants.CAMERA0_TRANSFORM,
                ObjectDetectorConstants.OBJECT0_HEIGHT_METERS,
                1,
                0,
                1,
                0,
                robotPoseAtFrame.get());
    }

    private Optional<ObjectDetectionObservation> selectPreferredObservation(
            List<Optional<ObjectDetectionObservation>> observations) {
        ObjectDetectionObservation preferred = null;
        double maxArea = Double.NEGATIVE_INFINITY;
        for (Optional<ObjectDetectionObservation> observation : observations) {
            if (observation.isEmpty() || observation.get().area() <= maxArea) {
                continue;
            }

            preferred = observation.get();
            maxArea = preferred.area();
        }
        return Optional.ofNullable(preferred);
    }

    private void clearObservations() {
        latestObjectObservation = List.of();
        latestTargetObservation = Optional.empty();
    }

    private boolean observationsStale() {
        return currentTimestampSupplier.getAsDouble() - lastFrameTimestampSeconds
                > STALE_OBSERVATION_TIMEOUT_SECONDS;
    }

    private void logObjectObservation(List<Optional<ObjectDetectionObservation>> observation) {
        String prefix = "Detection/ML:";
        int detectionSize = observation.size();
        maxDetectionsSize = Math.max(detectionSize, maxDetectionsSize);

        Logger.recordOutput("Detection/CameraName", cameraName);
        Logger.recordOutput(prefix + "Active Detections", detectionSize);
        for (int i = 0; i < maxDetectionsSize; i++) {
            Translation2d translation = INVALID_TRANSLATION;
            if (i < detectionSize) {
                Optional<ObjectDetectionObservation> detection = observation.get(i);
                if (detection.isPresent() && detection.get().objectPose().isPresent()) {
                    translation = detection.get().objectPose().get().getTranslation();
                }
            }
            Logger.recordOutput(prefix + i, translation);
        }
    }

    private void logTargetObservation(Optional<ObjectDetectionObservation> observation) {
        String prefix = "Detection/Target: ";
        if (observation.isPresent()) {
            ObjectDetectionObservation obs = observation.get();
            Logger.recordOutput(prefix + "Object ID", obs.objID());
            Logger.recordOutput(prefix + "Confidence", obs.confidence());
            Logger.recordOutput(prefix + "Pitch", obs.pitch().in(Degrees));
            Logger.recordOutput(prefix + "Yaw", obs.yaw().in(Degrees));
            Logger.recordOutput(prefix + "Area", obs.area());
        } else {
            Logger.recordOutput(prefix + "Object ID", -9999);
        }
    }

    @Override
    public void periodic() {
        if (!latestObjectObservation.isEmpty() && observationsStale()) {
            clearObservations();
        }

        logObjectObservation(latestObjectObservation);
        logTargetObservation(latestTargetObservation);
    }
}
