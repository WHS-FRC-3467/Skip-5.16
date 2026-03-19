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
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Generates the Object Detection subsystem which consists of some number of Object Detection
 * Devices (each representing a unique camera) each with their own inputs data structure that
 * periodically updates via a call from the device layer to the IO implementation layer.
 */
public class ObjectDetector extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final ObjectDetection objectDetection;
    private int maxDetectionsSize = 0;
    private static final Translation2d INVALID_TRANSLATION = new Translation2d(-1, -1);

    @Getter private List<Optional<ObjectDetectionObservation>> latestObjectObservation;
    @Getter private Optional<ObjectDetectionObservation> latestBigContourObservation;
    @Getter private Optional<ObjectDetectionObservation> latestLowContourObservation;

    // Blue left canonical ROI in robot coordinates for object detection
    private static final Distance ROI_X_START = Meters.of(0);
    private static final Distance ROI_X_END = Meters.of(4.63);
    private static final Distance ROI_Y_START = Meters.of(-0.61);
    private static final Distance ROI_Y_END = Meters.of(2.44);

    /**
     * Constructs a new ObjectDetector subsystem with the specified IO implementation. Creates an
     * Object Detection device that can periodically update its inputs field.
     *
     * @param cameraName the name of the camera for logging purposes
     * @param io the IO implementation for object detection (real, sim, or replay)
     */
    public ObjectDetector(String cameraName, ObjectDetectionIO io) {
        objectDetection = new ObjectDetection(cameraName, io);
    }

    // Private helper for generating latest ML Object Observation
    private Optional<ObjectDetectionObservation> generateObjectObservation(
            PhotonTrackedTarget target) {
        // Attempt to generate full Object record using ML model
        Optional<ObjectDetectionObservation> observation =
                objectDetection.getObjectObservation(
                        target,
                        ObjectDetectorConstants.CAMERA0_TRANSFORM,
                        ObjectDetectorConstants.OBJECT0_HEIGHT_METERS,
                        1,
                        0,
                        1,
                        0,
                        robotState.getEstimatedPose());
        // Return latest Object observation (full, partial, or empty record)
        return observation;
    }

    // Private helper for generating latest Contour observation
    private Optional<ObjectDetectionObservation> generateContourObservation(
            ContourSelectionMode selection) {
        // Attempt to generate partial Object record using Blob model
        Optional<ObjectDetectionObservation> observation =
                objectDetection.getContourObservation(objectDetection.getTargets(), selection);
        // Return latest Blob observation (partial or empty record)
        return observation;
    }

    // Private helper for logging Object Detection values. -1 for stale values.
    private void logObjectObservation(List<Optional<ObjectDetectionObservation>> observation) {
        String prefix = "Detection/ML:";
        int detectionSize = observation.size();
        maxDetectionsSize = Math.max(detectionSize, maxDetectionsSize);

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

    // Private helper for logging Contour values. Object ID = -9999 for stale values.
    private void logContourObservation(
            Optional<ObjectDetectionObservation> observation, ContourSelectionMode mode) {
        String prefix = "Detection/" + "Contour: " + mode + ": ";
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

    /**
     * Ascertain whether a detected object's pose is within the robot local coordinates defined
     * above.
     */
    private boolean isObjectWithinROI(Optional<ObjectDetectionObservation> observation) {
        if (observation.isEmpty() || observation.get().objectPose().isEmpty()) {
            return false;
        }
        Translation2d translation =
                robotState
                        .getEstimatedPose()
                        .minus(observation.get().objectPose().get())
                        .getTranslation()
                        .unaryMinus();
        // If robot is on right side, flip blue left canonical ROI across the robot-local X-axis to get correct
        // region in robot coordinates
        boolean isRightSide = robotState.getEstimatedPose().getTranslation().getY()
                < FieldConstants.FIELD_WIDTH / 2.0;
        if (isRightSide) {
            translation = new Translation2d(translation.getX(), -translation.getY());
        }

        return translation.getX() >= ROI_X_START.in(Meters)
                && translation.getX() <= ROI_X_END.in(Meters)
                && translation.getY() >= ROI_Y_START.in(Meters)
                && translation.getY() <= ROI_Y_END.in(Meters);
    }

    @Override
    public void periodic() {
        // Update the inputs data structure associated with this Object Detection camera with latest
        // readings
        objectDetection.periodic();
        // Now that inputs are updated, re-populate observations with new data
        if (objectDetection.getTargets().length > 0) {
            // Generate latest ML Object observations
            latestObjectObservation =
                    Arrays.stream(objectDetection.getTargets())
                            .map(this::generateObjectObservation)
                            .toList();
            // Generate latest Contour observations
            latestBigContourObservation = generateContourObservation(ContourSelectionMode.LARGEST);
            latestLowContourObservation = generateContourObservation(ContourSelectionMode.LOWEST);
            // Filter out observations that are outside of the defined ROI

        } else {
            // Prevent stale data from persisting
            latestObjectObservation = List.of();
            latestBigContourObservation = Optional.empty();
            latestLowContourObservation = Optional.empty();
        }
        // Log Object & Blob observations for sim
        logObjectObservation(latestObjectObservation);
        logContourObservation(latestBigContourObservation, ContourSelectionMode.LARGEST);
        logContourObservation(latestLowContourObservation, ContourSelectionMode.LOWEST);
    }
}
