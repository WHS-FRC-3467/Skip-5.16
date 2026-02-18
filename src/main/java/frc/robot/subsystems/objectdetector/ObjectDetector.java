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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Generates the Object Detection subsystem which consists of some number of Object Detection
 * Devices (each representing a unique camera) each with their own inputs data structure that
 * periodically updates via a call from the device layer to the IO implementation layer.
 */
public class ObjectDetector extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final ObjectDetection objectDetection;

    @Getter private Optional<ObjectDetectionObservation> latestObjectObservation;
    @Getter private Optional<ObjectDetectionObservation> latestBigContourObservation;
    @Getter private Optional<ObjectDetectionObservation> latestLowContourObservation;
    @Getter private ArrayList<Translation2d> objectPoseBuffer = new ArrayList<>(10);

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

    // Private helper for generating latest ML Object Observation and updating internal buffer of
    // detected object poses (if it was successfully generated).
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
        // If no record/pose was generated, return empty/partial record early & don't update pose
        // buffer
        if (observation.isEmpty() || observation.get().objectPose().isEmpty()) {
            return observation;
        }
        // Pose generated: update object pose buffer of most-recently detected objects
        objectDetection.updateObservationPoseBuffer(
                10,
                objectPoseBuffer,
                0.4572,
                observation.get().objectPose().get().getTranslation());
        // Return latest Object observation (full)
        return observation;
    }

    // Private helper for generating latest Contour observation.
    private Optional<ObjectDetectionObservation> generateContourObservation(
            ContourSelectionMode selection) {
        // Attempt to generate partial Object record using Blob model
        Optional<ObjectDetectionObservation> observation =
                objectDetection.getContourObservation(objectDetection.getTargets(), selection);
        // Return latest Blob observation (partial or empty)
        return observation;
    }

    // Private helper for logging Object Detection values. Object ID = -9999 for stale values.
    private void logObjectObservation(Optional<ObjectDetectionObservation> observation) {
        // Logged calculations for sim
        if (observation.isPresent()
                && (observation.get().objectPose().isPresent()
                        && observation.get().distance().isPresent())) {
            Logger.recordOutput(
                    "Detection/" + "ML:" + "Latest Detection's Object ID",
                    observation.get().objID());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Latest Detection's Detection Confidence",
                    observation.get().confidence());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Latest Detection's Calculated Coordinates",
                    observation.get().objectPose().get().getTranslation());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Latest Detection's Calculated Distance",
                    observation.get().distance().get().in(Meters));

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Newest Coordinate Detection",
                    objectPoseBuffer.get(objectPoseBuffer.size() - 1));

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Oldest Coordinate Detection", objectPoseBuffer.get(0));

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Detection List Size", objectPoseBuffer.size());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Sim Target #0 True Range",
                    ObjectDetectorConstants.SIM_TARGETS[0]
                            .getPose()
                            .toPose2d()
                            .getTranslation()
                            .minus(robotState.getEstimatedPose().getTranslation())
                            .getX());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Sim Target #0 True Heading",
                    ObjectDetectorConstants.SIM_TARGETS[0]
                            .getPose()
                            .toPose2d()
                            .getTranslation()
                            .minus(robotState.getEstimatedPose().getTranslation())
                            .getY());

            Logger.recordOutput(
                    "Detection/" + "ML:" + "Sim Target #0 True Distance",
                    ObjectDetectorConstants.SIM_TARGETS[0]
                            .getPose()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(robotState.getEstimatedPose().getTranslation()));
        } else {
            Logger.recordOutput("Detection/" + "ML:" + "Latest Detection's Object ID", -9999);
        }
    }

    // Private helper for logging Contour values. Object ID = -9999 for stale values.
    private void logContourObservation(
            Optional<ObjectDetectionObservation> observation, ContourSelectionMode mode) {
        // Logged calculations for sim
        if (observation.isPresent()) {
            Logger.recordOutput(
                    "Detection/" + "Contour: " + mode + ": Object ID", observation.get().objID());

            Logger.recordOutput(
                    "Detection/" + "Contour: " + mode + ": Confidence",
                    observation.get().confidence());

            Logger.recordOutput(
                    "Detection/" + "Contour: " + mode + ": Pitch",
                    observation.get().pitch().in(Degrees));

            Logger.recordOutput(
                    "Detection/" + "Contour: " + mode + ": Yaw",
                    observation.get().yaw().in(Degrees));

            Logger.recordOutput(
                    "Detection/" + "Contour: " + mode + ": Area", observation.get().area());
        } else {
            Logger.recordOutput("Detection/" + "Contour: " + mode + ": Object ID", -9999);
        }
    }

    @Override
    public void periodic() {
        // Update the inputs data structure associated with this object detection camera with latest
        // readings
        objectDetection.periodic();
        // Now that inputs are updated, re-populate observations with new data
        if (objectDetection.getTargets().length > 0) {
            // Generate latest ML Object observation
            latestObjectObservation = generateObjectObservation(objectDetection.getTargets()[0]);
            // Generate latest Contour observations
            latestBigContourObservation = generateContourObservation(ContourSelectionMode.LARGEST);
            latestLowContourObservation = generateContourObservation(ContourSelectionMode.LOWEST);
        } else {
            // Prevent stale data from persisting
            latestObjectObservation = Optional.empty();
            latestBigContourObservation = Optional.empty();
            latestLowContourObservation = Optional.empty();
        }
        // Log Object & Blob observations for sim
        logObjectObservation(latestObjectObservation);
        logContourObservation(latestBigContourObservation, ContourSelectionMode.LARGEST);
        logContourObservation(latestLowContourObservation, ContourSelectionMode.LOWEST);
    }
}
